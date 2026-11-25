#pragma once

#include "json.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <map>
#include <set>
#include <stdexcept>
#include <string>

namespace Meridian {
using Json = nlohmann::json;
constexpr double Pi = 3.14159265358979323846;

inline double Wrap(double angle) { return angle - 360.0 * std::floor(angle / 360.0); }
inline double Delta(double from, double to) { return Wrap(to - from + 180.0) - 180.0; }

struct AircraftState {
    std::string id, type, op, livery, reg, squawk;
    double lat = 0, lon = 0, alt = 0, hdg = 0, pitch = 0, roll = 0, gs = 0, vs = 0;
    double gear = 1, flaps = 0;
    bool ground = false, taxi = false, land = false, beacon = true, strobe = false, nav = true;
};

inline std::string Text(const Json& j, const char* key, size_t max, bool required = false) {
    if (!j.contains(key) && !required) return {};
    auto s = j.at(key).get<std::string>();
    if ((required && s.empty()) || s.size() > max ||
        std::any_of(s.begin(), s.end(), [](unsigned char c) { return c < 32 || c == 127; }))
        throw std::invalid_argument(std::string("invalid ") + key);
    return s;
}

inline double Number(const Json& j, const char* key, double min, double max) {
    if (!j.at(key).is_number()) throw std::invalid_argument(key);
    double d = j.at(key).get<double>();
    if (!std::isfinite(d) || d < min || d > max) throw std::invalid_argument(key);
    return d;
}

inline bool Light(const Json& j, const char* key) {
    if (j.at(key).is_boolean()) return j.at(key).get<bool>();
    double value = Number(j, key, 0, 1);
    if (value != 0 && value != 1) throw std::invalid_argument(key);
    return value == 1;
}

struct FollowMe {
    bool on = false;
    double lat = 0, lon = 0, hdg = 0;
};

struct NativeAssign {
    std::string id, type, op;
    int slot = 0;
    // Where the flight is when the slot is claimed, degrees and feet,
    // so a fresh plane starts at the controller's position, not wherever
    // the slot was left. Missing numbers mean "no seed".
    bool pos = false;
    double lat = 0, lon = 0, ele = 0;
    double hdg = 0, gs = 0, vs = 0;
    bool ground = true;
};

struct Packet {
    std::string kind;
    uint64_t seq = 0;
    std::map<std::string, AircraftState> aircraft;
    // "native" roster: which ICAO type each AI-plane slot should be, so the
    // plugin can load default .acf models the engine then flies itself.
    std::vector<NativeAssign> native;
    // When the roster owns every slot it wants: an unclaimed slot stays
    // empty rather than being spawned into by the simulator's own AI.
    bool exclusive = false;
    FollowMe followme;
    // The pilot's cleared taxi route for the map overlay, lat/lon pairs.
    std::vector<std::pair<double, double>> route;
    bool routeOn = false;
    // A hello marked probe is read-only: it must never claim ownership.
    bool probe = false;
    // A METAR request's station.
    std::string icao;
    // A model-match question: what would XPMP2 draw this type/operator
    // as, and how well? ``q`` is the echo token pairing reply to request.
    std::string type, op, livery, cs;
    uint64_t q = 0;
};

inline Packet Parse(const std::string& wire) {
    if (wire.empty() || wire.size() > 65507) throw std::invalid_argument("datagram size");
    // Reject deeply nested JSON before parsing to keep flight-loop work bounded.
    int depth = 0;
    auto j = Json::parse(wire, [&depth](int, Json::parse_event_t event, Json&) {
        if (event == Json::parse_event_t::object_start || event == Json::parse_event_t::array_start) {
            if (++depth > 12) throw std::invalid_argument("JSON nesting");
        } else if (event == Json::parse_event_t::object_end || event == Json::parse_event_t::array_end) --depth;
        return true;
    });
    if (!j.is_object() || !j.at("v").is_number_integer() || j.at("v") != 1)
        throw std::invalid_argument("protocol version");
    Packet p;
    p.kind = Text(j, "t", 16, true);
    if (p.kind == "clear" || p.kind == "hello") {
        if (p.kind == "hello" && j.contains("probe") && j.at("probe").is_boolean())
            p.probe = j.at("probe").get<bool>();
        return p;
    }
    if (p.kind == "followme") {
        p.followme.on = Light(j, "on");
        p.followme.lat = Number(j, "lat", -90, 90);
        p.followme.lon = Number(j, "lon", -180, 180);
        p.followme.hdg = Wrap(Number(j, "hdg", -360, 360));
        return p;
    }
    if (p.kind == "route") {
        p.routeOn = Light(j, "on");
        const auto& pts = j.at("pts");
        if (!pts.is_array() || pts.size() > 512)
            throw std::invalid_argument("route points");
        for (const auto& pt : pts) {
            if (!pt.is_array() || pt.size() != 2 ||
                !pt[0].is_number() || !pt[1].is_number())
                throw std::invalid_argument("route point");
            double lat = pt[0].get<double>(), lon = pt[1].get<double>();
            if (!std::isfinite(lat) || !std::isfinite(lon) ||
                lat < -90 || lat > 90 || lon < -180 || lon > 180)
                throw std::invalid_argument("route point");
            p.route.emplace_back(lat, lon);
        }
        return p;
    }
    if (p.kind == "metar") {
        p.icao = Text(j, "icao", 8, true);
        for (const unsigned char c : p.icao)
            if (!std::isalnum(c)) throw std::invalid_argument("icao");
        return p;
    }
    if (p.kind == "match") {
        p.type = Text(j, "type", 4, true);
        p.op = Text(j, "op", 3); p.livery = Text(j, "livery", 128);
        p.cs = Text(j, "cs", 16);
        if (!j.at("q").is_number_unsigned())
            throw std::invalid_argument("match token");
        p.q = j.at("q").get<uint64_t>();
        return p;
    }
    if (p.kind == "native") {
        p.exclusive = j.contains("exclusive") && Light(j, "exclusive");
        const auto& ac = j.at("ac");
        if (!ac.is_array() || ac.size() > 63)
            throw std::invalid_argument("native roster");
        std::set<int> slots;
        for (const auto& a : ac) {
            NativeAssign assign;
            assign.id = Text(a, "id", 64, true);
            assign.type = Text(a, "type", 4, true);
            assign.op = Text(a, "op", 3);
            if (!a.at("slot").is_number_integer()) throw std::invalid_argument("slot");
            assign.slot = a.at("slot").get<int>();
            if (assign.slot < 1 || assign.slot > 63 ||
                !slots.insert(assign.slot).second)
                throw std::invalid_argument("slot");
            if (a.contains("lat") || a.contains("lon") || a.contains("alt")) {
                assign.lat = Number(a, "lat", -90, 90);
                assign.lon = Number(a, "lon", -180, 180);
                assign.ele = Number(a, "alt", -2000, 100000);
                if (a.contains("hdg")) assign.hdg = Number(a, "hdg", -360, 720);
                if (a.contains("gs")) assign.gs = Number(a, "gs", 0, 1200);
                if (a.contains("vs")) assign.vs = Number(a, "vs", -30000, 30000);
                if (a.contains("gnd") && a.at("gnd").is_boolean()) assign.ground = a.at("gnd").get<bool>();
                assign.pos = true;
            }
            p.native.push_back(std::move(assign));
        }
        return p;
    }
    if (p.kind != "traffic" || !j.at("seq").is_number_unsigned())
        throw std::invalid_argument("message type or sequence");
    p.seq = j.at("seq").get<uint64_t>();
    const auto& ac = j.at("ac");
    if (!ac.is_array() || ac.size() > 512) throw std::invalid_argument("aircraft array");
    for (const auto& a : ac) {
        AircraftState s;
        s.id = Text(a, "id", 64, true);
        s.type = Text(a, "type", 4, true);
        s.op = Text(a, "op", 3); s.livery = Text(a, "livery", 128); s.reg = Text(a, "reg", 32);
        s.squawk = Text(a, "squawk", 4, true);
        if (s.squawk.size() != 4 || s.squawk.find_first_not_of("01234567") != std::string::npos)
            throw std::invalid_argument("squawk");
        s.lat = Number(a, "lat", -90, 90); s.lon = Number(a, "lon", -180, 180);
        s.alt = Number(a, "alt_ft", -2000, 100000);
        s.hdg = Wrap(Number(a, "hdg", -360, 360));
        s.pitch = Number(a, "pitch", -90, 90); s.roll = Number(a, "roll", -180, 180);
        s.gs = Number(a, "gs_kt", 0, 2000); s.vs = Number(a, "vs_fpm", -30000, 30000);
        s.ground = a.at("gnd").get<bool>();
        s.gear = Number(a, "gear", 0, 1); s.flaps = Number(a, "flaps", 0, 1);
        const auto& lights = a.at("lights");
        s.taxi = Light(lights, "taxi"); s.land = Light(lights, "land");
        s.beacon = Light(lights, "bcn"); s.strobe = Light(lights, "strb"); s.nav = Light(lights, "nav");
        if (!p.aircraft.emplace(s.id, s).second) throw std::invalid_argument("duplicate id");
    }
    return p;
}

// Where the last report's speed and turn take the picture next:
// groundspeed and heading rates carried forward -- braking runs to its
// stop instead of being projected through it, and a corner keeps turning
// instead of being straightened the moment the feed hesitates. Speed
// below zero is a stop, not a rollback; the turn decays to straight over
// a second, because a turn that was finishing is not one that continues.
inline AircraftState Project(AircraftState s, double seconds,
                             double gsRate = 0, double hdgRate = 0) {
    double travel = s.gs * seconds, heading = s.hdg;
    if (gsRate < 0 && s.gs + gsRate * seconds < 0) {
        double stop = -s.gs / gsRate;
        travel = s.gs * stop + 0.5 * gsRate * stop * stop;
    } else {
        travel += 0.5 * gsRate * seconds * seconds;
    }
    if (hdgRate != 0) heading += hdgRate * (1.0 - std::exp(-seconds));
    double angular = travel * 1852.0 / 3600.0 / 6371000.0;
    double lat = s.lat * Pi / 180, lon = s.lon * Pi / 180, hdg = heading * Pi / 180;
    double nextLat = std::asin(std::clamp(std::sin(lat) * std::cos(angular) +
                          std::cos(lat) * std::sin(angular) * std::cos(hdg), -1.0, 1.0));
    lon += std::atan2(std::sin(hdg) * std::sin(angular) * std::cos(lat),
                     std::cos(angular) - std::sin(lat) * std::sin(nextLat));
    s.lat = nextLat * 180 / Pi; s.lon = Delta(0, lon * 180 / Pi);
    s.hdg = Wrap(heading);
    if (!s.ground) s.alt += s.vs * seconds / 60;
    return s;
}

class Motion {
    std::deque<std::pair<double, AircraftState>> samples;
public:
    explicit Motion(const AircraftState& s, double now) { samples.emplace_back(now, s); }
    void Update(const AircraftState& s, double now) {
        if (now <= samples.back().first) samples.back().second = s;
        else samples.emplace_back(now, s);
        while (samples.size() > 64 || (samples.size() > 2 && now - samples[1].first > 4)) samples.pop_front();
    }
    // The rates the last pair of reports made: knots a second and
    // degrees a second. Single-point aircraft get no continuation, and
    // a pair older than five seconds apart is too stale to trust.
    std::pair<double, double> Trend() const {
        if (samples.size() < 2) return {0, 0};
        const auto& [t0, a] = *(samples.end() - 2);
        const auto& [t1, b] = samples.back();
        double dt = t1 - t0;
        if (dt <= 0.05 || dt > 5.0) return {0, 0};
        return {std::clamp((b.gs - a.gs) / dt, -25.0, 25.0),
                std::clamp(Delta(a.hdg, b.hdg) / dt, -45.0, 45.0)};
    }
    AircraftState Sample(double now) const {
        double renderAt = now - .2;
        const auto& [currentAt, current] = samples.back();
        if (renderAt >= currentAt) {
            const auto& [gsRate, hdgRate] = Trend();
            // On the ground the next word is due soon; in the air a
            // gap can run longer before the picture freezes.
            double cap = current.ground ? 1.0 : 2.0;
            return Project(current, std::clamp(renderAt - currentAt, 0.0, cap),
                           gsRate, hdgRate);
        }
        if (renderAt <= samples.front().first) return samples.front().second;
        auto next = std::upper_bound(samples.begin(), samples.end(), renderAt,
            [](double t, const auto& sample) { return t < sample.first; });
        const auto& [previousAt, previous] = *std::prev(next);
        const auto& [nextAt, target] = *next;
        double f = (renderAt - previousAt) / (nextAt - previousAt);
        auto s = target;
        s.lat = previous.lat + (target.lat - previous.lat) * f;
        s.lon = Delta(0, previous.lon + Delta(previous.lon, target.lon) * f);
        s.alt = previous.alt + (target.alt - previous.alt) * f;
        s.hdg = Wrap(previous.hdg + Delta(previous.hdg, target.hdg) * f);
        s.pitch = previous.pitch + (target.pitch - previous.pitch) * f;
        s.roll = previous.roll + Delta(previous.roll, target.roll) * f;
        return s;
    }
};

class Session {
    bool hasSequence = false;
    uint64_t sequence = 0;
    double trafficAt = -1e9;
    double followmeAt = -1e9;
    double routeAt = -1e9;
    bool ownerKnown = false;
    uint64_t owner = 0;
    double ownerAt = -1e9;
public:
    std::map<std::string, AircraftState> snapshot;
    FollowMe followme;
    std::vector<std::pair<double, double>> route;
    bool Connected(double now) const { return ownerKnown && now - ownerAt < 10; }
    // A restarted engine announcing itself takes the session over once
    // the old owner has gone quiet -- its sequence numbers start from
    // zero again, and without this the stale-seq guard would mute it
    // until the owner timed out. An owner still accepted within the
    // last half second is alive and keeps the session, so a status
    // probe's hello cannot wipe drawn traffic or mute real packets.
    void Claim(uint64_t sender, double now) {
        if (sender == owner && ownerKnown) return;
        if (Connected(now) && now - ownerAt < .5) return;
        Clear();
        owner = sender; ownerKnown = true; ownerAt = now;
    }
    // A silent engine leaves the car where it last stood for a few seconds,
    // then pulls it -- a parked car on the taxiway is worse than none.
    bool FollowmeActive(double now) const { return followme.on && now - followmeAt < 15; }
    // The route outlives the car -- a drawn line on the map costs nothing
    // and losing it mid-taxi is worse than a stale one for a few seconds.
    bool RouteActive(double now) const { return !route.empty() && now - routeAt < 20; }
    bool Apply(const Packet& packet, double now, uint64_t sender = 0) {
        if (packet.kind == "hello") return true;
        if (Connected(now) && sender != owner) return false;
        if (!Connected(now)) Clear();
        if (packet.kind == "traffic" && now - trafficAt < 10 && hasSequence && packet.seq <= sequence) return false;
        if (packet.kind == "clear") Clear();
        else if (packet.kind == "followme") {
            followme = packet.followme; followmeAt = now;
        } else if (packet.kind == "route") {
            if (packet.routeOn) route = packet.route;
            else route.clear();
            routeAt = now;
        } else {
            snapshot = packet.aircraft;
            trafficAt = now; sequence = packet.seq; hasSequence = true;
        }
        owner = sender; ownerKnown = true; ownerAt = now;
        return true;
    }
    bool Expire(double now) {
        if (now - trafficAt < 10 || !hasSequence) return false;
        ClearTraffic(); return true;
    }
    void ClearTraffic() {
        snapshot.clear(); hasSequence = false; trafficAt = -1e9;
    }
    void Clear() {
        ClearTraffic();
        followme.on = false; followmeAt = -1e9;
        route.clear(); routeAt = -1e9;
    }
};
}
