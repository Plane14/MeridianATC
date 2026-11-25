#include "Udp.h"
#include "Traffic.h"
#include "XPMPAircraft.h"
#include "XPLMDataAccess.h"
#include "XPLMGraphics.h"
#include "XPLMInstance.h"
#include "XPLMMap.h"
#include "XPLMMenus.h"
#include "XPLMNavigation.h"
#include "XPLMPlanes.h"
#include "XPLMPlugin.h"
#include "XPLMProcessing.h"
#include "XPLMScenery.h"
#include "XPLMUtilities.h"
#include "XPLMWeather.h"
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>
#include <set>
#include <vector>
#if defined(_WIN32)
#include <Windows.h>
#endif
#if defined(__APPLE__)
#include <OpenGL/gl.h>
#include <OpenGL/OpenGL.h>
#elif defined(_WIN32)
#include <GL/gl.h>
#else
#include <GL/gl.h>
#include <GL/glx.h>
#endif

namespace {
using namespace Meridian;
namespace fs = std::filesystem;
constexpr const char* Version = "0.2.0";
double Now() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
void Log(const std::string& text) { XPLMDebugString(("MeridianATC-XP: " + text + "\n").c_str()); }
int Preferences(const char*, const char* key, int fallback) {
    return std::string(key) == XPMP_CFG_ITM_SUPPORT_REMOTE ? -1 : fallback;
}
struct Config {
    int port = 49010;
    bool labels = true, tcas = true;
    std::vector<fs::path> csl;
} config;
fs::path root;
bool initialized = false, visible = true;
double lastStatus = -1e9, lastNativePositions = -1e9;
Udp udp;
sockaddr_in peer{};
Session session;
XPLMFlightLoopID loop = nullptr;
XPLMMenuID menu = nullptr;
int menuIndex = -1;
XPLMDataRef connectedRef = nullptr, aircraftRef = nullptr;
std::map<std::string, XPLMCommandRef> commands;

// The cleared taxi route, drawn on X-Plane's own map as a markings layer.
// The engine streams the lat/lon polyline over the same datagram channel
// as the follow-me car; the layer projects it each redraw.
std::set<XPLMMapLayerID> mapLayers;

// The draw callback gets an OpenGL context only while X-Plane itself runs
// the GL driver -- under Vulkan or Metal there is none, and raw GL calls
// are silently lost. Asked once per redraw, logged once ever.
bool ContextCurrent() {
#if defined(_WIN32)
    return wglGetCurrentContext() != nullptr;
#elif defined(__APPLE__)
    return CGLGetCurrentContext() != nullptr;
#else
    return glXGetCurrentContext() != nullptr;
#endif
}
bool warnedNoGL = false;

void DrawRouteLayer(XPLMMapLayerID, const float*, float, float,
                    XPLMMapStyle, XPLMMapProjectionID projection, void*) {
    if (!session.RouteActive(Now()) || session.route.size() < 2) return;
    if (!ContextCurrent()) {
        if (!warnedNoGL) {
            warnedNoGL = true;
            Log("Taxi-route map layer needs the OpenGL graphics driver");
        }
        return;
    }
    XPLMSetGraphicsState(0, 0, 0, 0, 0, 1, 0);
    glColor4f(0.1f, 0.8f, 0.9f, 0.9f);
    glLineWidth(4.0f);
    glBegin(GL_LINE_STRIP);
    for (const auto& [lat, lon] : session.route) {
        float x = 0.0f, y = 0.0f;
        XPLMMapProject(projection, lat, lon, &x, &y);
        glVertex2f(x, y);
    }
    glEnd();
}

void LayerDeleted(XPLMMapLayerID layer, void*) { mapLayers.erase(layer); }

void MapOpened(const char* identifier, void*) {
    // The hook stays registered while disabled -- there is no unregister --
    // so a map opening between Disable and Enable must not revive the layer.
    if (!initialized || std::string(identifier) != XPLM_MAP_USER_INTERFACE)
        return;
    XPLMCreateMapLayer_t params{};
    params.structSize = sizeof(params);
    params.mapToCreateLayerIn = identifier;
    params.layerType = xplm_MapLayer_Markings;
    params.willBeDeletedCallback = LayerDeleted;
    params.drawCallback = DrawRouteLayer;
    params.showUiToggle = 1;
    params.layerName = "MeridianATC taxi route";
    if (XPLMMapLayerID layer = XPLMCreateMapLayer(&params))
        mapLayers.insert(layer);
}

void AnswerMetar(const std::string& icao, const sockaddr_in& to) {
    // The report the simulator itself downloaded -- its own weather's
    // truth for the ATIS. Empty when the sim is not in real-weather
    // mode; the caller falls back to its other sources.
    XPLMFixedString150_t metar{};
    XPLMGetMETARForAirport(icao.c_str(), &metar);
    metar.buffer[sizeof(metar.buffer) - 1] = '\0';
    Json reply = {{"v", 1}, {"t", "metar"}, {"icao", icao},
                  {"metar", std::string(metar.buffer)}};
    udp.Send(reply.dump(), to);
}

void SendCommand(const std::string& cmd, bool down = true) {
    if (!session.Connected(Now())) return;
    std::string wire = std::string("{\"v\":1,\"t\":\"command\",\"cmd\":\"") + cmd +
        "\",\"down\":" + (down ? "true" : "false") + "}";
    udp.Send(wire, peer);
}
int OnCommand(XPLMCommandRef ref, XPLMCommandPhase phase, void*) {
    if (!initialized) return 1;
    try {
        for (const auto& [name, candidate] : commands) {
            if (candidate != ref) continue;
            const bool ptt = name.find("push_to_talk") != std::string::npos ||
                name.find("/talk") != std::string::npos;
            if (ptt && phase == xplm_CommandContinue) break;
            SendCommand(name.substr(name.find_last_of('/') + 1),
                        ptt ? phase == xplm_CommandBegin : true);
            break;
        }
    } catch (const std::exception& e) { Log(e.what()); }
    return 1;
}

class TrafficAircraft : public XPMP2::Aircraft {
    Motion motion;
    XPLMProbeRef probe = nullptr;
public:
    std::string modelKey;
    // What the sim was last told to draw -- the status report's
    // read-back answers this against the drawn position, the way
    // SimConnect's own object reads do on the other sim.
    AircraftState applied;
    // The drawn reference point's height over the probed terrain,
    // feet; -1 until the first ground probe answers.
    double agl_ft = -1.0;
    TrafficAircraft(const AircraftState& s, double now) :
        Aircraft(s.type, s.op, s.livery, 0, "", s.id), motion(s, now),
        modelKey(s.type + "\n" + s.op + "\n" + s.livery) { Apply(s, now, false); }
    ~TrafficAircraft() override { if (probe) XPLMDestroyProbe(probe); }
    void Apply(const AircraftState& s, double now, bool update = true) {
        applied = s;
        if (update) motion.Update(s, now);
        label = s.id;
        std::snprintf(acInfoTexts.tailNum, sizeof(acInfoTexts.tailNum), "%s", s.reg.c_str());
        std::snprintf(acInfoTexts.icaoAcType, sizeof(acInfoTexts.icaoAcType), "%s", s.type.c_str());
        std::snprintf(acInfoTexts.icaoAirline, sizeof(acInfoTexts.icaoAirline), "%s", s.op.c_str());
        std::snprintf(acInfoTexts.airline, sizeof(acInfoTexts.airline), "%s", s.op.c_str());
        std::snprintf(acInfoTexts.flightNum, sizeof(acInfoTexts.flightNum), "%s", s.id.c_str());
        // A squawk that is not a number (empty, "----", a feed quirk) must
        // not throw here: the exception out of Apply erases the whole
        // aircraft. Default 2000, the unassigned code, like the engine's.
        try { acRadar.code = std::stol(s.squawk); }
        catch (const std::exception&) { acRadar.code = 2000; }
        acRadar.mode = xpmpTransponderMode_ModeC;
        SetGearRatio(static_cast<float>(s.gear)); SetFlapRatio(static_cast<float>(s.flaps));
        SetThrustRatio(static_cast<float>(std::clamp(s.gs / (s.ground ? 40.0 : 250.0), 0.0, 1.0)));
        SetLightsTaxi(s.taxi); SetLightsLanding(s.land); SetLightsBeacon(s.beacon);
        SetLightsStrobe(s.strobe); SetLightsNav(s.nav);
        SetOnGrnd(s.ground); bClampToGround = s.ground;
        SetVisible(visible);
    }
    void UpdatePosition(float, int) override {
        auto s = motion.Sample(Now());
        if (s.ground) {
            if (!probe) probe = XPLMCreateProbe(xplm_ProbeY);
            double x, y, z;
            XPLMWorldToLocal(s.lat, s.lon, s.alt * .3048, &x, &y, &z);
            XPLMProbeInfo_t info{}; info.structSize = sizeof(info);
            if (probe && XPLMProbeTerrainXYZ(probe, static_cast<float>(x), static_cast<float>(y),
                                           static_cast<float>(z), &info) == xplm_ProbeHitTerrain) {
                double lat, lon, altitude;
                XPLMLocalToWorld(info.locationX, info.locationY, info.locationZ, &lat, &lon, &altitude);
                agl_ft = s.alt - altitude / .3048;
                s.alt = altitude / .3048;
            }
        }
        SetLocation(s.lat, s.lon, s.alt);
        SetHeading(static_cast<float>(s.hdg)); SetPitch(static_cast<float>(s.pitch)); SetRoll(static_cast<float>(s.roll));
    }
};
std::map<std::string, std::unique_ptr<TrafficAircraft>> aircraft;
std::set<std::string> unmatched;

void TCAS();

// --- X-Plane's own aircraft, flown by the sim's flight model ----------
// The engine steers these through the multiplayer control datarefs; the
// plugin's share is what no dataref can do: acquiring the planes, loading
// a default-fleet .acf into each slot, and latching the AI-autopilot
// override. Whether XPMP2's TCAS override currently owns the planes --
// ownership is exclusive, so native traffic and the override swap turns.
bool xpmpOwned = false;
bool nativeAcquired = false, nativeBlocked = false;
std::map<int, std::pair<std::string, std::string>> nativeSlots; // slot -> {id, acf}
XPLMDataRef ovrAuto = nullptr, ovrPath = nullptr;
std::vector<fs::path> acfList;
// Every installed .acf, indexed by the ICAO designator written inside it.
std::map<std::string, std::vector<fs::path>> acfByIcao;

void ScanAcf() {
    if (!acfList.empty()) return;
    char system[1024]{}; XPLMGetSystemPath(system);
    std::error_code ec;
    const fs::path base = fs::path(system) / "Aircraft";
    for (auto it = fs::recursive_directory_iterator(base, ec);
         !ec && it != fs::recursive_directory_iterator(); it.increment(ec)) {
        if (!it->is_regular_file(ec) || it->path().extension() != ".acf")
            continue;
        const fs::path p = it->path();
        // Austin's testbeds are no aeroplane to look at, and an add-on's
        // plugin-side helper files are not real aircraft either.
        const std::string rel = p.lexically_relative(base).generic_string();
        if (rel.rfind("Experimental/", 0) == 0 ||
            rel.find("/plugins/") != std::string::npos)
            continue;
        acfList.push_back(p);
        // XP12 .acf files are a text properties stream: the designator the
        // airframe declares is the name every roster type maps onto.
        std::ifstream in(p);
        std::string line;
        while (std::getline(in, line)) {
            // A CRLF save keeps its \r on any platform's getline; the
            // designator must not end up keyed as "A320\r".
            while (!line.empty() && (line.back() == '\r' ||
                                     line.back() == ' '))
                line.pop_back();
            if (line.rfind("P acf/_ICAO ", 0) == 0 && line.size() > 12) {
                acfByIcao[line.substr(12)].push_back(p);
                break;
            }
        }
    }
    Log("installed airframes: " + std::to_string(acfList.size()) +
        " across " + std::to_string(acfByIcao.size()) + " types");
}
fs::path FindAcf(const std::string& needle) {
    for (const auto& p : acfList)
        if (p.filename().string().find(needle) != std::string::npos) return p;
    return {};
}

// Which file of a type's candidates an AI slot should fly. Purpose-built
// AI airframes (Dazzy_AI, Laminar's *_AI) carry a fraction of a study
// add-on's geometry and scripts; the default fleet is next best; a livery
// or weapon variant of the same airframe is what a civilian should not be.
int AcfScore(const fs::path& p) {
    const std::string s = p.generic_string(), f = p.filename().string();
    int score = s.find("/Dazzy_AI/") != std::string::npos ? 0
        : f.find("_AI.") != std::string::npos ? 0
        : s.find("/Laminar Research/") != std::string::npos ? 1 : 2;
    for (const char* bad : {"seaplane", "mariner", "Float", "floats",
                            "float", "Amphib", "JATO", "Formation",
                            "Bomber", "Ground_Attack", "Interceptor",
                            "NASA", "USAF"})
        if (f.find(bad) != std::string::npos ||
            s.find(bad) != std::string::npos) score += 4;
    return score;
}
fs::path BestIcao(const std::string& code, bool freighter = false) {
    auto it = acfByIcao.find(code);
    if (it == acfByIcao.end() || it->second.empty()) return {};
    auto best = std::min_element(it->second.begin(), it->second.end(),
        [](const fs::path& a, const fs::path& b) {
            return AcfScore(a) < AcfScore(b); });
    // A freighter callsign gets the cargo frame where the type offers one.
    if (freighter)
        for (const auto& c : it->second) {
            const std::string f = c.filename().string();
            if (f.find("AIF") != std::string::npos ||
                f.find("Cargo") != std::string::npos)
                return c;
        }
    return *best;
}

std::string ModelFor(const std::string& type, const std::string& op) {
    const bool freighter = op == "FDX" || op == "UPS" || op == "GEC"
        || op == "BOX" || op == "CLX" || op == "ABW" || op == "DHL";
    // 1. The airframe itself: the type's own designator, then the codes its
    //    family shares a flight model with.
    static const std::map<std::string, std::vector<const char*>> family = {
        {"A318", {"A320", "A319"}}, {"A319", {"A320", "A318"}},
        {"A20N", {"A320", "A20N"}}, {"A21N", {"A321", "A320", "A20N"}},
        {"A321", {"A320", "A319", "A20N"}}, {"A319CJ", {"A319", "A320"}},
        {"A332", {"A333", "A339"}}, {"A338", {"A339", "A333"}},
        {"A342", {"A343"}}, {"A359", {"A333", "A339", "A343"}},
        {"A35K", {"A359", "A333"}}, {"A306", {"B763", "A333"}},
        {"B732", {"B733", "B738"}}, {"B733", {"B738"}},
        {"B734", {"B738", "B733"}}, {"B735", {"B733", "B738"}},
        {"B736", {"B738", "B733"}}, {"B737", {"B738", "B733"}},
        {"B38M", {"B738"}}, {"B37M", {"B738"}}, {"B39M", {"B738"}},
        {"B739", {"B738"}}, {"B73G", {"B738"}},
        {"B742", {"B744"}}, {"B743", {"B744"}}, {"B741", {"B744"}},
        {"B748", {"B744"}}, {"B74S", {"B744"}}, {"B74D", {"B744"}},
        {"B753", {"B752"}}, {"B762", {"B763"}}, {"B764", {"B763"}},
        {"B772", {"B77W", "B77L"}}, {"B773", {"B77W", "B772"}},
        {"B77W", {"B772", "B77L"}}, {"B778", {"B772", "B77W"}},
        {"B779", {"B772", "B77W"}}, {"B788", {"B772", "B763"}},
        {"B789", {"B772", "B77W"}}, {"B78X", {"B772", "B77W"}},
        {"B712", {"B717"}}, {"B717", {"B712", "MD82"}},
        {"B721", {"B722"}}, {"B722", {"B721"}}, {"B720", {"B721"}},
        {"MD81", {"MD82"}}, {"MD83", {"MD82"}}, {"MD87", {"MD82"}},
        {"MD88", {"MD82"}}, {"MD90", {"MD82"}}, {"MD11", {"B767"}},
        {"DC10", {"B767"}}, {"E135", {"E170"}}, {"E145", {"E170"}},
        {"E170", {"E190"}}, {"E75L", {"E170", "E190"}},
        {"E195", {"E190"}}, {"E290", {"E190", "E170"}},
        {"E295", {"E190", "E170"}}, {"CRJ1", {"CRJ9", "E170"}},
        {"CRJ2", {"CRJ9", "E170"}}, {"CRJ7", {"CRJ9", "E170"}},
        {"CRJ9", {"CRJ7", "E170", "MD82"}}, {"CRJX", {"CRJ9", "E170"}},
        {"AT43", {"AT45", "AT76", "DH8D"}}, {"AT45", {"AT76", "DH8D"}},
        {"AT46", {"AT45", "AT76"}}, {"AT72", {"AT76", "AT45"}},
        {"AT73", {"AT76", "AT45"}}, {"AT75", {"AT76", "AT45"}},
        {"AT76", {"AT45", "DH8D"}}, {"DH8A", {"DH8D", "AT45"}},
        {"DH8B", {"DH8D", "AT45"}}, {"DH8C", {"DH8D", "AT45"}},
        {"SF34", {"AT45", "DH8D"}}, {"SW4", {"AT45", "B190"}},
        {"JS31", {"JS32", "B190"}}, {"JS32", {"JS31", "B190"}},
        {"JS41", {"JS32", "JS31"}}, {"E110", {"B190", "B18T"}},
        {"B190", {"BE9L", "JS32"}}, {"D328", {"AT45", "JS32"}},
        {"DH2T", {"DHC6", "DHC7"}}, {"DHC2", {"PA18", "C208"}},
        {"DHC3", {"C208", "DHC6"}}, {"C208", {"DHC6", "KODI"}},
        {"PC12", {"C208", "KODI"}}, {"TBM7", {"C208", "KODI"}},
        {"TBM8", {"C208", "KODI"}}, {"TBM9", {"C208", "KODI"}},
        {"KODI", {"C208", "PC12"}}, {"C25A", {"C750", "C56X"}},
        {"C25B", {"C56X", "C750"}}, {"C25C", {"C56X", "C750"}},
        {"C510", {"SF50", "C750"}}, {"C525", {"SF50", "C750"}},
        {"C550", {"C56X", "C750"}}, {"C560", {"C56X", "C750"}},
        {"C56X", {"C750", "C560"}}, {"C650", {"C750"}},
        {"C680", {"C750", "C56X"}}, {"C700", {"C750"}},
        {"C750", {"C56X"}}, {"LJ35", {"C750", "SF50"}},
        {"LJ45", {"C750", "SF50"}}, {"LJ60", {"C750"}},
        {"GLF4", {"C750", "F2TH"}}, {"GLF5", {"GLF4", "C750"}},
        {"GLF6", {"GLF4", "C750"}}, {"F2TH", {"C750", "GLF4"}},
        {"F900", {"F2TH", "C750"}}, {"FA7X", {"F2TH", "C750"}},
        {"C152", {"C172"}}, {"C162", {"C172", "C152"}},
        {"C170", {"C172"}}, {"C172", {"C152", "C170", "P28A"}},
        {"C177", {"C172", "C182"}}, {"C182", {"C172", "C170"}},
        {"C206", {"C172", "C208"}}, {"C210", {"C172", "BE36"}},
        {"PA18", {"PA18", "C172"}}, {"J3", {"PA18"}},
        {"P28A", {"C172", "PA28"}}, {"PA28", {"P28A", "C172"}},
        {"P28R", {"P28A", "BE36"}}, {"PA32", {"P28A", "BE36"}},
        {"PA38", {"C152", "PA18"}}, {"PA44", {"BE58", "PA31"}},
        {"PA46", {"BE36", "P28A"}}, {"DA40", {"SR22", "C172"}},
        {"DA42", {"BE58", "BE60"}}, {"DA62", {"BE58", "BE60"}},
        {"PA34", {"BE58", "BE60", "PA31"}}, {"PA31", {"BE58", "BE60"}},
        {"BE36", {"BE58", "P28A"}}, {"BE35", {"BE36"}},
        {"BE33", {"BE36"}}, {"BE55", {"BE58", "BE60"}},
        {"BE58", {"BE60", "BE95", "BE36"}}, {"BE60", {"BE58"}},
        {"BE9L", {"BE9K", "B190"}}, {"BE9K", {"BE9L", "B190"}},
        {"BE20", {"BE9L", "B190"}}, {"B350", {"BE9L", "B190"}},
        {"SR22", {"SR20", "DA40", "C172"}}, {"SR20", {"SR22"}},
        {"TB20", {"DR40", "P28A", "BE36"}}, {"TB21", {"TB20", "DR40"}},
        {"DR40", {"TB20", "P28A", "C172"}}, {"RV7", {"RV10", "PA18"}},
        {"RV10", {"RV7", "BE36"}}, {"EVOT", {"ALIA", "SF50"}},
        {"DC3", {"DC3T", "BT67", "C46"}}, {"DC3T", {"DC3"}},
        {"BT67", {"DC3T", "DC3"}}, {"C46", {"DC3"}},
        {"DHC6", {"SC7", "DHC7"}}, {"SC7", {"DHC6", "TRIS"}},
        {"TRIS", {"DHC6", "SC7"}}, {"BN2P", {"DHC6", "SC7"}},
        {"P180", {"BE58", "JS32"}}, {"JS32", {"JS31", "B190"}},
        {"EUFI", {"F4", "F104"}}, {"F15", {"F4", "F18"}},
        {"F16", {"F18", "F4"}}, {"F18", {"F4"}}, {"F22", {"F18"}},
        {"A10", {"F4"}}, {"HAR", {"AV8B", "F4"}}, {"AV8B", {"HAR"}},
        {"AS21", {"ASK", "GLID"}}, {"GLID", {"AS21"}},
        {"B06", {"B407", "Jetranger"}}, {"B407", {"B06"}},
        {"B429", {"B407", "B06"}}, {"B412", {"B407", "UH1"}},
        {"UH1", {"B06", "B412"}}, {"H500", {"R22", "B06"}},
        {"R44", {"R22"}}, {"R66", {"R44", "R22"}},
        {"AS50", {"B06", "EC30"}}, {"AS55", {"AS50", "EC35"}},
        {"EC30", {"EC35", "B06"}}, {"EC35", {"EC30", "AS50"}},
        {"EC45", {"EC35"}}, {"H125", {"AS50", "B06"}},
        {"H145", {"EC45", "B412"}}, {"H160", {"S76", "EC45"}},
        {"AS32", {"S76", "B412"}}, {"A109", {"S76", "B429"}},
        {"A139", {"S76", "A109"}}, {"MD52", {"MD60", "B407"}},
        {"MD60", {"MD52", "B06"}}, {"H47", {"S76", "CH53"}},
        {"H53", {"CH53", "S76"}}, {"H64", {"S76"}},
        {"V22", {"S76", "B06"}}, {"ALIA", {"EVOT", "S76"}},
    };
    static const std::set<std::string> heliTypes = {
        "R22", "R44", "R66", "S76", "B407", "B429", "B06", "B412",
        "AS50", "AS55", "AS32", "EC30", "EC35", "EC45", "A109",
        "A139", "MD52", "MD60", "UH1", "H500", "H160", "H145",
        "H47", "H53", "H64", "H125", "V22", "ALIA", "EVOT",
    };
    const bool heli = heliTypes.count(type) != 0;
    auto pick = [&](const char* code) -> fs::path {
        return code ? BestIcao(code, freighter) : fs::path{}; };
    if (auto p = pick(type.c_str()); !p.empty()) return p.string();
    if (auto fit = family.find(type); fit != family.end()) {
        fs::path best;
        for (const char* code : fit->second)
            if (auto p = pick(code); !p.empty() &&
                (best.empty() || AcfScore(p) < AcfScore(best)))
                best = p;
        if (!best.empty()) return best.string();
    }
    // Rotorcraft a family cannot cover default to the installed wheeled
    // medium, then the light skid trainer.
    if (heli)
        for (const char* code : {"S76", "B06", "B407", "R22", "ALIA"})
            if (auto p = pick(code); !p.empty()) return p.string();
    // 2. The default fleet's filenames, for a type index the scan found no
    //    _ICAO for and for an install without the default fleet's _ICAO.
    static const std::pair<const char*, const char*> table[] = {
        {"A33", "A330"}, {"MD8", "MD-82"},
        {"SR2", "SR22"}, {"SF5", "SF50"},
        {"CIT", "Citation"}, {"C75", "Citation"},
        {"B58", "Baron 58"}, {"BE5", "Baron"},
        {"BE9", "King Air"}, {"C90", "King Air"},
        {"PA1", "Super Cub"}, {"RV1", "RV-10"}, {"EVO", "Evolution"},
        {"A10", "Aerolite"}, {"F14", "Tomcat"},
        {"ASK", "ASK"}, {"GLI", "ASK"},
        {"R22", "R22"}, {"R44", "R22"}, {"S76", "S-76"},
        {"EVT", "ALIA"}, {"ALI", "ALIA"},
        {"C17", "172"}, {"C15", "172"}, {"C18", "172"},
        {"PA2", "172"}, {"P28", "172"},
    };
    // Every needle here is a fixed wing; a rotorcraft ("A109" begins "A10")
    // that reached this far goes straight to the helicopter fallback.
    if (!heli)
        for (const auto& [key, needle] : table)
            if (type.find(key) == 0)
                if (auto p = FindAcf(needle); !p.empty()) return p.string();
    // A rotorcraft the table did not name still gets a helicopter -- never
    // the fixed-wing stand-in below, whose controls it could not fly.
    if (heli)
        for (const char* needle : {"S-76", "R22", "ALIA"})
            if (auto p = FindAcf(needle); !p.empty()) return p.string();
    // Stand-in by class: anything airliner-sized wears the biggest jet the
    // install has, light turboprops a turboprop, everything else the
    // Skyhawk, a helicopter whatever rotorcraft exists.
    static const std::set<std::string> propTwin = {"AT7", "DH8", "DHC",
        "SW4", "SF3", "JS4", "E110", "B190"};
    const char* needle = type.size() && (type[0] == 'A' || type[0] == 'B'
            || type[0] == 'E' || type[0] == 'D' || type[0] == 'M')
        && !propTwin.count(type) ? "737-800"
        : propTwin.count(type) || type.find("TBM") == 0 ? "King Air" : "172";
    for (const char* try_ : {needle, "172", "737-800"})
        if (auto p = FindAcf(try_); !p.empty()) return p.string();
    return acfList.empty() ? std::string{} : acfList.front().string();
}

void EnableNative() {
    if (nativeAcquired || nativeBlocked) return;
    // Plane ownership is exclusive: XPMP2 holds it for the TCAS override,
    // so native traffic asks it to let go first and hands it back later.
    if (xpmpOwned) { XPMPMultiplayerDisable(); xpmpOwned = false; }
    nativeAcquired = XPLMAcquirePlanes(nullptr, nullptr, nullptr) != 0;
    nativeBlocked = !nativeAcquired;
    // One failure must not block the mode forever: the next empty roster
    // or session reset clears the flag and lets it retry. While it lasts
    // the CSL drawings still fly, so take the override back rather than
    // black the whole renderer out for want of the slots.
    if (nativeBlocked) {
        Log("AI planes unavailable -- another plugin owns them");
        if (config.tcas && !xpmpOwned) TCAS();
    }
}
void SeedPosition(int slot, const NativeAssign& assign) {
    // A newly claimed slot starts where the controller put the flight,
    // not where the slot's last occupant left it. One write per axis;
    // the flight model takes it from there.
    if (!assign.pos) return;
    double x = 0, y = 0, z = 0;
    XPLMWorldToLocal(assign.lat, assign.lon, assign.ele * 0.3048, &x, &y, &z);
    const double axes[] = {x, y, z};
    const char* names[] = {"x", "y", "z"};
    char path[64];
    for (int i = 0; i < 3; ++i) {
        std::snprintf(path, sizeof(path),
                      "sim/multiplayer/position/plane%d_%s", slot, names[i]);
        if (XPLMDataRef ref = XPLMFindDataRef(path))
            XPLMSetDatad(ref, axes[i]);
    }
    // A reused slot is still flying the previous occupant unless its
    // velocity and attitude are seeded with the position. X-Plane's local
    // axes are +x east, +y up and +z south; headings count clockwise
    // from north. Ground spawns start at rest; airborne spawns keep the
    // controller's motion rather than the stale one.
    const double gs = assign.ground ? 0.0 : assign.gs * 0.514444;
    const double rad = assign.hdg * Pi / 180.0;
    const float velocities[] = {
        static_cast<float>(std::sin(rad) * gs),
        static_cast<float>(assign.ground ? 0.0 : assign.vs / 60.0 * 0.3048),
        static_cast<float>(-std::cos(rad) * gs),
    };
    const char* vnames[] = {"v_x", "v_y", "v_z"};
    for (int i = 0; i < 3; ++i) {
        std::snprintf(path, sizeof(path),
                      "sim/multiplayer/position/plane%d_%s", slot, vnames[i]);
        if (XPLMDataRef ref = XPLMFindDataRef(path))
            XPLMSetDataf(ref, velocities[i]);
    }
    const float attitude[] = {
        static_cast<float>(assign.hdg), 0.0f, 0.0f,
    };
    const char* anames[] = {"psi", "the", "phi"};
    for (int i = 0; i < 3; ++i) {
        std::snprintf(path, sizeof(path),
                      "sim/multiplayer/position/plane%d_%s", slot, anames[i]);
        if (XPLMDataRef ref = XPLMFindDataRef(path))
            XPLMSetDataf(ref, attitude[i]);
    }
}
Json NativePositions() {
    // RREF/DREF traffic crosses X-Plane's UDP interface as float32, which
    // quantizes a degree of latitude to the better part of a metre. The
    // plugin reads the same slots in their native double precision and
    // reports them on its own channel; the control loop uses this for
    // position and falls back to RREF only when these packets stop.
    Json aircraft = Json::object();
    for (const auto& [slot, pair] : nativeSlots) {
        char path[128];
        auto refFor = [&](const char* leaf) {
            std::snprintf(path, sizeof(path),
                          "sim/multiplayer/position/plane%d_%s", slot, leaf);
            return XPLMFindDataRef(path);
        };
        XPLMDataRef lat = refFor("lat"), lon = refFor("lon"), ele = refFor("el");
        XPLMDataRef psi = refFor("psi"), phi = refFor("phi"), the = refFor("the");
        XPLMDataRef vx = refFor("v_x"), vy = refFor("v_y"), vz = refFor("v_z");
        if (!lat || !lon || !ele || !psi || !phi || !the || !vx || !vy || !vz)
            continue;
        aircraft[pair.first] = {
            XPLMGetDatad(lat), XPLMGetDatad(lon), XPLMGetDatad(ele),
            XPLMGetDataf(psi), XPLMGetDataf(phi), XPLMGetDataf(the),
            XPLMGetDataf(vx), XPLMGetDataf(vy), XPLMGetDataf(vz),
        };
    }
    return {{"v", 1}, {"t", "nativepos"}, {"ac", aircraft}};
}
void ClearNative() {
    if (nativeSlots.empty() && !nativeAcquired && !nativeBlocked) return;
    int zero = 0;
    for (const auto& [slot, pair] : nativeSlots) {
        if (ovrAuto) XPLMSetDatavi(ovrAuto, &zero, slot, 1);
        if (ovrPath) XPLMSetDatavi(ovrPath, &zero, slot, 1);
    }
    nativeSlots.clear();
    if (nativeAcquired) {
        XPLMSetActiveAircraftCount(1);
        XPLMReleasePlanes();
        nativeAcquired = false;
    }
    nativeBlocked = false;
    if (config.tcas && !xpmpOwned) TCAS();
}
void ApplyRoster(const std::vector<NativeAssign>& roster, bool exclusive) {
    if (roster.empty()) { ClearNative(); return; }
    ScanAcf();
    EnableNative();
    if (!nativeAcquired) return;
    int high = 0, one = 1, zero = 0;
    std::map<int, std::pair<std::string, std::string>> next;
    for (const auto& assign : roster) {
        high = std::max(high, assign.slot);
        const std::string acf = ModelFor(assign.type, assign.op);
        auto it = nativeSlots.find(assign.slot);
        if (it == nativeSlots.end() || it->second.first != assign.id
                || it->second.second != acf) {
            if (!acf.empty()) XPLMSetAircraftModel(assign.slot, acf.c_str());
            SeedPosition(assign.slot, assign);
            next[assign.slot] = {assign.id, acf};
            if (ovrAuto) XPLMSetDatavi(ovrAuto, &one, assign.slot, 1);
            if (ovrPath) XPLMSetDatavi(ovrPath, &zero, assign.slot, 1);
        } else {
            next[assign.slot] = it->second;
        }
    }
    for (const auto& [slot, pair] : nativeSlots)
        if (!next.count(slot)) {
            if (ovrAuto) XPLMSetDatavi(ovrAuto, &zero, slot, 1);
        }
    nativeSlots = std::move(next);
    // Every slot stays alive unless the roster is exclusive: shared,
    // the ones it did not claim were handed back above (ovrAuto 0) and
    // the simulator's own AI fills them with its native traffic rather
    // than leaving the sky empty; exclusive, a slot nobody claimed is
    // simply absent -- the engine fills them itself or wants them empty.
    XPLMSetActiveAircraftCount(1 + (exclusive ? high : std::max(high, 19)));
}

// The follow-me car is no aircraft: an instanced object placed on the
// taxiway ahead of the pilot from the engine's position stream.
XPLMObjectRef followmeObject = nullptr;
XPLMInstanceRef followmeInstance = nullptr;
XPLMProbeRef terrainProbe = nullptr;
void LoadFollowMe() {
    const auto path = (root / "Resources" / "FollowMe.obj").string();
    followmeObject = XPLMLoadObject(path.c_str());
    if (!followmeObject)
        Log("FollowMe.obj not loadable -- the follow-me car is off");
}
void FollowMeCar(double now) {
    if (!session.FollowmeActive(now)) {
        if (followmeInstance) {
            XPLMDestroyInstance(followmeInstance);
            followmeInstance = nullptr;
        }
        return;
    }
    if (!followmeObject) return;
    if (!followmeInstance) {
        followmeInstance = XPLMCreateInstance(followmeObject, nullptr);
        if (!followmeInstance) return;
    }
    if (!terrainProbe) terrainProbe = XPLMCreateProbe(xplm_ProbeY);
    const auto& f = session.followme;
    double x, y, z;
    XPLMWorldToLocal(f.lat, f.lon, 0, &x, &y, &z);
    if (terrainProbe) {
        XPLMProbeInfo_t info{};
        info.structSize = sizeof(info);
        if (XPLMProbeTerrainXYZ(terrainProbe, static_cast<float>(x),
                static_cast<float>(y), static_cast<float>(z), &info)
                == xplm_ProbeHitTerrain)
            y = info.locationY;
    }
    XPLMDrawInfo_t draw{};
    draw.structSize = sizeof(draw);
    draw.x = static_cast<float>(x); draw.y = static_cast<float>(y);
    draw.z = static_cast<float>(z); draw.heading = static_cast<float>(f.hdg);
    XPLMInstanceSetPosition(followmeInstance, &draw, nullptr);
}

std::string Trim(std::string s) {
    auto first = s.find_first_not_of(" \t\r\n");
    return first == std::string::npos ? "" : s.substr(first, s.find_last_not_of(" \t\r\n") - first + 1);
}
void ReadConfig() {
    config = Config{};
    std::ifstream file(root / "meridianatc-xp.ini");
    std::string line;
    while (std::getline(file, line)) {
        line = Trim(line);
        if (line.empty() || line[0] == '#' || line[0] == ';' || line[0] == '[') continue;
        auto split = line.find('=');
        if (split == std::string::npos) continue;
        auto key = Trim(line.substr(0, split)), value = Trim(line.substr(split + 1));
        try {
            if (key == "port") {
                size_t end = 0; int port = std::stoi(value, &end);
                if (end != value.size() || port < 1 || port > 65535) throw std::invalid_argument("port");
                config.port = port;
            } else if (key == "labels" || key == "tcas") {
                if (value != "0" && value != "1") throw std::invalid_argument("boolean");
                (key == "labels" ? config.labels : config.tcas) = value == "1";
            } else if (key == "csl" && !value.empty()) {
                fs::path path(value);
                config.csl.push_back(path.is_absolute() ? path : root / path);
            }
        } catch (const std::exception&) { Log("Ignoring invalid INI setting: " + key); }
    }
}
void LoadCSL() {
    char system[1024]{}; XPLMGetSystemPath(system);
    std::vector<fs::path> paths = {root / "Resources" / "CSL",
        fs::path(system) / "Resources/plugins/LiveTraffic/Resources/CSL",
        fs::path(system) / "Resources/plugins/xPilot/Resources/CSL"};
    paths.insert(paths.end(), config.csl.begin(), config.csl.end());
    std::set<fs::path> loaded;
    for (auto& path : paths) {
        std::error_code error;
        if (!fs::is_directory(path, error)) continue;
        auto canonical = fs::weakly_canonical(path, error);
        if (error || !loaded.insert(canonical).second) continue;
        const char* result = XPMPLoadCSLPackage(path.string().c_str());
        if (*result) Log("CSL " + path.string() + ": " + result);
    }
    Log("CSL models: " + std::to_string(XPMPGetNumberOfInstalledModels()));
}
void Labels() { XPMPEnableAircraftLabels(config.labels); }
void TCAS() {
    if (config.tcas) {
        const char* result = XPMPMultiplayerEnable();
        xpmpOwned = *result == '\0';
        if (*result) Log(std::string("TCAS control unavailable: ") + result);
    } else {
        XPMPMultiplayerDisable();
        xpmpOwned = false;
    }
}
void Sync(double now) {
    unmatched.clear();
    for (auto it = aircraft.begin(); it != aircraft.end();) {
        if (!session.snapshot.count(it->first)) it = aircraft.erase(it); else ++it;
    }
    for (const auto& [id, s] : session.snapshot) {
        try {
            auto it = aircraft.find(id);
            auto key = s.type + "\n" + s.op + "\n" + s.livery;
            if (it != aircraft.end() && it->second->modelKey != key) { aircraft.erase(it); it = aircraft.end(); }
            if (it == aircraft.end()) aircraft.emplace(id, std::make_unique<TrafficAircraft>(s, now));
            else it->second->Apply(s, now);
        } catch (const std::exception&) { aircraft.erase(id); unmatched.insert(id); }
    }
}
Json FmsPlan() {
    // The user's loaded flight plan, for corroborating the route the
    // engine thinks it is flying: what is actually in the box beats what
    // a briefing said should be. Entries arrive in leg order; the
    // displayed one is where the FMS is steering.
    Json entries = Json::array();
    const int count = XPLMCountFMSEntries();
    const int displayed = XPLMGetDisplayedFMSEntry();
    for (int i = 0; i < count && i < 512; ++i) {
        XPLMNavType type = xplm_Nav_Unknown; char id[256]{};
        int altitude = 0; float lat = 0, lon = 0;
        XPLMGetFMSEntryInfo(i, &type, id, nullptr, &altitude, &lat, &lon);
        entries.push_back({{"i", i}, {"t", static_cast<int>(type)},
            {"id", id}, {"alt", altitude}, {"lat", lat}, {"lon", lon},
            {"active", i == displayed}});
    }
    return entries;
}
Json Status(double now) {
    int xp, sdk; XPLMHostApplicationID host; XPLMGetVersions(&xp, &sdk, &host);
    char system[1024]{}; XPLMGetSystemPath(system);
    Json drawn = Json::array(); Json models = Json::object();
    for (const auto& [id, ac] : aircraft) {
        if (ac->IsVisible() && ac->IsRendered()) drawn.push_back(id);
        // What each one is actually drawn as, and how well it matched --
        // the engine's model audit reads this, not a guess.
        // The drawn pose and the systems the sim carries, so the engine
        // verifies live traffic the way SimConnect's object reads let it
        // on the other sim: the reference point's place, its height over
        // the terrain, and the gear, flaps, thrust and lights applied.
        double lat, lon, alt_ft;
        ac->GetLocation(lat, lon, alt_ft);
        models[id] = {{"m", ac->GetModelName()}, {"q", ac->GetMatchQuality()},
            {"pos", {lat, lon, alt_ft}}, {"agl", ac->agl_ft},
            {"gear", ac->applied.gear}, {"flaps", ac->applied.flaps},
            {"gnd", ac->applied.ground},
            {"thr", std::clamp(ac->applied.gs / (ac->applied.ground ? 40.0 : 250.0), 0.0, 1.0)},
            {"lights", {ac->applied.taxi, ac->applied.land, ac->applied.beacon,
                        ac->applied.strobe, ac->applied.nav}}};
    }
    // The callsigns whose planes the sim actually carries, so the engine
    // can tell a confirmed slot from a roster entry that never became a
    // plane (e.g. another plugin holding the slots).
    Json nativeIds = Json::array();
    for (const auto& [slot, pair] : nativeSlots) nativeIds.push_back(pair.first);
    Json status = {{"v", 1}, {"t", "status"}, {"plugin", "MeridianATC-XP"}, {"version", Version},
        {"xpmp2", MERIDIAN_XPMP2_VERSION}, {"xplane", xp}, {"csl_models", XPMPGetNumberOfInstalledModels()},
        {"aircraft", session.snapshot.size()}, {"drawn", drawn}, {"unmatched", unmatched},
        {"followme", session.FollowmeActive(now)}, {"sysroot", system},
        {"native_cap", nativeBlocked ? 0 : 1}, {"native", static_cast<int>(nativeSlots.size())},
        {"native_ac", nativeIds},
        {"fms", FmsPlan()}, {"models", models}};
    // One datagram holds the whole report, and the wire ends at 65507
    // bytes: a busy apron's per-aircraft block on a long FMS can outrun
    // it and the datagram dies whole, every read-back with it. Shed
    // weight the checks miss last -- the extras first, then the
    // entries of planes not drawn at all.
    if (status.dump().size() > 60000) {
        for (auto& [id, entry] : models.items())
            for (const char* key : {"pos", "agl", "gnd", "gear", "flaps",
                                    "thr", "lights"})
                entry.erase(key);
        status["models"] = models;
    }
    if (status.dump().size() > 60000) {
        Json drawnOnly = Json::object();
        for (const auto& id : drawn)
            if (models.contains(id)) drawnOnly[id] = models[id];
        status["models"] = drawnOnly;
    }
    return status;
}
void AnswerMatch(const Packet& packet, const sockaddr_in& to) {
    // What this type and paint would draw as -- XPMP2's own score, which
    // is exactly what the created model gets, without creating anything.
    int quality = XPMPModelMatchQuality(packet.type.c_str(),
        packet.op.empty() ? nullptr : packet.op.c_str(),
        packet.livery.empty() ? nullptr : packet.livery.c_str(),
        packet.cs.empty() ? nullptr : packet.cs.c_str());
    Json reply = {{"v", 1}, {"t", "match"}, {"q", packet.q},
                  {"quality", quality}};
    udp.Send(reply.dump(), to);
}
float Poll(float, float, int, void*) {
    try {
        double now = Now();
        if (session.Expire(now)) {
            aircraft.clear(); unmatched.clear(); ClearNative();
        }
        // The sim clears the AI override when it pleases; the engine's
        // control writes only hold if the flag keeps standing.
        int one = 1;
        for (const auto& [slot, pair] : nativeSlots)
            if (ovrAuto) XPLMSetDatavi(ovrAuto, &one, slot, 1);
        std::string wire; sockaddr_in from{};
        for (int n = 0; n < 64 && udp.Receive(wire, from); ++n) {
            try {
                auto packet = Parse(wire);
                if (packet.kind == "hello") {
                    if (!packet.probe) {
                        auto sender = (static_cast<uint64_t>(ntohl(from.sin_addr.s_addr)) << 16) |
                            ntohs(from.sin_port);
                        session.Claim(sender, now);
                        peer = from;
                    }
                    udp.Send(Status(now).dump(), from);
                    continue;
                }
                if (packet.kind == "metar") {
                    AnswerMetar(packet.icao, from);
                    continue;
                }
                if (packet.kind == "match") {
                    AnswerMatch(packet, from);
                    continue;
                }
                if (packet.kind == "native") {
                    ApplyRoster(packet.native, packet.exclusive);
                    peer = from;
                    continue;
                }
                auto sender = (static_cast<uint64_t>(ntohl(from.sin_addr.s_addr)) << 16) | ntohs(from.sin_port);
                if (!session.Apply(packet, now, sender)) continue;
                peer = from;
                Sync(now);
            } catch (const std::exception&) { /* Invalid datagrams leave the current snapshot intact. */ }
        }
        // Outside the receive loop: the car must also react to silence --
        // expired updates pull it off the taxiway.
        FollowMeCar(now);
        if (session.Connected(now) && now - lastStatus >= 1) {
            udp.Send(Status(now).dump(), peer); lastStatus = now;
        }
        if (session.Connected(now) && !nativeSlots.empty()
                && now - lastNativePositions >= 0.2) {
            udp.Send(NativePositions().dump(), peer);
            lastNativePositions = now;
        }
    } catch (const std::exception& e) { Log(e.what()); }
    return -1;
}
void Checks() {
    XPLMCheckMenuItem(menu, 0, visible ? xplm_Menu_Checked : xplm_Menu_Unchecked);
    XPLMCheckMenuItem(menu, 1, config.labels ? xplm_Menu_Checked : xplm_Menu_Unchecked);
    XPLMCheckMenuItem(menu, 2, config.tcas ? xplm_Menu_Checked : xplm_Menu_Unchecked);
}
void Menu(void*, void* item) {
    if (!initialized) return;
    try {
        switch (reinterpret_cast<intptr_t>(item)) {
            case 0: visible = !visible; for (auto& [id, ac] : aircraft) ac->SetVisible(visible); break;
            case 1: config.labels = !config.labels; Labels(); break;
            case 2: config.tcas = !config.tcas; TCAS(); break;
            case 3: aircraft.clear(); LoadCSL(); Sync(Now()); break;
            case 4: Log(Status(Now()).dump()); break;
        }
        Checks();
    } catch (const std::exception& e) { Log(e.what()); }
}
int Connected(void*) { return initialized && session.Connected(Now()) ? 1 : 0; }
int AircraftCount(void*) { return static_cast<int>(session.snapshot.size()); }
void Shutdown() {
    ClearNative();
    if (loop) { XPLMDestroyFlightLoop(loop); loop = nullptr; }
    for (auto layer : mapLayers) XPLMDestroyMapLayer(layer);
    mapLayers.clear();
    if (followmeInstance) { XPLMDestroyInstance(followmeInstance); followmeInstance = nullptr; }
    if (terrainProbe) { XPLMDestroyProbe(terrainProbe); terrainProbe = nullptr; }
    if (followmeObject) { XPLMUnloadObject(followmeObject); followmeObject = nullptr; }
    aircraft.clear(); unmatched.clear(); session = Session{}; udp.Close();
    if (initialized) { XPMPMultiplayerDisable(); XPMPMultiplayerCleanup(); initialized = false; }
}
}

PLUGIN_API int XPluginStart(char* name, char* signature, char* description) {
    std::snprintf(name, 256, "MeridianATC-XP");
    std::snprintf(signature, 256, "org.meridianatc.xplane");
    std::snprintf(description, 256, "MeridianATC traffic via XPMP2 and localhost UDP");
    XPLMEnableFeature("XPLM_USE_NATIVE_PATHS", 1);
    char path[1024]{}; XPLMGetPluginInfo(XPLMGetMyID(), nullptr, path, nullptr, nullptr);
    root = fs::path(path).parent_path().parent_path();
    menuIndex = XPLMAppendMenuItem(XPLMFindPluginsMenu(), "MeridianATC", nullptr, 0);
    menu = XPLMCreateMenu("MeridianATC", XPLMFindPluginsMenu(), menuIndex, Menu, nullptr);
    const char* items[] = {"Show traffic", "Labels", "TCAS control", "Reload CSL", "Log status"};
    for (intptr_t i = 0; i < 5; ++i) XPLMAppendMenuItem(menu, items[i], reinterpret_cast<void*>(i), 0);
    static std::vector<std::string> descriptions;
    for (const char* cname : {"meridianatc/transmit_com1", "meridianatc/transmit_com2",
             "meridianatc/talk", "meridianatc/talk_copilot", "meridianatc/mic_push_to_talk",
             "meridianatc/copy_error"}) {
        descriptions.push_back(std::string("MeridianATC: ") + (cname + 13));
        commands[cname] = XPLMCreateCommand(cname, descriptions.back().c_str());
        XPLMRegisterCommandHandler(commands[cname], OnCommand, 1, nullptr);
    }
    connectedRef = XPLMRegisterDataAccessor("meridianatc/xp/connected", xplmType_Int, 0, Connected, nullptr,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);
    aircraftRef = XPLMRegisterDataAccessor("meridianatc/xp/aircraft", xplmType_Int, 0, AircraftCount, nullptr,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);
    return 1;
}
PLUGIN_API int XPluginEnable() {
    if (initialized) return 1;
    try {
        ReadConfig();
        visible = true;
        const char* result = XPMPMultiplayerInit("MeridianATC-XP", (root / "Resources").string().c_str(), Preferences, "A320");
        if (*result) { Log(result); XPMPMultiplayerCleanup(); return 0; }
        initialized = true;
        ovrAuto = XPLMFindDataRef("sim/operation/override/override_plane_ai_autopilot");
        ovrPath = XPLMFindDataRef("sim/operation/override/override_planepath");
        LoadCSL(); Labels(); TCAS(); Checks(); LoadFollowMe();
        if (!udp.Open(config.port)) throw std::runtime_error("Cannot bind localhost UDP port " + std::to_string(config.port));
        XPLMRegisterMapCreationHook(MapOpened, nullptr);
        if (XPLMMapExists(XPLM_MAP_USER_INTERFACE))
            MapOpened(XPLM_MAP_USER_INTERFACE, nullptr);
        XPLMCreateFlightLoop_t params{}; params.structSize = sizeof(params);
        params.phase = xplm_FlightLoop_Phase_AfterFlightModel; params.callbackFunc = Poll;
        loop = XPLMCreateFlightLoop(&params);
        if (!loop) throw std::runtime_error("Cannot create flight loop");
        XPLMScheduleFlightLoop(loop, -1, 1);
        Log("Listening on 127.0.0.1:" + std::to_string(config.port));
        return 1;
    } catch (const std::exception& e) { Log(e.what()); Shutdown(); return 0; }
}
PLUGIN_API void XPluginDisable() { Shutdown(); }
PLUGIN_API void XPluginStop() {
    for (auto& [name, ref] : commands) if (ref) XPLMUnregisterCommandHandler(ref, OnCommand, 1, nullptr);
    commands.clear();
    Shutdown();
    if (connectedRef) XPLMUnregisterDataAccessor(connectedRef);
    if (aircraftRef) XPLMUnregisterDataAccessor(aircraftRef);
    if (menu) XPLMDestroyMenu(menu);
    if (menuIndex >= 0) XPLMRemoveMenuItem(XPLMFindPluginsMenu(), menuIndex);
}
PLUGIN_API void XPluginReceiveMessage(XPLMPluginID, int, void*) {}
