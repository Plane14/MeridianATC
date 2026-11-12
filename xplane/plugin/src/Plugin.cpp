#include "Udp.h"
#include "Traffic.h"
#include "XPMPAircraft.h"
#include "XPLMDataAccess.h"
#include "XPLMGraphics.h"
#include "XPLMMenus.h"
#include "XPLMPlugin.h"
#include "XPLMProcessing.h"
#include "XPLMScenery.h"
#include "XPLMUtilities.h"
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>
#include <set>
#include <vector>

namespace {
using namespace Meridian;
namespace fs = std::filesystem;
constexpr const char* Version = "0.1.0";
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
bool initialized = false, visible = true, peerKnown = false;
double lastSeen = -1e9, lastStatus = -1e9;
Udp udp;
sockaddr_in peer{};
Session session;
XPLMFlightLoopID loop = nullptr;
XPLMMenuID menu = nullptr;
int menuIndex = -1;
XPLMDataRef connectedRef = nullptr, aircraftRef = nullptr;

class TrafficAircraft : public XPMP2::Aircraft {
    Motion motion;
    XPLMProbeRef probe = nullptr;
public:
    std::string modelKey;
    TrafficAircraft(const AircraftState& s, double now) :
        Aircraft(s.type, s.op, s.livery, 0, "", s.id), motion(s, now),
        modelKey(s.type + "\n" + s.op + "\n" + s.livery) { Apply(s, now, false); }
    ~TrafficAircraft() override { if (probe) XPLMDestroyProbe(probe); }
    void Apply(const AircraftState& s, double now, bool update = true) {
        if (update) motion.Update(s, now);
        label = s.id;
        std::snprintf(acInfoTexts.tailNum, sizeof(acInfoTexts.tailNum), "%s", s.reg.c_str());
        std::snprintf(acInfoTexts.icaoAcType, sizeof(acInfoTexts.icaoAcType), "%s", s.type.c_str());
        std::snprintf(acInfoTexts.icaoAirline, sizeof(acInfoTexts.icaoAirline), "%s", s.op.c_str());
        std::snprintf(acInfoTexts.airline, sizeof(acInfoTexts.airline), "%s", s.op.c_str());
        std::snprintf(acInfoTexts.flightNum, sizeof(acInfoTexts.flightNum), "%s", s.id.c_str());
        acRadar.code = std::stol(s.squawk);
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
                s.alt = altitude / .3048;
            }
        }
        SetLocation(s.lat, s.lon, s.alt);
        SetHeading(static_cast<float>(s.hdg)); SetPitch(static_cast<float>(s.pitch)); SetRoll(static_cast<float>(s.roll));
    }
};
std::map<std::string, std::unique_ptr<TrafficAircraft>> aircraft;
std::set<std::string> unmatched;

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
        if (*result) Log(std::string("TCAS control unavailable: ") + result);
    } else XPMPMultiplayerDisable();
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
Json Status() {
    int xp, sdk; XPLMHostApplicationID host; XPLMGetVersions(&xp, &sdk, &host);
    Json drawn = Json::array();
    for (const auto& [id, ac] : aircraft) if (ac->IsVisible() && ac->IsRendered()) drawn.push_back(id);
    return {{"v", 1}, {"t", "status"}, {"plugin", "MeridianATC-XP"}, {"version", Version},
        {"xpmp2", MERIDIAN_XPMP2_VERSION}, {"xplane", xp}, {"csl_models", XPMPGetNumberOfInstalledModels()},
        {"aircraft", session.snapshot.size()}, {"drawn", drawn}, {"unmatched", unmatched}};
}
float Poll(float, float, int, void*) {
    try {
        double now = Now();
        if (session.Expire(now)) { aircraft.clear(); unmatched.clear(); }
        std::string wire; sockaddr_in from{};
        for (int n = 0; n < 64 && udp.Receive(wire, from); ++n) {
            try {
                auto packet = Parse(wire);
                bool newPeer = !peerKnown || now - lastSeen >= 10;
                if (!newPeer && (from.sin_port != peer.sin_port || from.sin_addr.s_addr != peer.sin_addr.s_addr)) {
                    if (packet.kind == "hello") udp.Send(Status().dump(), from);
                    continue;
                }
                if (newPeer) session.Clear();
                if (!session.Apply(packet, now)) continue;
                peer = from; peerKnown = true; lastSeen = now;
                if (packet.kind != "hello") Sync(now);
                if (packet.kind == "hello") { udp.Send(Status().dump(), peer); lastStatus = now; }
            } catch (const std::exception&) { /* Invalid datagrams leave the current snapshot intact. */ }
        }
        if (peerKnown && now - lastSeen < 10 && now - lastStatus >= 1) {
            udp.Send(Status().dump(), peer); lastStatus = now;
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
            case 4: Log(Status().dump()); break;
        }
        Checks();
    } catch (const std::exception& e) { Log(e.what()); }
}
int Connected(void*) { return initialized && peerKnown && Now() - lastSeen < 10 ? 1 : 0; }
int AircraftCount(void*) { return static_cast<int>(session.snapshot.size()); }
void Shutdown() {
    if (loop) { XPLMDestroyFlightLoop(loop); loop = nullptr; }
    aircraft.clear(); unmatched.clear(); session.Clear(); udp.Close(); peerKnown = false;
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
        LoadCSL(); Labels(); TCAS(); Checks();
        if (!udp.Open(config.port)) throw std::runtime_error("Cannot bind localhost UDP port " + std::to_string(config.port));
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
    Shutdown();
    if (connectedRef) XPLMUnregisterDataAccessor(connectedRef);
    if (aircraftRef) XPLMUnregisterDataAccessor(aircraftRef);
    if (menu) XPLMDestroyMenu(menu);
    if (menuIndex >= 0) XPLMRemoveMenuItem(XPLMFindPluginsMenu(), menuIndex);
}
PLUGIN_API void XPluginReceiveMessage(XPLMPluginID, int, void*) {}
