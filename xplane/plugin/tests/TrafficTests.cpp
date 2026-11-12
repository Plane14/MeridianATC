#include "Udp.h"
#include "Traffic.h"
#include <iostream>
#include <thread>

using namespace Meridian;
void Check(bool condition, const char* what) {
    if (!condition) throw std::runtime_error(what);
}
template<typename F> void Reject(F fn) {
    bool rejected = false;
    try { fn(); } catch (const std::exception&) { rejected = true; }
    Check(rejected, "expected rejection");
}
Json Aircraft() {
    return {{"id", "DLH4AB"}, {"type", "A320"}, {"op", "DLH"}, {"livery", ""}, {"reg", "D-AIAB"},
        {"lat", 48.0}, {"lon", 11.0}, {"alt_ft", 12000}, {"hdg", 359}, {"pitch", 2}, {"roll", 0},
        {"gs_kt", 250}, {"vs_fpm", 600}, {"gnd", false}, {"gear", 0}, {"flaps", 0},
        {"lights", {{"taxi", 0}, {"land", 0}, {"bcn", 1}, {"strb", 1}, {"nav", 1}}}, {"squawk", "2000"}};
}
void Protocol() {
    auto a = Aircraft();
    Json j = {{"v", 1}, {"t", "traffic"}, {"seq", 1}, {"ac", Json::array({a})}};
    auto packet = Parse(j.dump());
    Check(packet.aircraft.at("DLH4AB").beacon, "lights");
    Check(packet.aircraft.at("DLH4AB").alt == 12000, "altitude");
    Check(Parse(R"({"v":1,"t":"hello"})").kind == "hello", "hello");
    Check(Parse(R"({"v":1,"t":"clear"})").kind == "clear", "clear");
    Session session;
    Check(session.Apply(packet, 10) && session.snapshot.size() == 1, "snapshot");
    Check(!session.Apply(packet, 11), "duplicate sequence");
    auto empty = packet; empty.aircraft.clear(); empty.seq = 2;
    Check(session.Apply(empty, 12) && session.snapshot.empty(), "absent aircraft removed");
    packet.seq = 3; session.Apply(packet, 13);
    Check(!session.Expire(22.99) && session.Expire(23) && session.snapshot.empty(), "timeout");
    packet.seq = 0; Check(session.Apply(packet, 24), "restart after timeout");
    session.Apply(Parse(R"({"v":1,"t":"clear"})"), 25);
    Check(session.snapshot.empty(), "clear session");
    j["ac"] = Json::array(); Check(Parse(j.dump()).aircraft.empty(), "empty snapshot");
    Reject([] { Parse(""); }); Reject([] { Parse("not json"); });
    Reject([] { Parse(R"({"v":2,"t":"hello"})"); });
    Reject([] { Parse(R"({"v":1,"t":"unknown"})"); });
    Reject([] { Parse(std::string(65508, ' ')); });
    Reject([] { Parse(std::string(30, '[') + "0" + std::string(30, ']')); });
    j["ac"] = Json::array({a, a}); Reject([&] { Parse(j.dump()); });
    j["ac"] = Json::array({a}); j["seq"] = -1; Reject([&] { Parse(j.dump()); });
    j["seq"] = 2; j["ac"][0]["lat"] = 91; Reject([&] { Parse(j.dump()); });
    j["ac"][0] = a; j["ac"][0]["squawk"] = "8888"; Reject([&] { Parse(j.dump()); });
    j["ac"][0] = a; j["ac"][0]["gnd"] = 1; Reject([&] { Parse(j.dump()); });
    j["ac"][0] = a; j["ac"][0]["lights"]["taxi"] = .5; Reject([&] { Parse(j.dump()); });
    j["ac"][0] = a; j["ac"][0].erase("hdg"); Reject([&] { Parse(j.dump()); });
    j["ac"] = Json::array(); for (int i = 0; i < 513; ++i) j["ac"].push_back(a);
    Reject([&] { Parse(j.dump()); });
}
void Ownership() {
    Session session;
    auto hello = Parse(R"({"v":1,"t":"hello"})");
    auto clear = Parse(R"({"v":1,"t":"clear"})");
    auto traffic = Parse(Json{{"v", 1}, {"t", "traffic"}, {"seq", 1}, {"ac", Json::array({Aircraft()})}}.dump());
    Check(session.Apply(hello, 0, 1) && !session.Connected(0), "doctor hello never owns idle slot");
    Check(session.Apply(traffic, 1, 2), "traffic claims slot after doctor");
    Check(session.Apply(hello, 9, 3) && session.snapshot.size() == 1, "foreign hello does not clear traffic");
    Check(!session.Apply(clear, 9, 3), "foreign clear rejected");
    Check(!session.Apply(traffic, 9, 3), "foreign traffic rejected");
    Check(session.Apply(hello, 10, 2), "owner hello answered");
    Check(session.Apply(traffic, 11, 3), "hello does not postpone takeover or sequence reset");
    Check(session.Apply(clear, 12, 3) && session.snapshot.empty(), "owner clear accepted");
    Check(!session.Apply(traffic, 13, 2), "clear retains sender ownership");
    Check(session.Apply(clear, 22, 2), "clear can take over after inactivity");
    Check(!session.Connected(32), "ownership expires after clear");
}
void Movement() {
    auto a = Parse(Json{{"v", 1}, {"t", "traffic"}, {"seq", 1}, {"ac", Json::array({Aircraft()})}}.dump()).aircraft.begin()->second;
    a.hdg = 359; Motion motion(a, 10);
    auto b = a; b.hdg = 1; b.lat += .01;
    motion.Update(b, 11);
    auto middle = motion.Sample(10.7);
    Check(std::abs(Delta(0, middle.hdg)) < .001, "heading wraps through north");
    Check(std::abs(middle.lat - (a.lat + .005)) < .00001, "position interpolation");
    auto projected = motion.Sample(12.2);
    Check(projected.lat > b.lat && std::abs(projected.alt - b.alt - 10) < .0001, "gs/vs extrapolation");
    Check(std::abs(motion.Sample(100).lat - motion.Sample(13.2).lat) < .00001, "extrapolation capped");
    b.ground = true; Check(Project(b, 2).alt == b.alt, "ground ignores VS");
    a.lon = 179.9; b.lon = -179.9; Motion dateLine(a, 0); dateLine.Update(b, 1);
    Check(std::abs(std::abs(dateLine.Sample(.7).lon) - 180) < .001, "date line wrap");
    a.lat = 0; Motion frequent(a, 0);
    for (int i = 1; i <= 10; ++i) { a.lat = i * .001; frequent.Update(a, i * .1); }
    Check(std::abs(frequent.Sample(1.05).lat - .0085) < .000001, "10 Hz buffered interpolation");
    a.lat = .011; frequent.Update(a, 1.1);
    Check(std::abs(frequent.Sample(1.1).lat - .009) < .000001, "new snapshot does not snap to previous tick");
    frequent.Update(a, 1.1);
    Check(std::isfinite(frequent.Sample(1.2).lat), "duplicate arrival timestamp");
}
void Socket() {
    Udp server;
    int port = 49110;
    while (port < 49210 && !server.Open(port)) ++port;
    Check(port < 49210, "bind loopback");
    Udp duplicate; Check(!duplicate.Open(port), "duplicate port fails safely");
    std::string wire; sockaddr_in from{};
    Check(!server.Receive(wire, from), "nonblocking empty socket");
    auto client = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    sockaddr_in to{}; to.sin_family = AF_INET; to.sin_port = htons(static_cast<unsigned short>(port));
    to.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    std::string hello = R"({"v":1,"t":"hello"})";
    sendto(client, hello.data(), static_cast<int>(hello.size()), 0, reinterpret_cast<sockaddr*>(&to), sizeof(to));
    bool received = false;
    for (int i = 0; i < 100 && !(received = server.Receive(wire, from)); ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    Check(received && Parse(wire).kind == "hello", "UDP receive");
    server.Send("status", from);
#ifdef _WIN32
    DWORD timeout = 1000;
    setsockopt(client, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char*>(&timeout), sizeof(timeout));
#else
    timeval timeout{1, 0}; setsockopt(client, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
#endif
    char reply[32]{};
    Check(recv(client, reply, sizeof(reply), 0) == 6 && std::string(reply, 6) == "status", "source-address reply");
#ifdef _WIN32
    closesocket(client);
#else
    close(client);
#endif
    server.Close(); Check(server.Open(port), "clean socket reopen");
}
int main() {
    try { Protocol(); Ownership(); Movement(); Socket(); std::cout << "protocol, ownership, snapshot lifecycle, motion and UDP tests passed\n"; }
    catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
