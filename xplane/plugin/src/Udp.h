#pragma once

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>
#endif
#include <string>

namespace Meridian {
class Udp {
#ifdef _WIN32
    SOCKET fd = INVALID_SOCKET;
    bool started = false;
#else
    int fd = -1;
#endif
public:
    ~Udp() { Close(); }
    bool Open(int port) {
        Close();
#ifdef _WIN32
        WSADATA data;
        if (WSAStartup(MAKEWORD(2, 2), &data)) return false;
        started = true;
#endif
        fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
#ifdef _WIN32
        if (fd == INVALID_SOCKET) { Close(); return false; }
        u_long nonblocking = 1;
        if (ioctlsocket(fd, FIONBIO, &nonblocking)) { Close(); return false; }
        BOOL exclusive = TRUE;
        setsockopt(fd, SOL_SOCKET, SO_EXCLUSIVEADDRUSE, reinterpret_cast<const char*>(&exclusive), sizeof(exclusive));
#else
        if (fd < 0) return false;
        if (fcntl(fd, F_SETFL, O_NONBLOCK) < 0) { Close(); return false; }
#endif
        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        address.sin_port = htons(static_cast<unsigned short>(port));
        if (bind(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address))) { Close(); return false; }
        return true;
    }
    bool Receive(std::string& text, sockaddr_in& from) {
        char buffer[65536];
#ifdef _WIN32
        int size = sizeof(from);
#else
        socklen_t size = sizeof(from);
#endif
        auto n = recvfrom(fd, buffer, sizeof(buffer), 0, reinterpret_cast<sockaddr*>(&from), &size);
        if (n <= 0 || from.sin_addr.s_addr != htonl(INADDR_LOOPBACK)) return false;
        text.assign(buffer, static_cast<size_t>(n));
        return true;
    }
    void Send(const std::string& text, const sockaddr_in& to) {
        sendto(fd, text.data(), static_cast<int>(text.size()), 0,
               reinterpret_cast<const sockaddr*>(&to), sizeof(to));
    }
    void Close() {
#ifdef _WIN32
        if (fd != INVALID_SOCKET) closesocket(fd);
        fd = INVALID_SOCKET;
        if (started) WSACleanup();
        started = false;
#else
        if (fd >= 0) close(fd);
        fd = -1;
#endif
    }
};
}
