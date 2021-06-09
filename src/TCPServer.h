#include <lwip/sockets.h>

class TCPServer {
    public:

    typedef void (*service_t)(const TCPServer *p, const int fd);

    TCPServer(const int port, const service_t service);

    int port = 0;

    int listener_fd = 0;
    int accepted_fd = 0;
    
    sockaddr_in dest;
    sockaddr_in src;

    service_t service = nullptr;
    static void echo_service(const TCPServer *p, const int sock);
};
