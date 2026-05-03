#include "udp_stream.h"
#include <zephyr/kernel.h>
#include <zephyr/net/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <zephyr/logging/log.h>


LOG_MODULE_REGISTER(udp_stream, LOG_LEVEL_INF);

static int udp_sock = -1;
static struct sockaddr_in destination_addr;

int udp_stream_init(const char *dest_ip, uint16_t port)
{
    if (udp_sock >= 0) {
        LOG_WRN("UDP stream already initialized.");
        return 0;
    }

    udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (udp_sock < 0) {
        LOG_ERR("Failed to create UDP socket: %d", udp_sock);
        return udp_sock;
    }

    destination_addr.sin_family = AF_INET;
    destination_addr.sin_port = htons(port);
    
    if (inet_pton(AF_INET, dest_ip, &destination_addr.sin_addr) <= 0) {
        LOG_ERR("Invalid destination IP address: %s", dest_ip);
        close(udp_sock);
        udp_sock = -1;
        return -EINVAL;
    }

    LOG_INF("UDP stream initialized targeting %s:%u", dest_ip, port);
    return 0;
}

int udp_stream_send(const void *data, size_t length)
{
    if (udp_sock < 0 || data == NULL || length == 0) {
        return -EINVAL;
    }

    int ret = sendto(udp_sock, data, length, 0, 
                     (struct sockaddr *)&destination_addr, 
                     sizeof(destination_addr));

    if (ret < 0) {
        LOG_ERR("UDP send failed: %d", errno);
        return -errno;
    }

    return ret;
}