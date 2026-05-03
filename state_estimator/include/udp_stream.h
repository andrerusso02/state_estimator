#ifndef UDP_STREAM_H
#define UDP_STREAM_H

#include <stddef.h>
#include <stdint.h>

/**
 * @brief Initializes the UDP socket and destination address.
 * 
 * @param dest_ip String representation of the destination IP (e.g., "255.255.255.255")
 * @param port Destination port
 * @return 0 on success, negative error code on failure.
 */
int udp_stream_init(const char *dest_ip, uint16_t port);

/**
 * @brief Sends an arbitrary payload over the initialized UDP socket.
 * 
 * @param data Pointer to the payload buffer.
 * @param length Size of the payload in bytes.
 * @return Bytes sent on success, negative error code on failure.
 */
int udp_stream_send(const void *data, size_t length);

#endif /* UDP_STREAM_H */