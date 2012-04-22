#ifndef _PACKET_H_
#define _PACKET_H_

#include <stdbool.h>

enum packet_type {
    PACKET_QUAT = 0,
    PACKET_ACC = 1,
    PACKET_GYRO = 2,
    PACKET_MAG = 3,
    PACKET_COLOR = 4,
    PACKET_BLINK = 5,
    PACKET_MAX = 6
};

typedef struct packet_s {
    unsigned char type;
    union {
        float quat[4];
        float sensor[3];
        unsigned char color[4];
    } data;
} packet_t, *packet_p;

// Max unpacked size
#define UNPACKED_MAX_SIZE (1+4*sizeof(float))
// Maximum size of a packet_t as a SLIP frame
#define PACKET_MAX_SIZE (2+4*sizeof(float)*2)

// Pack into a buffer in network order with SLIP framing/escaping
// Assumes the out buffer is at least PACKET_MAX_SIZE.
// Returns the size of the packed packet
int packet_pack(packet_p packet, unsigned char *out);

// Unpack a SLIP encoded stream one char at a time, returning 1 when a packet is done
bool packet_unpack(packet_p packet, unsigned char c);

#endif /* _PACKET_H_ */

