#ifndef _PACKET_H_
#define _PACKET_H_

#include <stdbool.h>
#include <inttypes.h>

enum packet_type {
    PACKET_QUAT = 0,
    PACKET_ACC = 1,
    PACKET_GYRO = 2,
    PACKET_MAG = 3,
    PACKET_COLOR = 4,
    PACKET_BLINK = 5,
    PACKET_IR = 6,
    PACKET_STREAM = 7,
    PACKET_VERSION = 8,
    PACKET_ID = 9,
    PACKET_CAL = 10,
    PACKET_MAX = 11
};

typedef struct packet_s {
    unsigned char type;
    union {
        float calibration[6];
        uint32_t net[6];
        float quat[4];
        int16_t sensor[3];
        unsigned char color[3];
        uint8_t bitmask;
        uint32_t version;
    } data;
} packet_t, *packet_p;

// Max unpacked size
#define UNPACKED_MAX_SIZE (1+6*sizeof(float))
// Maximum size of a packet_t as a SLIP frame
#define PACKET_MAX_SIZE (1+UNPACKED_MAX_SIZE*2)

// Pack into a buffer in network order with SLIP framing/escaping
// Assumes the out buffer is at least PACKET_MAX_SIZE.
// Returns the size of the packed packet
int packet_pack(packet_p packet, unsigned char *out);

// Unpack a SLIP encoded stream one char at a time, returning 1 when a packet is done
bool packet_unpack(packet_p packet, unsigned char c);

#endif /* _PACKET_H_ */

