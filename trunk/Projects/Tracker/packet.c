#include "packet.h"
#include <inttypes.h>
#include <LUFA/Common/Common.h>

// A NONSTANDARD FOR TRANSMISSION OF IP DATAGRAMS OVER SERIAL LINES: SLIP
// http://tools.ietf.org/rfc/rfc1055.txt
#define END             0xC0    /* indicates end of packet */
#define ESC             0xDB    /* indicates byte stuffing */
#define ESC_END         0xDC    /* ESC ESC_END means END data byte */
#define ESC_ESC         0xDD    /* ESC ESC_ESC means ESC data byte */

int pack_seq(unsigned char *buf, int len, unsigned char *out)
{
    unsigned char *initial = out;

    while (len--) {
        switch (*buf) {
            case END:
                *out++ = ESC;
                *out++ = ESC_END;
                break;
            case ESC:
                *out++ = ESC;
                *out++ = ESC_ESC;
                break;
            default:
                *out++ = *buf;
                break;
        }
        buf++;
    }
    
    return out-initial;
}

// Pack into a buffer in network order with SLIP framing/escaping
int packet_pack(packet_p packet, unsigned char *out)
{
    int out_len = 1;
    *out = packet->type;
    
    switch(packet->type) {
        case PACKET_QUAT:
            for (int i = 0; i < 4; i++) {
                uint32_t tmp = cpu_to_be32(*(uint32_t *)&packet->data.quat[i]);
                out_len += pack_seq((unsigned char *)&tmp, sizeof(uint32_t), out+out_len);
            }
            break;
            
        case PACKET_ACC:
        case PACKET_GYRO:
        case PACKET_MAG:
            for (int i = 0; i < 3; i++) {
                uint32_t tmp = cpu_to_be32(*(uint32_t *)&packet->data.sensor[i]);
                out_len += pack_seq((unsigned char *)&tmp, sizeof(uint32_t), out+out_len);
            }
            break;
            
        case PACKET_COLOR:
        case PACKET_BLINK:
            out_len += pack_seq(packet->data.color, 4, out+out_len);
            break;
    }
    
    *(out+out_len) = END;
    out_len++;
    
    return out_len;
}

// Unpack from a SLIP packed buffer
int packet_unpack(packet_p packet, unsigned char *in, int in_size)
{
    // TODO: well, you know
    return 0;
}

