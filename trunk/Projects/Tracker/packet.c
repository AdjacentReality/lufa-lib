#include "packet.h"
#include <LUFA/Common/Common.h>

// A NONSTANDARD FOR TRANSMISSION OF IP DATAGRAMS OVER SERIAL LINES: SLIP
// http://tools.ietf.org/rfc/rfc1055.txt
#define END             0xC0    /* indicates end of packet */
#define ESC             0xDB    /* indicates byte stuffing */
#define ESC_END         0xDC    /* ESC ESC_END means END data byte */
#define ESC_ESC         0xDD    /* ESC ESC_ESC means ESC data byte */

const int const g_packet_size[PACKET_MAX] = {1+4*sizeof(float), 1+3*sizeof(int16_t), 1+3*sizeof(int16_t),
    1+3*sizeof(int16_t), 1+3, 1+3, 1+1, 1+sizeof(uint32_t), 1+sizeof(uint32_t)};

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
    uint32_t tmp;
    *out = packet->type;
    
    switch(packet->type) {
        case PACKET_QUAT:
            for (int i = 0; i < 4; i++) {
                tmp = cpu_to_be32(*(uint32_t *)&packet->data.quat[i]);
                out_len += pack_seq((unsigned char *)&tmp, sizeof(uint32_t), out+out_len);
            }
            break;
            
        case PACKET_ACC:
        case PACKET_GYRO:
        case PACKET_MAG:
            for (int i = 0; i < 3; i++) {
                tmp = cpu_to_be16(*(int16_t *)&packet->data.sensor[i]);
                out_len += pack_seq((unsigned char *)&tmp, sizeof(int16_t), out+out_len);
            }
            break;
            
        case PACKET_COLOR:
        case PACKET_BLINK:
            out_len += pack_seq(packet->data.color, 3, out+out_len);
            break;
        case PACKET_IR:
            out_len += pack_seq(packet->data.color, 1, out+out_len);
            break;
        case PACKET_VERSION:
        case PACKET_ID:
            tmp = cpu_to_be32(packet->data.version);
            out_len += pack_seq((unsigned char *)&tmp, sizeof(uint32_t), out+out_len);
            break;
    }
    
    *(out+out_len) = END;
    out_len++;
    
    return out_len;
}

static bool packet_parse(packet_p packet, unsigned char *buf, int len)
{
    packet->type = buf[0];
    
    // For now, we only care about reading color setting messages
    if (((buf[0] == PACKET_COLOR) || (buf[0] == PACKET_BLINK)) &&
        (len == g_packet_size[PACKET_COLOR])) {
        packet->data.color[0] = buf[1];
        packet->data.color[1] = buf[2];
        packet->data.color[2] = buf[3];
        return 1;
    }
    
    return 0;
}

static unsigned char unpacked_buf[UNPACKED_MAX_SIZE+1];
static unsigned int unpacked_len = 0;
static bool escaping = 0;

// Unpack a SLIP encoded stream one char at a time, returning 1 when a packet is done
bool packet_unpack(packet_p packet, unsigned char c)
{
    bool ret = 0;

    switch (c) {
        case END:
            if (packet_parse(packet, unpacked_buf, unpacked_len))
                ret = 1;
            unpacked_len = 0;
            break;
        case ESC:
            escaping = 1;
            break;
        case ESC_END:
            if (escaping) *(unpacked_buf+unpacked_len) = END;
            else *(unpacked_buf+unpacked_len) = ESC_END;
            unpacked_len++;
            break;
        case ESC_ESC:
            if (escaping) *(unpacked_buf+unpacked_len) = ESC;
            else *(unpacked_buf+unpacked_len) = ESC_ESC;
            unpacked_len++;
            break;
        default:
            *(unpacked_buf+unpacked_len) = c;
            unpacked_len++;
            break;
    }
    
    if (c != ESC) escaping = 0;
    // reset the buffer if it no packet was formed
    if (unpacked_len > UNPACKED_MAX_SIZE) unpacked_len = 0;

    return ret;
}

