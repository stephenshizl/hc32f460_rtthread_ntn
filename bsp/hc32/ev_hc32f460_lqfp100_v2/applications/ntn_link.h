/* SPDX-License-Identifier: Apache-2.0 */
#ifndef NTN_LINK_H
#define NTN_LINK_H
#include <string.h>
/* Matched MCU/BT firmware only. HDLC escaping keeps DATA opaque even when
 * the payload contains a complete, otherwise valid management packet. */
#define NTNL_VERSION 1U
#define NTNL_DATA 1U
#define NTNL_INFO 16U
#define NTNL_SET_NAME 17U
#define NTNL_REPLY 144U
#define NTNL_MAX_PAYLOAD 128U
#define NTNL_RAW_MAX (NTNL_MAX_PAYLOAD + 8U)
#define NTNL_WIRE_MAX (NTNL_RAW_MAX * 2U + 2U)
typedef struct {
    unsigned char raw[NTNL_RAW_MAX];
    unsigned short used;
    unsigned char started, escaped, discard;
    unsigned long good, errors;
} ntnl_decoder;
typedef void (*ntnl_receiver)(unsigned char, unsigned short,
                              const unsigned char *, unsigned short);
static __inline unsigned short ntnl_crc(const unsigned char *p, unsigned n) {
    unsigned short crc = 0xffffU;
    unsigned i;
    while (n--) {
        crc ^= (unsigned short)*p++ << 8;
        for (i=0;i<8;i++) crc=(crc&0x8000U)?(unsigned short)((crc<<1)^0x1021U):(unsigned short)(crc<<1);
    }
    return crc;
}
static __inline unsigned ntnl_encode(unsigned char kind, unsigned short id,
        const unsigned char *payload, unsigned short len, unsigned char *out) {
    unsigned char raw[NTNL_RAW_MAX];
    unsigned i, pos=0;
    unsigned short crc;
    if (len>NTNL_MAX_PAYLOAD || (len && !payload)) return 0;
    raw[0]=NTNL_VERSION;raw[1]=kind;raw[2]=(unsigned char)(id>>8);raw[3]=(unsigned char)id;
    raw[4]=(unsigned char)(len>>8);raw[5]=(unsigned char)len;
    if (len) memcpy(raw+6,payload,len);
    crc=ntnl_crc(raw,len+6U);raw[len+6U]=(unsigned char)(crc>>8);raw[len+7U]=(unsigned char)crc;
    out[pos++]=0x7e;
    for (i=0;i<len+8U;i++) {
        if (raw[i]==0x7e || raw[i]==0x7d) {out[pos++]=0x7d;out[pos++]=raw[i]^0x20;}
        else out[pos++]=raw[i];
    }
    out[pos++]=0x7e;return pos;
}
static __inline void ntnl_expire(ntnl_decoder *d) {
    if (d->used || d->escaped || d->discard) d->errors++;
    d->used=0;d->escaped=0;d->discard=0;d->started=0;
}
static __inline void ntnl_feed(ntnl_decoder *d, const unsigned char *p, unsigned n,
                       ntnl_receiver receive) {
    while (n--) {
        unsigned char ch=*p++;
        if (ch==0x7e) {
            if (d->started && (d->used || d->discard || d->escaped)) {
                unsigned short len=d->used>=6U ? (unsigned short)((d->raw[4]<<8)|d->raw[5]) : 0;
                if (!d->discard && !d->escaped && d->used>=8U &&
                    d->raw[0]==NTNL_VERSION && len<=NTNL_MAX_PAYLOAD && d->used==len+8U &&
                    ntnl_crc(d->raw,len+6U)==(unsigned short)((d->raw[len+6U]<<8)|d->raw[len+7U])) {
                    d->good++;receive(d->raw[1],(unsigned short)((d->raw[2]<<8)|d->raw[3]),d->raw+6,len);
                } else d->errors++;
            }
            d->used=0;d->escaped=0;d->discard=0;d->started=1;continue;
        }
        if (!d->started || d->discard) continue;
        if (d->escaped) {
            if (ch!=0x5e && ch!=0x5d) {d->discard=1;continue;}
            ch^=0x20;d->escaped=0;
        } else if (ch==0x7d) {d->escaped=1;continue;}
        if (d->used>=NTNL_RAW_MAX) {d->discard=1;continue;}
        d->raw[d->used++]=ch;
    }
}
#endif
