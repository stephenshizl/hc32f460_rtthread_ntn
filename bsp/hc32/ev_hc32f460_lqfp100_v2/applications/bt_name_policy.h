/* SPDX-License-Identifier: Apache-2.0 */
#ifndef BT_NAME_POLICY_H
#define BT_NAME_POLICY_H
#include <string.h>
static __inline int btn_valid(const char *s, unsigned max) {
    unsigned n=0;
    if (!s || !*s) return 0;
    for (;*s;s++) {
        if (++n>max || !((*s>='a'&&*s<='z')||(*s>='A'&&*s<='Z')||
            (*s>='0'&&*s<='9')||*s=='_'||*s=='-')) return 0;
    }
    return 1;
}
static __inline int btn_imei(const char *s) {
    unsigned i;int nonzero=0,nonnine=0;
    if (!s || strlen(s)!=15U) return 0;
    for (i=0;i<15;i++) {
        if (s[i]<'0'||s[i]>'9') return 0;
        nonzero|=s[i]!='0';nonnine|=s[i]!='9';
    }
    return nonzero&&nonnine;
}
static __inline void btn_auto(char out[32],const char *identity,const char mac[13]) {
    unsigned n=identity ? (unsigned)strlen(identity) : 0;
    memcpy(out,"NTN-Terminal-",13U);
    if (n && n<=18U && btn_valid(identity,18U)) strcpy(out+13,identity);
    else if (n && btn_valid(identity,32U)) {
        memcpy(out+13,identity,5U);out[18]='-';strcpy(out+19,mac);
    } else strcpy(out+13,mac);
}
/* Only complete lines from a successful CGSN/ATI transaction are candidates.
 * No substring match against echo/URC and no accepting a truncated identifier. */
static __inline int btn_extract(const char *response,int want_imei,char *out,unsigned cap) {
    const char *p=response;
    while (*p) {
        const char *end=p;const char *v;unsigned n;
        while (*end && *end!='\r' && *end!='\n') end++;
        if (!*end) break;
        v=p;
        if ((unsigned)(end-v)>=6U && !strncmp(v,"+CGSN:",6)) v+=6;
        else if (want_imei && (unsigned)(end-v)>=5U && !strncmp(v,"IMEI:",5)) v+=5;
        else if (*v=='+' || (end-v>=2 && v[0]=='A' && v[1]=='T') ||
                 (end-v==2 && !strncmp(v,"OK",2)) ||
                 (end-v==5 && !strncmp(v,"ERROR",5))) {p=end+1;continue;}
        while (v<end && *v==' ') v++;
        while (end>v && end[-1]==' ') end--;
        if (end-v>=2 && v[0]=='"' && end[-1]=='"') {v++;end--;}
        n=(unsigned)(end-v);
        if (n && n<cap) {
            char candidate[33];
            if (n<sizeof(candidate)) {
                memcpy(candidate,v,n);candidate[n]=0;
                if ((want_imei && btn_imei(candidate)) ||
                    (!want_imei && btn_valid(candidate,32U) && strcmp(candidate,"0") && strcmp(candidate,"UNKNOWN"))) {
                    memcpy(out,candidate,n+1U);return 1;
                }
            }
        }
        /* Use original line boundary, not the space/quote-trimmed end. */
        while (*p && *p!='\r' && *p!='\n') p++;
        while (*p=='\r'||*p=='\n') p++;
    }
    return 0;
}
#endif
