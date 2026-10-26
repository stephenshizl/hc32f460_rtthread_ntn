"""Compile the production shared wire codec and name policy on the host."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

BSP = Path(__file__).resolve().parents[1]
C = r'''
#include <assert.h>
#include <stdio.h>
#include "ntn_link.h"
#include "bt_name_policy.h"
static unsigned delivered, management;
static unsigned char got[NTNL_MAX_PAYLOAD];
static unsigned short size, last_id;
static void receive(unsigned char kind,unsigned short id,const unsigned char *p,unsigned short n) {
 delivered++;management+=kind==NTNL_SET_NAME;size=n;last_id=id;memcpy(got,p,n);
}
static void roundtrip(const unsigned char *data,unsigned len,unsigned split) {
 unsigned char wire[NTNL_WIRE_MAX];ntnl_decoder d={{0},0,0,0,0,0,0};unsigned pos,n;
 n=ntnl_encode(NTNL_DATA,0x7e7d,data,len,wire);assert(n<=sizeof(wire));
 for(pos=0;pos<n;pos+=split) ntnl_feed(&d,wire+pos,n-pos<split?n-pos:split,receive);
 assert(d.good==1 && !d.errors && size==len && last_id==0x7e7d && !memcmp(got,data,len));
}
int main(void) {
 unsigned char data[NTNL_MAX_PAYLOAD],wire[NTNL_WIRE_MAX],nested[NTNL_WIRE_MAX];
 unsigned i,n,m,before;ntnl_decoder d={{0},0,0,0,0,0,0};char name[32],id[33];
 for(i=0;i<sizeof(data);i++) data[i]=(unsigned char)i;
 for(i=1;i<=sizeof(data);i++) roundtrip(data,i,1);
 puts("PASS codec bytewise all lengths, binary NUL/CRLF/escape");
 for(i=1;i<40;i++) roundtrip(data,sizeof(data),i);
 puts("PASS codec variable fragmentation");
 n=ntnl_encode(NTNL_DATA,1,data,3,wire);before=delivered;
 ntnl_feed(&d,wire,n,receive);ntnl_feed(&d,wire,n,receive);
 assert(delivered==before+2);puts("PASS coalesced/adjacent frames");
 m=ntnl_encode(NTNL_SET_NAME,4,(unsigned char *)"evil",4,nested);
 before=management;roundtrip(nested,m,1);assert(management==before);
 puts("PASS embedded complete management packet stays DATA");
 n=ntnl_encode(NTNL_SET_NAME,2,data,20,wire);wire[10]^=1;before=management;
 ntnl_feed(&d,wire,n,receive);assert(management==before && d.errors);
 n=ntnl_encode(NTNL_DATA,3,data,8,wire);ntnl_feed(&d,wire,n,receive);assert(last_id==3);
 puts("PASS bad CRC never executes, next frame recovers");
 ntnl_feed(&d,(unsigned char *)"\x7e",1,receive);
 for(i=0;i<1000;i++) ntnl_feed(&d,(unsigned char *)"X",1,receive);
 ntnl_feed(&d,wire,n,receive);assert(last_id==3);puts("PASS oversize discarded until delimiter");
 ntnl_feed(&d,wire,n/2,receive);ntnl_expire(&d);before=delivered;
 ntnl_feed(&d,wire+n/2,n-n/2,receive);assert(delivered==before);
 ntnl_feed(&d,wire,n,receive);assert(delivered==before+1);puts("PASS partial timeout resynchronization");
 ntnl_feed(&d,(unsigned char *)"\x7e\x7d\x11\x7e",4,receive);
 ntnl_feed(&d,wire,n,receive);assert(last_id==3);puts("PASS invalid escape resynchronization");
 assert(!btn_valid("",31)&&!btn_valid("a,b",31)&&!btn_valid("中文",31)&&!btn_valid("a\r",31));
 memset(id,'A',31);id[31]=0;assert(btn_valid(id,31));id[31]='B';id[32]=0;assert(!btn_valid(id,31));
 assert(btn_valid("AUTO",31));puts("PASS names 0/31/32 bytes and literal AUTO");
 btn_auto(name,"yixing_249_005","98D972FDD7C0");assert(!strcmp(name,"NTN-Terminal-yixing_249_005"));
 btn_auto(name,"862130042600001","98D972FDD7C0");assert(strlen(name)==28);
 puts("PASS real modem SN and full IMEI naming");
 btn_auto(name,"12345678901234567890123456789012","98D972FDD7C0");assert(strlen(name)==31&&!strcmp(name+19,"98D972FDD7C0"));
 btn_auto(name,"bad,identity","98D972FDD7C0");assert(!strcmp(name,"NTN-Terminal-98D972FDD7C0"));
 puts("PASS long/invalid SN uses full MAC discriminator");
 assert(btn_extract("AT+CGSN=0\r+CGSN:yixing_249_005\r\nOK\r\n",0,id,sizeof(id))&&!strcmp(id,"yixing_249_005"));
 assert(btn_extract("AT+CGSN=1\r+CGSN:862130042600001\r\nOK\r\n",1,id,sizeof(id)));
 assert(btn_extract("+CGSN: \"862130042600001\"\r\nOK\r\n",1,id,sizeof(id)));
 puts("PASS real complete-line identity responses and quoting");
 assert(!btn_extract("AT+CGSN=1\r\nERROR\r\n",1,id,sizeof(id)));
 assert(!btn_extract("+CGSN:000000000000000\r\nOK\r\n",1,id,sizeof(id)));
 assert(!btn_extract("+URC:862130042600001\r\n",1,id,sizeof(id)));
 assert(!btn_extract("+CGSN:862130042600001",1,id,sizeof(id)));
 puts("PASS echo/ERROR/URC/placeholder/truncation rejection");
 puts("All 13 production codec/policy groups passed");
 return 0;
}
'''

def main():
    compiler = os.environ.get("HOST_CC") or shutil.which("gcc")
    if not compiler:
        raise SystemExit("Set HOST_CC to host GCC")
    with tempfile.TemporaryDirectory(prefix="bt_naming_") as temp:
        source = Path(temp) / "test.c"
        binary = Path(temp) / ("test.exe" if os.name == "nt" else "test")
        source.write_text(C, encoding="utf-8")
        subprocess.run([compiler,"-std=c99","-Wall","-Wextra","-Werror",
                        "-I",str(BSP/"applications"),str(source),"-o",str(binary)],check=True)
        subprocess.run([str(binary)],check=True)

if __name__ == "__main__":
    main()
