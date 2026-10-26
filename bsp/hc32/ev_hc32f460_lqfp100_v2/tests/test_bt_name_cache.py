"""Production AC63 name cache, tested with a simulated atomic syscfg backend."""
import argparse,os,re,shutil,subprocess,tempfile
from pathlib import Path
from test_ntn_control import function
BSP=Path(__file__).resolve().parents[1]
STUBS=r'''
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include "ntn_link.h"
#include "bt_name_policy.h"
typedef uint8_t u8;typedef uint16_t u16;typedef uint32_t u32;typedef int32_t s32;
#define VM_NTN_NAME_A 44
#define VM_NTN_NAME_B 45
#define NTN_NAME_RECORD_SIZE 42U
static u32 ntn_name_generation,ntn_name_revision;
static u8 ntn_name_slot=VM_NTN_NAME_B;
static char ntn_cached_name[32],local_name[32];
static unsigned char storage[2][NTN_NAME_RECORD_SIZE];
static int writes,fail_write,corrupt_read,connected,hci;
static int syscfg_read(int id,void *out,unsigned len) {
 assert(len==NTN_NAME_RECORD_SIZE);memcpy(out,storage[id-44],len);
 if(corrupt_read)((u8 *)out)[20]^=1;
 return len;
}
static int syscfg_write(int id,const void *in,unsigned len) {
 if(fail_write)return -1;writes++;memcpy(storage[id-44],in,len);return len;
}
static const u8 *bt_get_mac_addr(void) {static u8 mac[]={0xc0,0xd7,0xfd,0x72,0xd9,0x98};return mac;}
static void bt_set_local_name(char *name,u8 n) {assert(n<=31);memcpy(local_name,name,n);local_name[n]=0;}
static const char *bt_get_local_name(void) {return local_name;}
static int transport_spp_is_connected(void) {return connected;}
static void lmp_hci_write_local_name(const char *name) {hci++;assert(!strcmp(local_name,name));}
'''
CASES=r'''
int main(void) {
 int count;u8 old_slot;
 ntn_name_revision=0;ntn_name_boot();assert(!strcmp(local_name,"NTN-Terminal-98D972FDD7C0"));
 assert(writes==0);puts("PASS missing/corrupt cache boots unique MAC name without writes");
 assert(ntn_name_save("Cache_A")==0);old_slot=ntn_name_slot;assert(writes==1);
 assert(ntn_name_save("Cache_A")==0&&writes==1);puts("PASS same-name retries do not rewrite VM");
 connected=1;ntn_name_apply_idle();assert(!strcmp(local_name,"NTN-Terminal-98D972FDD7C0")&&hci==0);
 connected=0;ntn_name_apply_idle();assert(!strcmp(local_name,"Cache_A")&&hci==1);
 puts("PASS connected rename preserves response channel; idle applies RAM and HCI");
 assert(ntn_name_save("Cache_B")==0&&ntn_name_slot!=old_slot);
 ntn_cached_name[0]=local_name[0]=0;ntn_name_generation=0;ntn_name_boot();assert(!strcmp(local_name,"Cache_B"));
 puts("PASS reboot chooses newest valid persistent slot");
 storage[ntn_name_slot-44][15]^=1;ntn_name_boot();assert(!strcmp(local_name,"Cache_A"));
 puts("PASS torn newest slot falls back to previous valid name");
 fail_write=1;count=writes;assert(ntn_name_save("Unwritten")<0&&writes==count&&!strcmp(ntn_cached_name,"Cache_A"));fail_write=0;
 puts("PASS VM write failure retains previous applied/desired cache");
 corrupt_read=1;assert(ntn_name_save("VerifyFailure")<0&&!strcmp(ntn_cached_name,"Cache_A"));corrupt_read=0;
 puts("PASS VM readback failure is not acknowledged as saved");
 assert(ntn_name_save("Final_Name")==0);ntn_name_boot();assert(!strcmp(local_name,"Final_Name"));
 puts("PASS subsequent successful transaction recovers");
 puts("All 8 production VM cache groups passed");return 0;
}
'''
def main():
 parser=argparse.ArgumentParser();parser.add_argument('--sdk',required=True,type=Path);args=parser.parse_args()
 source=(args.sdk/'apps/spp_and_le/modules/bt/bt_name_link_impl.h').read_text(encoding='utf-8')
 funcs=['ntn_name_get32','ntn_name_put32','ntn_name_record_ok','ntn_name_boot','ntn_name_save','ntn_name_apply_idle']
 code=STUBS+'\n'.join(function(source,n) for n in funcs)+CASES
 with tempfile.TemporaryDirectory(prefix='bt_cache_') as tmp:
  p=Path(tmp)/'test.c';exe=Path(tmp)/('test.exe' if os.name=='nt' else 'test');p.write_text(code,encoding='utf-8')
  subprocess.run([os.environ.get('HOST_CC') or shutil.which('gcc'),'-std=c99','-Wall','-Wextra','-Werror','-I',str(BSP/'applications'),str(p),'-o',str(exe)],check=True)
  subprocess.run([str(exe)],check=True)
if __name__=='__main__':main()
