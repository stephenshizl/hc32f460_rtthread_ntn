"""Exercise production service state transitions with mocked OS/flash/UART ports."""
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
BSP=Path(__file__).resolve().parents[1]
STUBS=r'''
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
typedef unsigned char rt_uint8_t;
typedef unsigned short rt_uint16_t;
typedef unsigned rt_uint32_t,rt_tick_t;
typedef int rt_int32_t,rt_bool_t,rt_err_t;
typedef size_t rt_size_t;
struct rt_mutex {int locked;};
#define RT_TRUE 1
#define RT_FALSE 0
#define RT_NULL NULL
#define RT_EOK 0
#define RT_ERROR 1
#define RT_EINVAL 10
#define RT_ENOMEM 5
#define RT_WAITING_FOREVER -1
#define INIT_APP_EXPORT(x)
#define RT_UNUSED(x) (void)(x)
#define rt_snprintf snprintf
typedef struct {char ble_name[32],dev_sn[33];rt_uint8_t mode;} param_bt_config_t;
static param_bt_config_t config={"NTN-Terminal","",1};
static unsigned now=6000,writes,queries,revision_on_wire;static int fail_save,quiet=1,debug_lock,race;
static unsigned short wire_id;static unsigned char wire_kind;static char wire_name[32];
static unsigned rt_tick_get(void) {return now;}
static unsigned rt_tick_from_millisecond(unsigned v) {return v;}
static int rt_mutex_init(struct rt_mutex *m,const char *n,int flags) {(void)n;(void)flags;m->locked=0;return 0;}
#define RT_IPC_FLAG_PRIO 0
static int rt_mutex_take(struct rt_mutex *m,int timeout) {(void)timeout;assert(!m->locked);m->locked=1;return 0;}
static void rt_mutex_release(struct rt_mutex *m) {assert(m->locked);m->locked=0;}
static void param_get_bt_snapshot(param_bt_config_t *c) {*c=config;}
static int param_set_bt_name(const char *name,unsigned char mode,int save) {
 (void)save;if(fail_save)return -1;
 if(config.mode!=mode || strcmp(config.ble_name,name)){writes++;strcpy(config.ble_name,name);config.mode=mode;}
 return 0;
}
static int hw_power_bt_get(void) {return 1;}
static int pc_rs232_is_idle(unsigned t) {(void)t;return quiet;}
static int ntn_debug_lock_is_held(void) {return debug_lock;}
static size_t ble_management_send(unsigned char kind,unsigned short id,const void *payload,unsigned short len) {
 const unsigned char *p=payload;wire_kind=kind;wire_id=id;
 if(len) {revision_on_wire=((unsigned)p[0]<<24)|((unsigned)p[1]<<16)|((unsigned)p[2]<<8)|p[3];
 memcpy(wire_name,p+4,len-4);wire_name[len-4]=0;}
 return len+1;
}
static int ntn_identity_query(const char *,char *,size_t);
'''
CASES=r'''
static int ntn_identity_query(const char *cmd,char *out,size_t capacity) {
 queries++;
 if(race){char result[640];race=0;bt_identity_command("AT+MCUBTNAME=RaceManual",result,sizeof(result));}
 snprintf(out,capacity,!strcmp(cmd,"AT+CGSN=0")?"+CGSN:yixing_249_005\r\nOK\r\n":"+CGSN:862130042600001\r\nOK\r\n");
 return 0;
}
static void reply(unsigned char status,unsigned revision,const char *name,unsigned short id) {
 unsigned char p[44];unsigned n=strlen(name);p[0]=status;p[1]=status==1;
 p[2]=0xc0;p[3]=0xd7;p[4]=0xfd;p[5]=0x72;p[6]=0xd9;p[7]=0x98;
 put32(p+8,revision);p[12]=n;memcpy(p+13,name,n);bt_identity_link_reply(id,p,n+13);
}
int main(void) {
 char scratch[2049],out[640];unsigned count,rev;unsigned short old_id;
 assert(bt_identity_init()==0);bt_identity_poll(scratch,sizeof(scratch));assert(wire_kind==NTNL_INFO);
 reply(0,0,"NTN-Terminal-98D972FDD7C0",wire_id);
 now+=2000;bt_identity_poll(scratch,sizeof(scratch));
 assert(!strcmp(config.ble_name,"NTN-Terminal-yixing_249_005") && wire_kind==NTNL_SET_NAME);
 reply(0,s_revision,wire_name,wire_id);assert(!strcmp(s_state,"APPLIED"));
 puts("PASS AUTO real SN, paired UART readback before APPLIED");
 assert(bt_identity_command("AT+MCUBTNAME=Manual_A",out,sizeof(out)));
 assert(config.mode==2 && !strcmp(s_source,"MANUAL") && !strcmp(s_state,"PENDING"));
 count=writes;rev=s_revision;bt_identity_command("AT+MCUBTNAME=Manual_A",out,sizeof(out));
 assert(writes==count && rev==s_revision);puts("PASS repeated setting is idempotent");
 now+=2000;bt_identity_poll(scratch,sizeof(scratch));old_id=wire_id;
 reply(0,rev-1,"Manual_A",wire_id);assert(strcmp(s_state,"APPLIED"));
 now+=2000;bt_identity_poll(scratch,sizeof(scratch));reply(0,rev,"Manual_A",old_id);
 assert(s_waiting==wire_id);reply(1,rev,"NTN-Terminal-yixing_249_005",wire_id);
 assert(!strcmp(s_state,"DEFERRED") && s_error==0);puts("PASS stale revision/id ignored and connected rename deferred");
 now+=2000;bt_identity_poll(scratch,sizeof(scratch));reply(0,rev,"Manual_A",wire_id);
 assert(!strcmp(s_state,"APPLIED"));puts("PASS deferred apply reconciles on next real readback");
 fail_save=1;bt_identity_command("AT+MCUBTNAME=Unwritten",out,sizeof(out));
 assert(strstr(out,"ERROR")&&!strcmp(config.ble_name,"Manual_A"));fail_save=0;
 puts("PASS flash-save failure never changes accepted policy");
 bt_identity_command("AT+MCUBTNAME=",out,sizeof(out));assert(strstr(out,"ERROR"));
 bt_identity_command("AT+MCUBTNAME=bad,name",out,sizeof(out));assert(strstr(out,"ERROR"));
 bt_identity_command("AT+MCUBTNAME=12345678901234567890123456789012",out,sizeof(out));assert(strstr(out,"ERROR"));
 bt_identity_command("AT+MCUBTNAME=AUTO",out,sizeof(out));assert(config.mode==2&&!strcmp(config.ble_name,"AUTO"));
 puts("PASS invalid names rejected; literal AUTO remains manual");
 bt_identity_command("AT+MCUBTNAMEAUTO",out,sizeof(out));assert(config.mode==1);
 assert(!strcmp(config.ble_name,"NTN-Terminal-yixing_249_005"));
 race=1;now+=2000;bt_identity_poll(scratch,sizeof(scratch));
 assert(config.mode==2&&!strcmp(config.ble_name,"RaceManual"));
 puts("PASS old background identity result cannot override a manual request");
 strcpy(config.dev_sn,"TERM0001");bt_identity_command("AT+MCUBTNAMEAUTO",out,sizeof(out));
 assert(!strcmp(config.ble_name,"NTN-Terminal-TERM0001"));
 strcpy(config.dev_sn,"12345678901234567890123456789012");bt_identity_sn_changed();
 now+=2000;bt_identity_poll(scratch,sizeof(scratch));assert(strlen(config.ble_name)==31);
 assert(!strcmp(config.ble_name+19,"98D972FDD7C0"));puts("PASS terminal SN priority and long-SN MAC suffix");
 config.dev_sn[0]=0;quiet=0;count=queries;bt_identity_sn_changed();now+=2000;
 bt_identity_poll(scratch,sizeof(scratch));assert(queries==count);quiet=1;
 debug_lock=1;bt_identity_sn_changed();now+=2000;bt_identity_poll(scratch,sizeof(scratch));assert(queries==count);debug_lock=0;
 puts("PASS PC traffic and debug lease defer identity reads");
 now+=9000;bt_identity_poll(scratch,sizeof(scratch));assert(!s_online&&!s_actual[0]&&!strcmp(s_state,"LINK_TIMEOUT"));
 puts("PASS link loss clears actual name and does not claim applied");
 {
 unsigned char bad[14]={0};bad[12]=1;bad[13]='X';
 bt_identity_link_reply(wire_id,bad,sizeof(bad));
 assert(!s_online && !s_actual[0] && !strcmp(s_state,"INVALID_MAC"));
 }
 puts("PASS invalid MAC never claims unique/applied identity");
 puts("All 11 production service groups passed");return 0;
}
'''
def main():
 source=(BSP/'applications/bt_identity.c').read_text(encoding='utf-8')
 source=re.sub(r'^#include .*$', '', source,flags=re.M)
 source=source.split('#ifdef RT_USING_FINSH')[0]
 code=STUBS+'\n#include "ntn_link.h"\n#include "bt_name_policy.h"\n'+source+CASES
 with tempfile.TemporaryDirectory(prefix='bt_service_') as tmp:
  p=Path(tmp)/'test.c';exe=Path(tmp)/('test.exe' if os.name=='nt' else 'test');p.write_text(code,encoding='utf-8')
  subprocess.run([os.environ.get('HOST_CC') or shutil.which('gcc'),'-std=c99','-Wall','-Wextra','-Werror','-I',str(BSP/'applications'),str(p),'-o',str(exe)],check=True)
  subprocess.run([str(exe)],check=True)
if __name__=='__main__':main()
