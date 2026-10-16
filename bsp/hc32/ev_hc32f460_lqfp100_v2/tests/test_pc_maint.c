#include "pc_maint_core.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static unsigned char raw[2048];static size_t rn;static char cmd[30][300];static int cn;
static void onraw(void *u,const uint8_t *p,size_t n){(void)u; assert(rn+n<=sizeof(raw));memcpy(raw+rn,p,n);rn+=n;}
static void oncmd(void *u,const uint8_t *p,size_t n){(void)u;assert(cn<30);assert(n<300);memcpy(cmd[cn],p,n);cmd[cn][n]=0;cn++;}
static const pc_maint_callbacks_t cb={onraw,oncmd,0};
static void feed(pc_maint_ctx_t *s,const char *p,uint32_t t,int en){pc_maint_input(s,(const uint8_t*)p,strlen(p),t,en,&cb);}
static void reset(pc_maint_ctx_t *s){pc_maint_init(s);rn=0;cn=0;memset(raw,0,sizeof(raw));memset(cmd,0,sizeof(cmd));}
int main(void){pc_maint_ctx_t s;reset(&s);
 /* Guard 1000ms, triple+ <=100ms, post guard, config entered then ordinary AT. */
 feed(&s,"+",1200,1);feed(&s,"+",1240,1);feed(&s,"+",1290,1);
 pc_maint_poll(&s,2300,1,&cb);feed(&s,"AT+MCUCFG=1\r\n",2450,1);
 assert(pc_maint_active(&s)&&cn==1&&!strcmp(cmd[0],"AT+MCUCFG=1")&&rn==0);
 feed(&s,"AT+MCUVER?\r\n",2480,1);assert(cn==2&&!strcmp(cmd[1],"AT+MCUVER?"));
 feed(&s,"AT+MCUCFG=0\r\n",2520,1);assert(cn==3&&!pc_maint_active(&s)&&rn==0);
 feed(&s,"DATA",2530,1);assert(rn==4&&!memcmp(raw,"DATA",4));
 /* Bad escape suffix must replay exactly once and preserve following bytes. */
 reset(&s);feed(&s,"RAW",200,1);feed(&s,"+++x",1500,1);
 assert(rn==7&&!memcmp(raw,"RAW+++x",7));
 /* Candidate fragmented then incomplete: exact original bytes go to RAW. */
 reset(&s);feed(&s,"++",1500,1);pc_maint_poll(&s,1700,1,&cb);
 assert(rn==2&&!memcmp(raw,"++",2));
 /* Invalid confirm command -> RAW exactly, including CR and LF. */
 reset(&s);feed(&s,"+++",2000,1);pc_maint_poll(&s,3100,1,&cb);
 feed(&s,"AT+BAD\r\n",3250,1);assert(cn==0);
 assert(rn==11&&!memcmp(raw,"+++AT+BAD\r\n",11));
 /* 3s timeout after 1s post quiet. */
 reset(&s);feed(&s,"+++",2000,1);pc_maint_poll(&s,6500,1,&cb);
 assert(rn==3&&!memcmp(raw,"+++",3));
 /* Disabling entry must never intercept RAW data. */
 reset(&s);feed(&s,"+++AT+MCUCFG=1\r",2000,0);
 assert(!pc_maint_active(&s)&&rn==strlen("+++AT+MCUCFG=1\r"));
 /* 60 second idle expiration discards incomplete control, then resumes RAW. */
 reset(&s);feed(&s,"+++",2000,1);pc_maint_poll(&s,3000,1,&cb);
 feed(&s,"AT+MCUCFG=1\r",3200,1);feed(&s,"AT+MCU",3210,1);
 pc_maint_poll(&s,64000,1,&cb);assert(!pc_maint_active(&s));
 feed(&s,"test",64030,1);assert(rn==4&&!memcmp(raw,"test",4));
 /* Embedded binary byte remains unmodified in RAW mode. */
 reset(&s);{uint8_t p[]={0,0xff,0x0d,0x0a,'+','+'};pc_maint_input(&s,p,sizeof(p),1500,1,&cb);
 assert(rn==sizeof(p)&&!memcmp(raw,p,sizeof(p)));}
 /* Must not recognize +++ during first second of boot. */
 reset(&s);feed(&s,"+++",100,1);assert(rn==3&&!memcmp(raw,"+++",3));
 /* Hostname-size local AT line is valid after entry (candidate remains capped). */
 reset(&s);feed(&s,"+++",2000,1);pc_maint_poll(&s,3000,1,&cb);
 feed(&s,"AT+MCUCFG=1\r",3100,1);
 {char long_cmd[290];size_t i;memcpy(long_cmd,"AT+MCUHOST=",11);
 for(i=11;i<270;i++) long_cmd[i]='a';
 long_cmd[270]='\r';
 long_cmd[271]=0;
 feed(&s,long_cmd,3200,1);assert(cn==2&&strlen(cmd[1])==270);}
 puts("PASS: maintenance guard/confirm/replay/timeout/exit/disabled/binary/long-AT");return 0;}
