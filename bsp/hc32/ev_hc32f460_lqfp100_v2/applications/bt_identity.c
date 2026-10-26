/* SPDX-License-Identifier: Apache-2.0 */
#include "bt_identity.h"
#include "bt_name_policy.h"
#include "ntn_link.h"
#include "ble_uart.h"
#include "param_manager.h"
#include "ntn_ms150.h"
#include "pc_rs232.h"
#include "hw_power.h"
#include <string.h>

/* One policy lock covers changes and ACK reconciliation. Never hold it over
 * an NTN query or while waiting for an ACK handled by the RX task. */
static struct rt_mutex s_name_lock;
static rt_bool_t s_ready,s_online,s_connected,s_refresh;
static char s_actual[32],s_mac[13],s_modem_sn[33],s_imei[16];
static const char *s_source="CACHED",*s_state="PENDING";
static rt_uint32_t s_revision=1,s_ack_revision;
static rt_uint16_t s_sequence,s_waiting;
static rt_tick_t s_last_reply;
static int s_error;
static rt_uint32_t get32(const rt_uint8_t *p) {
    return ((rt_uint32_t)p[0]<<24)|((rt_uint32_t)p[1]<<16)|((rt_uint32_t)p[2]<<8)|p[3];
}
static void put32(rt_uint8_t *p,rt_uint32_t v) {
    p[0]=(rt_uint8_t)(v>>24);p[1]=(rt_uint8_t)(v>>16);p[2]=(rt_uint8_t)(v>>8);p[3]=(rt_uint8_t)v;
}
static const char *mode_name(const param_bt_config_t *c) {return c->mode==2U?"MANUAL":"AUTO";}
static void compute_locked(param_bt_config_t *c) {
    char desired[32];const char *identity=RT_NULL;const char *source="BT_MAC";
    if (c->mode==2U) {s_source="MANUAL";return;}
    if (!s_mac[0]) {s_source="CACHED";return;}
    if (btn_valid(c->dev_sn,32)) {identity=c->dev_sn;source="TERMINAL_SN";}
    else if (s_modem_sn[0]) {identity=s_modem_sn;source="MODEM_SN";}
    else if (s_imei[0]) {identity=s_imei;source="IMEI";}
    /* Keep last automatic identity while a modem is temporarily unavailable. */
    else if (btn_valid(c->ble_name,31) && strcmp(c->ble_name,"NTN-Terminal") && strcmp(s_source,"BT_MAC")) {
        s_source="CACHED";return;
    }
    btn_auto(desired,identity,s_mac);
    if (strcmp(c->ble_name,desired)) {
        if (param_set_bt_name(desired,1U,RT_TRUE)!=RT_EOK) {
            s_error=3;s_state="SAVE_FAILED";return;
        }
        s_revision++;s_state="PENDING";s_error=0;param_get_bt_snapshot(c);
    }
    s_source=source;
}
static void snapshot_locked(char *out,rt_size_t cap) {
    param_bt_config_t c;char mac[18]="";
    param_get_bt_snapshot(&c);
    if (s_mac[0]) rt_snprintf(mac,sizeof(mac),"%.2s:%.2s:%.2s:%.2s:%.2s:%.2s",
        s_mac,s_mac+2,s_mac+4,s_mac+6,s_mac+8,s_mac+10);
    rt_snprintf(out,cap,"\r\n+MCUBTNAME:{\"mode\":\"%s\",\"desired\":\"%s\","
        "\"applied\":\"%s\",\"source\":\"%s\",\"state\":\"%s\",\"revision\":%u,"
        "\"mac\":\"%s\",\"connected\":%s,\"link_version\":%u,\"last_error\":%d}\r\nOK\r\n",
        mode_name(&c),c.ble_name,s_actual,s_source,s_state,s_revision,mac,
        s_connected?"true":"false",s_online?1U:0U,s_error);
}
void bt_identity_link_reply(rt_uint16_t id,const rt_uint8_t *p,rt_uint16_t len) {
    param_bt_config_t c;unsigned i;unsigned char any=0,all=0xff;char actual[32];
    if (!s_ready || len<13U || p[12]>31U || len!=13U+p[12]) return;
    memcpy(actual,p+13,p[12]);actual[p[12]]=0;
    if (!btn_valid(actual,31)) return;
    rt_mutex_take(&s_name_lock,RT_WAITING_FOREVER);
    if (!s_waiting || id!=s_waiting) {rt_mutex_release(&s_name_lock);return;}
    for (i=2;i<8;i++) {any|=p[i];all&=p[i];}
    if (!any || all==0xff) {
        s_waiting=0;s_online=RT_FALSE;s_mac[0]=s_actual[0]=0;
        s_error=21;s_state="INVALID_MAC";rt_mutex_release(&s_name_lock);return;
    }
    s_waiting=0;s_online=RT_TRUE;s_last_reply=rt_tick_get();s_connected=p[1]!=0;
    for (i=0;i<6;i++) rt_snprintf(s_mac+i*2,3,"%02X",p[7-i]);
    strcpy(s_actual,actual);s_ack_revision=get32(p+8);
    param_get_bt_snapshot(&c);
    s_error=(p[0]>1U)?p[0]:0; /* DEFERRED is pending, not an error. */
    if (p[0]==2) s_state="SAVE_FAILED";
    else if (p[0]==3) s_state="INVALID_NAME";
    else if (!strcmp(c.ble_name,s_actual) && s_ack_revision==s_revision) s_state="APPLIED";
    else if (p[0]==1) s_state="DEFERRED";
    else s_state="PENDING";
    rt_mutex_release(&s_name_lock);
}
void bt_identity_sn_changed(void) {
    if (!s_ready) return;
    rt_mutex_take(&s_name_lock,RT_WAITING_FOREVER);s_refresh=RT_TRUE;
    rt_mutex_release(&s_name_lock);
}
rt_bool_t bt_identity_command(const char *command,char *out,rt_size_t cap) {
    const char *value=RT_NULL;int action=0;param_bt_config_t c;rt_err_t result=RT_EOK;
    if (!strcmp(command,"AT+MCUBTNAME?") || !strcmp(command,"AT+MCUBTNAME")) action=1;
    else if (!strncmp(command,"AT+MCUBTNAME=",13)) {action=2;value=command+13;}
    else if (!strcmp(command,"AT+MCUBTNAMEAUTO")) action=3;
    else if (!strcmp(command,"AT+MCUBTREFRESH")) action=4;
    else if (!strcmp(command,"AT+MCUBTIDENT?") || !strcmp(command,"AT+MCUBTIDENT")) action=5;
    else return RT_FALSE;
    if (!s_ready) {rt_snprintf(out,cap,"\r\n+MCUERROR:6\r\nERROR\r\n");return RT_TRUE;}
    if (action==2 && !btn_valid(value,31)) {
        rt_snprintf(out,cap,"\r\n+MCUERROR:1\r\nERROR\r\n");return RT_TRUE;
    }
    rt_mutex_take(&s_name_lock,RT_WAITING_FOREVER);param_get_bt_snapshot(&c);
    if (action==2) {
        if (c.mode!=2U || strcmp(c.ble_name,value)) {
            result=param_set_bt_name(value,2U,RT_TRUE);
            if (result==RT_EOK) {s_revision++;s_error=0;s_source="MANUAL";s_state="PENDING";}
        }
    } else if (action==3) {
        if (c.mode!=1U) {
            result=param_set_bt_name(c.ble_name,1U,RT_TRUE);
            if (result==RT_EOK) {s_revision++;s_error=0;s_source="BT_MAC";s_state="PENDING";}
        }
        s_refresh=RT_TRUE;
    } else if (action==4) s_refresh=RT_TRUE;
    if (result!=RT_EOK) rt_snprintf(out,cap,"\r\n+MCUERROR:3\r\nERROR\r\n");
    else if (action==5) rt_snprintf(out,cap,
        "\r\n+MCUBTIDENT:{\"terminal_sn\":\"%s\",\"modem_sn\":\"%s\",\"imei\":\"%s\",\"mac\":\"%s\"}\r\nOK\r\n",
        c.dev_sn,s_modem_sn,s_imei,s_mac);
    else {
        param_get_bt_snapshot(&c);compute_locked(&c);snapshot_locked(out,cap);
        /* OK acknowledges the accepted/saved request. Only state=APPLIED
         * means the Bluetooth readback has caught up. Poll the same query. */
    }
    rt_mutex_release(&s_name_lock);return RT_TRUE;
}
void bt_identity_poll(char *response,rt_size_t response_size) {
    static rt_tick_t next_identity;
    if (!s_ready || rt_tick_get()<rt_tick_from_millisecond(5000U)) return;
    {
        param_bt_config_t c;rt_uint8_t payload[36],kind;rt_uint16_t id,len=0;
        rt_bool_t read_identity=RT_FALSE;rt_tick_t now=rt_tick_get();
        rt_mutex_take(&s_name_lock,RT_WAITING_FOREVER);param_get_bt_snapshot(&c);
        if (s_waiting && (now-s_last_reply)>rt_tick_from_millisecond(6000)) s_waiting=0;
        if ((now-s_last_reply)>rt_tick_from_millisecond(8000)) {
            s_online=RT_FALSE;s_actual[0]=0;s_connected=RT_FALSE;s_error=20;s_state="LINK_TIMEOUT";
        }
        compute_locked(&c);
        if (c.mode!=2U && !c.dev_sn[0] &&
            (s_refresh || (rt_int32_t)(now-next_identity)>=0)) read_identity=RT_TRUE;
        s_refresh=RT_FALSE;
        kind=NTNL_INFO;
        if (s_online && (strcmp(c.ble_name,s_actual) || s_ack_revision!=s_revision) && btn_valid(c.ble_name,31)) {
            kind=NTNL_SET_NAME;put32(payload,s_revision);len=(rt_uint16_t)strlen(c.ble_name);
            memcpy(payload+4,c.ble_name,len);len+=4;
        }
        id=++s_sequence;if (!id) id=++s_sequence;s_waiting=id;
        /* Start an independent request timeout; do not reset link liveness. */
        rt_mutex_release(&s_name_lock);
        if (hw_power_bt_get()) ble_management_send(kind,id,payload,len);
        if (read_identity && pc_rs232_is_idle(1000) && !ntn_debug_lock_is_held()) {
            char sn[33]="",imei[16]="";
            rt_err_t a=ntn_identity_query("AT+CGSN=0",response,response_size);
            if (a==RT_EOK) btn_extract(response,0,sn,sizeof(sn));
            a=ntn_identity_query("AT+CGSN=1",response,response_size);
            if (a==RT_EOK) btn_extract(response,1,imei,sizeof(imei));
            rt_mutex_take(&s_name_lock,RT_WAITING_FOREVER);
            if (sn[0]) strcpy(s_modem_sn,sn);
            if (imei[0]) strcpy(s_imei,imei);
            /* Read current mode again; an old query may never override MANUAL. */
            param_get_bt_snapshot(&c);compute_locked(&c);rt_mutex_release(&s_name_lock);
            next_identity=rt_tick_get()+rt_tick_from_millisecond((sn[0]||imei[0])?600000U:60000U);
        }

    }
}
static int bt_identity_init(void) {
    if (rt_mutex_init(&s_name_lock,"btname",RT_IPC_FLAG_PRIO)!=RT_EOK) return -RT_ERROR;
    s_last_reply=rt_tick_get();s_ready=RT_TRUE;
    return RT_EOK;
}
INIT_APP_EXPORT(bt_identity_init);
#ifdef RT_USING_FINSH
#include <finsh.h>
static int bt_name(int argc,char **argv) {
    char cmd[64],out[640];
    if (argc==1) strcpy(cmd,"AT+MCUBTNAME?");
    else if (argc==2 && !strcmp(argv[1],"auto")) strcpy(cmd,"AT+MCUBTNAMEAUTO");
    else if (argc==2 && !strcmp(argv[1],"refresh")) strcpy(cmd,"AT+MCUBTREFRESH");
    else if (argc==3 && !strcmp(argv[1],"set") && btn_valid(argv[2],31))
        rt_snprintf(cmd,sizeof(cmd),"AT+MCUBTNAME=%s",argv[2]);
    else {rt_kprintf("bt_name [set <ASCII-name>|auto|refresh]\n");return -RT_EINVAL;}
    bt_identity_command(cmd,out,sizeof(out));rt_kprintf("%s",out);return RT_EOK;
}
MSH_CMD_EXPORT(bt_name, query or set Bluetooth name policy);
#endif
