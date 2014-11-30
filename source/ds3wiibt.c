#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <ogc/machine/processor.h>
#include <bte/bte.h>
#include <bte/l2cap.h>
#include <bte/btpbuf.h>
#include "ds3wiibt.h"

static const unsigned char led_pattern[] = {
	0x0, 0x02, 0x04, 0x08, 0x10, 0x12, 0x14, 0x18, 0x1A, 0x1C, 0x1E
};

static int senddata_raw(struct l2cap_pcb *pcb, const void *message, u16 len);
static int set_operational(struct ds3wiibt_context *ctx);
static int send_output_report(struct ds3wiibt_context *ctx);
static void correct_input(struct ds3wiibt_input *inp);

static err_t l2ca_connect_ind_cb(void *arg, struct l2cap_pcb *pcb, err_t err);

static err_t l2ca_disconnect_ind_cb(void *arg, struct l2cap_pcb *pcb, err_t err);
static err_t l2ca_disconnect_cfm_cb(void *arg, struct l2cap_pcb *pcb);
static err_t l2ca_timeout_ind_cb(void *arg, struct l2cap_pcb *pcb, err_t err);
static err_t l2ca_recv_cb(void *arg, struct l2cap_pcb *pcb, struct pbuf *p, err_t err);

void ds3wiibt_initialize(struct ds3wiibt_context *ctx)
{
	ctx->ctrl_pcb = NULL;
	ctx->data_pcb = NULL;
	memset(&ctx->bdaddr, 0, sizeof(ctx->bdaddr));
	memset(&ctx->input,  0, sizeof(ctx->input));
	ds3wiibt_set_led(ctx, 1);
	ds3wiibt_set_rumble(ctx, 0x00, 0x00, 0x00, 0x00);
	ctx->usrdata = NULL;
	ctx->connect_cb = NULL;
	ctx->disconnect_cb = NULL;
	ctx->status = DS3WIIBT_STATUS_DISCONNECTED;
}

void ds3wiibt_set_userdata(struct ds3wiibt_context *ctx, void *data)
{
	ctx->usrdata = data;
}

void ds3wiibt_set_connect_cb(struct ds3wiibt_context *ctx, ds3wiibt_cb cb)
{
	ctx->connect_cb = cb;
}

void ds3wiibt_set_disconnect_cb(struct ds3wiibt_context *ctx, ds3wiibt_cb cb)
{
	ctx->disconnect_cb = cb;
}

void ds3wiibt_set_led(struct ds3wiibt_context *ctx, u8 led)
{
	ctx->led = led;
}

void ds3wiibt_set_rumble(struct ds3wiibt_context *ctx, u8 duration_right, u8 power_right, u8 duration_left, u8 power_left)
{
	ctx->rumble.duration_right = duration_right;
	ctx->rumble.power_right = power_right;
	ctx->rumble.duration_left = duration_left;
	ctx->rumble.power_left = power_left;
}

void ds3wiibt_send_ledsrumble(struct ds3wiibt_context *ctx)
{
	send_output_report(ctx);
}

void ds3wiibt_connect(struct ds3wiibt_context *ctx, struct bd_addr *addr)
{
	u32 level = IRQ_Disable();
	
	bd_addr_set(&ctx->bdaddr, addr);

	ctx->ctrl_pcb = l2cap_new();
	ctx->data_pcb = l2cap_new();
	
	l2cap_arg(ctx->ctrl_pcb, ctx);
	l2cap_arg(ctx->data_pcb, ctx);
	
	l2cap_disconnect_ind(ctx->ctrl_pcb, l2ca_disconnect_ind_cb);
	l2cap_disconnect_ind(ctx->data_pcb, l2ca_disconnect_ind_cb);
	
	l2cap_timeout_ind(ctx->ctrl_pcb, l2ca_timeout_ind_cb);
	l2cap_timeout_ind(ctx->data_pcb, l2ca_timeout_ind_cb);
	
	l2cap_connect_ind(ctx->ctrl_pcb,  &ctx->bdaddr, HIDP_PSM, l2ca_connect_ind_cb);
	l2cap_connect_ind(ctx->data_pcb,  &ctx->bdaddr, INTR_PSM, l2ca_connect_ind_cb);

	IRQ_Restore(level);
}

void ds3wiibt_close(struct ds3wiibt_context *ctx)
{
	//Turn OFF the LED and Rumble
	ds3wiibt_set_led(ctx, 0);
	ds3wiibt_set_rumble(ctx, 0x00, 0x00, 0x00, 0x00);
	ds3wiibt_send_ledsrumble(ctx);
	if (ds3wiibt_is_connected(ctx)) {
		if (ctx->ctrl_pcb) l2ca_disconnect_req(ctx->ctrl_pcb, l2ca_disconnect_cfm_cb);
		if (ctx->data_pcb) l2ca_disconnect_req(ctx->data_pcb, l2ca_disconnect_cfm_cb);
	} else {
		if (ctx->ctrl_pcb) l2cap_close(ctx->ctrl_pcb);
		if (ctx->data_pcb) l2cap_close(ctx->data_pcb);
		ctx->ctrl_pcb = NULL;
		ctx->data_pcb = NULL;
	}
	while (ds3wiibt_is_connected(ctx)) usleep(50);
}

static err_t l2ca_recv_cb(void *arg, struct l2cap_pcb *pcb, struct pbuf *p, err_t err)
{
	struct ds3wiibt_context *ctx = (struct ds3wiibt_context *)arg;
	if (ctx == NULL || pcb == NULL || err != ERR_OK) return -1;

	u8 *rd = p->payload;

	//LOG("RECV, PSM: 0x%02X  len: %i |", l2cap_psm(pcb), p->len);
	//LOG(" 0x%X  0x%X  0x%X  0x%X  0x%X\n", rd[0], rd[1], rd[2], rd[3], rd[4]);

	switch (l2cap_psm(pcb))	{
	case HIDP_PSM:
		switch (rd[0]) {
		case 0x00:
			if (ctx->status == DS3WIIBT_STATUS_DISCONNECTED) {
				send_output_report(ctx);
				ctx->status = DS3WIIBT_STATUS_CONNECTED;
				if (ctx->connect_cb != NULL) {
					//Notify the user
					ctx->connect_cb(ctx->usrdata);
				}
			}
			break;
		}
		break;
	case INTR_PSM:
		switch (rd[1]) {
		case 0x01: //Full report
			memcpy(&ctx->input, rd + 1, sizeof(ctx->input));
			break;
		}
		break;
	}
	return ERR_OK;
}

static err_t l2ca_connect_ind_cb(void *arg, struct l2cap_pcb *pcb, err_t err)
{
	struct ds3wiibt_context *ctx = (struct ds3wiibt_context *)arg;
	if (ctx == NULL || pcb == NULL) return -1;
	
	//LOG("l2ca_connect_ind_cb, PSM: 0x%02X\n", l2cap_psm(pcb));
	
	if (l2cap_psm(pcb) == HIDP_PSM) {
		/* Control PSM is connected */
		l2cap_recv(ctx->ctrl_pcb, l2ca_recv_cb);
	} else if (l2cap_psm(pcb) == INTR_PSM) {
		/* Both Control PSM and Data PSM are connected */
		l2cap_recv(ctx->data_pcb, l2ca_recv_cb);
		set_operational(ctx);
	}
	return ERR_OK;
}

static err_t l2ca_disconnect_ind_cb(void *arg, struct l2cap_pcb *pcb, err_t err)
{
	struct ds3wiibt_context *ctx = (struct ds3wiibt_context *)arg;
	if (ctx == NULL || pcb == NULL) return -1;
	
	LOG("l2ca_disconnect_ind_cb, PSM: 0x%02X\n", l2cap_psm(pcb));
	
	switch (l2cap_psm(pcb))	{
	case HIDP_PSM:
		l2cap_close(ctx->ctrl_pcb);
		ctx->ctrl_pcb = NULL;
		break;
	case INTR_PSM:
		l2cap_close(ctx->data_pcb);
		ctx->data_pcb = NULL;
		break;
	}
	
	//If the Data and Control PSMs are disconnected, listen for an incoming connection again
	if ((ctx->ctrl_pcb == NULL) && (ctx->data_pcb == NULL)) {
		
		ctx->status = DS3WIIBT_STATUS_DISCONNECTED;
		if (ctx->disconnect_cb != NULL) ctx->disconnect_cb(ctx->usrdata);
		
		ctx->ctrl_pcb = l2cap_new();
		ctx->data_pcb = l2cap_new();
		
		l2cap_arg(ctx->ctrl_pcb, ctx);
		l2cap_arg(ctx->data_pcb, ctx);
		
		l2cap_disconnect_ind(ctx->ctrl_pcb, l2ca_disconnect_ind_cb);
		l2cap_disconnect_ind(ctx->data_pcb, l2ca_disconnect_ind_cb);
		
		l2cap_timeout_ind(ctx->ctrl_pcb, l2ca_timeout_ind_cb);
		l2cap_timeout_ind(ctx->data_pcb, l2ca_timeout_ind_cb);

		l2cap_recv(ctx->ctrl_pcb, l2ca_recv_cb);
		l2cap_recv(ctx->data_pcb, l2ca_recv_cb);
	}

	return ERR_OK;
}

static err_t l2ca_disconnect_cfm_cb(void *arg, struct l2cap_pcb *pcb)
{
	struct ds3wiibt_context *ctx = (struct ds3wiibt_context *)arg;
	if (ctx == NULL || pcb == NULL) return -1;
	
	LOG("l2ca_disconnect_cfm_cb, PSM: 0x%02X\n", l2cap_psm(pcb));
	
	switch (l2cap_psm(pcb))	{
	case HIDP_PSM:
		l2cap_close(ctx->ctrl_pcb);
		ctx->ctrl_pcb = NULL;
		break;
	case INTR_PSM:
		l2cap_close(ctx->data_pcb);
		ctx->data_pcb = NULL;
		break;
	}

	//User has finished it, so don't listen again
	if ((ctx->ctrl_pcb == NULL) && (ctx->data_pcb == NULL)) {
		ctx->status = DS3WIIBT_STATUS_DISCONNECTED;
		if (ctx->disconnect_cb != NULL) ctx->disconnect_cb(ctx->usrdata);
	}

	return ERR_OK;
}

static err_t l2ca_timeout_ind_cb(void *arg, struct l2cap_pcb *pcb, err_t err)
{
	struct ds3wiibt_context *ctx = (struct ds3wiibt_context *)arg;
	if (ctx == NULL || pcb == NULL) return -1;
	
	LOG("l2ca_timeout_ind_cb, PSM: 0x%02X\n", l2cap_psm(pcb));

	//Disconnect?
	switch (l2cap_psm(pcb))	{
	case HIDP_PSM:
		break;
	case INTR_PSM:
		break;
	}

	return ERR_OK;
}


static int senddata_raw(struct l2cap_pcb *pcb, const void *message, u16 len)
{
	err_t err;
	struct pbuf *p;
	if ((pcb == NULL) || (message==NULL) || (len==0)) return ERR_VAL;
	if ((p = btpbuf_alloc(PBUF_RAW, (len), PBUF_RAM)) == NULL) {
		return ERR_MEM;
	}
	memcpy(p->payload, message, len);
	err = l2ca_datawrite(pcb, p);
	btpbuf_free(p);
	return err;
}

static int set_operational(struct ds3wiibt_context *ctx)
{
	static const unsigned char buf[] ATTRIBUTE_ALIGN(32) = {
		(HIDP_TRANS_SETREPORT | HIDP_DATA_RTYPE_FEATURE),
		0xF4, 0x42, 0x03, 0x00, 0x00
	};
	return senddata_raw(ctx->ctrl_pcb, buf, sizeof(buf));
}

static int send_output_report(struct ds3wiibt_context *ctx)
{
	unsigned char buf[] ATTRIBUTE_ALIGN(32) = {
		(HIDP_TRANS_SETREPORT | HIDP_DATA_RTYPE_OUPUT),
		0x01, //Report ID
		0x00, //Padding
		0x00, 0x00, 0x00, 0x00, //Rumble (r, r, l, l)
		0x00, 0x00, 0x00, 0x00, //Padding
		0x00, /* LED_1 = 0x02, LED_2 = 0x04, ... */
		0xff, 0x27, 0x10, 0x00, 0x32, /* LED_4 */
		0xff, 0x27, 0x10, 0x00, 0x32, /* LED_3 */
		0xff, 0x27, 0x10, 0x00, 0x32, /* LED_2 */
		0xff, 0x27, 0x10, 0x00, 0x32, /* LED_1 */
		0x00, 0x00, 0x00, 0x00, 0x00  /* LED_5 (not soldered) */
	};
	
	buf[3] = ctx->rumble.duration_right;
	buf[4] = ctx->rumble.power_right;
	buf[5] = ctx->rumble.duration_left;
	buf[6] = ctx->rumble.power_left;
	buf[11] = led_pattern[ctx->led%11];

	return senddata_raw(ctx->ctrl_pcb, buf, sizeof(buf));
}

