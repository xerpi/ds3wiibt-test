#include <stdio.h>
#include <unistd.h>
#include <gccore.h>
#include <ogc/machine/processor.h>
#include <fat.h>
#include <wiiuse/wpad.h>
#include "ds3wiibt.h"
#include "utils.h"

//Controller's MAC: 00:24:33:63:4B:6B
struct bd_addr addr = {.addr = {0x6B, 0x4B, 0x63, 0x33, 0x24, 0x00}};
//struct bd_addr addr = {.addr = {0x00, 0x24, 0x33, 0x63, 0x4B, 0x6B}};

static void print_data(struct ds3wiibt_input *inp);
static void conn_cb(void *usrdata);
static void discon_cb(void *usrdata);

int main(int argc, char *argv[])
{
	fatInitDefault();
	WPAD_Init();
	init_video();
	SYS_SetResetCallback(button_cb);
	SYS_SetPowerCallback(button_cb);
	printf("ds3wiibt by xerpi\n");


	struct ds3wiibt_context ctx;
	ds3wiibt_initialize(&ctx);
	ds3wiibt_set_userdata(&ctx, NULL);
	ds3wiibt_set_connect_cb(&ctx, conn_cb);
	ds3wiibt_set_disconnect_cb(&ctx, discon_cb);

	LOG("Connecting to: ");
	print_mac((struct bd_addr*)addr.addr);
	ds3wiibt_connect(&ctx, &addr);
	printf("Listening for an incoming connection...\n");
	
	while (run) {
		WPAD_ScanPads();
		u32 pressed = WPAD_ButtonsDown(0);
		if (pressed & WPAD_BUTTON_HOME) run = 0;
		if (ds3wiibt_is_connected(&ctx)) {
			print_data(&ctx.input);
			if (ctx.input.PS && ctx.input.start) run = 0;
		}
		flip_screen();
	}
	//ds3wiibt_close(&ctx);
	return 0;
}

static void conn_cb(void *usrdata)
{
	printf("Controller connected.\n");
}

static void discon_cb(void *usrdata)
{
	printf("Controller disconnected.\n");
}

static void print_data(struct ds3wiibt_input *inp)
{
	printf("\x1b[10;0H");
	printf("\n\nPS: %i   START: %i   SELECT: %i   /\\: %i   []: %i   O: %i   X: %i   L3: %i   R3: %i\n", \
		inp->PS, inp->start, inp->select, inp->triangle, \
		inp->square, inp->circle, inp->cross, inp->L3, inp->R3);

	printf("L1: %i   L2: %i   R1: %i   R2: %i   UP: %i   DOWN: %i   RIGHT: %i   LEFT: %i\n", \
		inp->L1, inp->L2, inp->R1, inp->R2, \
		inp->up, inp->down, inp->right, inp->left);

	printf("LX: 0x%02X   LY: 0x%02X   RX: 0x%02X   RY: 0x%02X\n", \
		inp->leftX, inp->leftY, inp->rightX, inp->rightY);

	printf("aX: 0x%04X   aY: 0x%04X   aZ: 0x%04X   Zgyro: 0x%04X\n", \
		inp->accelX, inp->accelY, inp->accelZ, inp->gyroZ);

	printf("L1 sens: 0x%02X   L2 sens: 0x%02X   R1 sens: 0x%02X   R2 sens: 0x%02X\n", \
		inp->L1_sens, inp->L2_sens, inp->R1_sens, inp->R2_sens);

	printf("/\\ sens: 0x%02X   [] sens: 0x%02X   O sens: 0x%02X   X sens: 0x%02X\n",
		inp->triangle_sens, inp->square_sens, inp->circle_sens, inp->cross_sens);

	printf("UP: 0x%02X   DOWN: 0x%02X   RIGHT: 0x%02X   LEFT: 0x%02X\n", \
		inp->up_sens, inp->down_sens, inp->right_sens, inp->left_sens);
}
