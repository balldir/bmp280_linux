/* Copyright (c) 2016 Max Asaulov
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */

#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <errno.h>
#include <string.h>
#include <sys/ioctl.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <math.h>
#include "bmp280.h"

#define PATH_STR   "/dev/i2c-0"

static const uint8_t addr_array[] = {0b1110110, 0b1110111};
static int file = -1;

void hexDump (char *desc, void *addr, int len) {
    int i;
    unsigned char buff[17];
    unsigned char *pc = (unsigned char*)addr;

    // Output description if given.
    if (desc != NULL)
        printf ("%s:\n", desc);

    if (len == 0) {
        printf("  ZERO LENGTH\n");
        return;
    }
    if (len < 0) {
        printf("  NEGATIVE LENGTH: %i\n",len);
        return;
    }

    // Process every byte in the data.
    for (i = 0; i < len; i++) {
        // Multiple of 16 means new line (with line offset).

        if ((i % 16) == 0) {
            // Just don't print ASCII for the zeroth line.
            if (i != 0)
                printf ("  %s\n", buff);

            // Output the offset.
            printf ("  %04x ", i);
        }

        // Now the hex code for the specific character.
        printf (" %02x", pc[i]);

        // And store a printable ASCII character for later.
        if ((pc[i] < 0x20) || (pc[i] > 0x7e))
            buff[i % 16] = '.';
        else
            buff[i % 16] = pc[i];
        buff[(i % 16) + 1] = '\0';
    }

    // Pad out last line if not exactly 16 characters.
    while ((i % 16) != 0) {
        printf ("   ");
        i++;
    }

    // And print the final ASCII bit.
    printf ("  %s\n", buff);
}

 s8 BMP280_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	struct i2c_rdwr_ioctl_data packets;
    	struct i2c_msg messages[1];
	u8 out_buff[sizeof(reg_addr) + cnt];
	out_buff[0] = reg_addr;
	memcpy(&out_buff[1], reg_data, cnt);

    	messages[0].addr  = dev_addr;
    	messages[0].flags = 0;
    	messages[0].len   = sizeof(out_buff);
    	messages[0].buf   = out_buff;

    	/* Transfer the i2c packets to the kernel and verify it worked */
    	packets.msgs  = messages;
    	packets.nmsgs = 1;
    	if(ioctl(file, I2C_RDWR, &packets) < 0) {
        	perror("Unable to send data");
        	return ERROR;
    	}

	printf("register write %2X\n", reg_addr);
        hexDump("Written data: ", reg_data, cnt);
	return SUCCESS;
}

static s8 BMP280_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
        struct i2c_rdwr_ioctl_data packets;
        struct i2c_msg messages[2];

        messages[0].addr  = dev_addr;
        messages[0].flags = 0;
        messages[0].len   = sizeof(reg_addr);
        messages[0].buf   = &reg_addr;

        messages[1].addr  = dev_addr;
        messages[1].flags = I2C_M_RD | I2C_M_NOSTART;
        messages[1].len   = cnt;
        messages[1].buf   = reg_data;

        /* Transfer the i2c packets to the kernel and verify it worked */
        packets.msgs  = messages;
        packets.nmsgs = 2;
        if(ioctl(file, I2C_RDWR, &packets) < 0) {
                perror("Unable to receive data");
                return ERROR;
        }

	printf("register read %2X\n", reg_addr);
	hexDump("Read data: ",reg_data,cnt);
        return SUCCESS;
}

static void BMP280_delay_msec(BMP280_MDELAY_DATA_TYPE ms)
{
        usleep(ms*1000);
}

int main()
{
	printf("Starting test...\n");
	file = open(PATH_STR, O_RDWR);
	if (file < 0) {
		printf("Error opening a bus %s : %s\n", PATH_STR, strerror(errno));
		if (errno == EACCES) {
			printf("Try sudo\n");
		};
		exit(1);
	};
        struct bmp280_t bmp280;
        bmp280.bus_read = BMP280_I2C_bus_read;
        bmp280.bus_write = BMP280_I2C_bus_write;
        bmp280.delay_msec = BMP280_delay_msec;
	for (size_t i = 0; i < (sizeof(addr_array)/sizeof(addr_array[0])); i++) {
		bmp280.chip_id = 0xFF;
                bmp280.dev_addr = addr_array[i];
                s32 com_rslt = bmp280_init(&bmp280);
                printf("  > Testing 0x%2X -> %s\n", bmp280.dev_addr, (com_rslt == 0) ? "Found" : "Not found");
                if (com_rslt != 0) {
                        continue;
		}
		/* CP from template */
		/* The variable used to assign the standby time*/
		u8 v_standby_time_u8 = BMP280_INIT_VALUE;
		/* The variable used to read uncompensated temperature*/
		s32 v_data_uncomp_tem_s32 = BMP280_INIT_VALUE;
		/* The variable used to read uncompensated pressure*/
		s32 v_data_uncomp_pres_s32 = BMP280_INIT_VALUE;
		/* The variable used to read real temperature*/
		s32 v_actual_temp_s32 = BMP280_INIT_VALUE;
		/* The variable used to read real pressure*/
		u32 v_actual_press_u32 = BMP280_INIT_VALUE;
		s32 v_actual_press_data_s32 = BMP280_INIT_VALUE;
		/* result of communication results*/
		com_rslt += bmp280_set_power_mode(BMP280_NORMAL_MODE);
		com_rslt += bmp280_set_work_mode(BMP280_HIGH_RESOLUTION_MODE);
		com_rslt += bmp280_set_standby_durn(BMP280_STANDBY_TIME_1_MS);
		com_rslt += bmp280_read_uncomp_pressure_temperature(&v_actual_press_data_s32, &v_actual_temp_s32);
		printf("   > Temp %i C preassure  %i millibar \n", v_actual_temp_s32, v_actual_press_data_s32);
		com_rslt += bmp280_read_pressure_temperature(&v_actual_press_u32,  &v_actual_temp_s32);
		if (com_rslt != 0) {
                	continue;
              	}
              	printf("   > Temp %i.%.2i C preassure  %i.%.2i millibar \n", v_actual_temp_s32/100,
                                                                          v_actual_temp_s32 % 100 ,
                                                                          v_actual_press_u32/100,
                                                                          v_actual_press_u32 % 100);
		com_rslt += bmp280_set_power_mode(BMP280_SLEEP_MODE);
	}
	
	close(file);
	return 0;
}
