/*
 * SPI testing utility (using spidev driver)
 *
 * Copyright (c) 2007  MontaVista Software, Inc.
 * Copyright (c) 2007  Anton Vorontsov <avorontsov@ru.mvista.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
 *
 * Cross-compile with cross-gcc -I/path/to/cross-kernel/include
 */
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <fcntl.h>
#include <time.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <sys/time.h>
#include <unistd.h>
#include <sched.h>
#include <linux/ioctl.h>
#include <sys/stat.h>
#include <time.h>
#include <sys/time.h>
#include <sched.h>
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define BUF_SIZE					2400
#define X_CHANNEL					0
#define Y_CHANNEL					1
#define Z_CHANNEL					2

static uint32_t speed = 6000000;
//static const char *device = "/dev/spidev1.0";
static const char *device = "/dev/spidev2.0";
int ndi_data[600]= {0,};
unsigned char default_rx[BUF_SIZE] = {0, };
int main(int argc, char *argv[])
{
	int ret = 0;
	int fd;
	int read_count = 10;
	int err_counter = 0;
	int current_frame_x = 0;
	int previous_frame_x = 0;
	int current_frame_y = 0;
	int previous_frame_y = 0;
	int current_frame_z = 0;
	int previous_frame_z = 0;
	int rc,old_scheduler_policy;
	struct sched_param my_params;
	old_scheduler_policy=sched_getscheduler(0);

	my_params.sched_priority=sched_get_priority_max(SCHED_RR);
	printf("SCHED_OTHER = %d SCHED_FIFO =%d SCHED_RR=%d \n",SCHED_OTHER,SCHED_FIFO,SCHED_RR);
	printf("the current scheduler = %d \n",old_scheduler_policy);
	rc=sched_setscheduler(0,SCHED_RR,&my_params);
	if(rc<0)
	{
   		perror("sched_setscheduler to SCHED_RR error");
   		exit(0);
	}
	old_scheduler_policy=sched_getscheduler(0);
	printf("the current scheduler = %d \n",old_scheduler_policy);

	rc=sched_setscheduler(0,SCHED_FIFO,&my_params);
	if(rc<0)
	{
   		perror("sched_setscheduler to SCHED_FIFO error");
   		exit(0);
	}
	old_scheduler_policy=sched_getscheduler(0);

	printf("the current scheduler = %d \n",old_scheduler_policy);


	fd = open(device, O_RDWR);
	if (fd < 0)
		printf("can't open device");

        ret=read(fd,default_rx,BUF_SIZE);
#if 0
	printf("GPIO HIGH\n");
        ioctl(fd, SPI_IOC_SET_RFIC_IO2_STATUS, 1);
        sleep(5);
        printf("GPIO LOW\n");
        ioctl(fd, SPI_IOC_SET_RFIC_IO2_STATUS, 0);
        sleep(5);
        printf("GPIO HIGH\n");
        ioctl(fd, SPI_IOC_SET_RFIC_IO2_STATUS, 1);
        sleep(5);
        printf("GPIO LOW\n");
        ioctl(fd, SPI_IOC_SET_RFIC_IO2_STATUS, 0);
        sleep(5);
#endif
	close(fd);

	return 0;
}
