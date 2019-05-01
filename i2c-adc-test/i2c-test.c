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
 *      if DEBUG4realdevice=0,the example is : i2c-test -D /dev/i2c-1  -s  0x10  -r  1  -v
 *      if DEBUG4realdevice=1,the example is : i2c-test -D /dev/i2c-1  -s  0x10  -r  -v
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <errno.h>
#include <stdbool.h>

#include "i2c-dev.h"

#define DEBUG4realdevice    1

#define u8 unsigned char
#define u32 unsigned int

struct realdevice_data {
	u8 reg_addr;
	u8 reg_val;
};


#define I2C_DEV                                  "/dev/i2c-1"
#define SLAVE_I2C_ADDR                               0x10

#define DUMP_BYTES_PER_LINE		(20) /* x4 char per byte = 80 chars */

#define realdevice_REG_MAX         0x08

#define ARRAY_SIZE(a)	(sizeof(a) / sizeof(*a))

#define pr_verbose(fmt, ...) do {\
		if (verbose)\
			printf(fmt, ##__VA_ARGS__);\
	} while (0)

static const char *device_name = "/dev/i2c-1";
static uint16_t       slave_address = SLAVE_I2C_ADDR;
static int                iterations = 1;
static int  read_bc=0;
static int  write_bc=0;
static int  rd_wr_cnt  = 1;
static bool verbose=false;


static int (*test_func)(int) = NULL;

static int test_wr_rd(int fd);
static int realdevice_read_reg(int fd,int chip_addr, u8 regaddr, u8 * regvalue);
static int realdevice_write_reg(int fd,int chip_addr, u8 regaddr, u8 regvalue);

static int read_realdevice_reg(int fd)
{
	u8 i,reg_addr,reg_val;
	int ret=-1;
	for(i=0;i<realdevice_REG_MAX;i++)
	{
		ret=realdevice_read_reg(fd,SLAVE_I2C_ADDR,i,&reg_val);
		printf("%s line%d reg 0x%x,reg_val is 0x%x\n",__func__,__LINE__,i,reg_val);
	}
	return ret;
}

static int write_realdevice_reg(int fd)
{
	u8 i,reg_addr,reg_val;
	int ret=-1;
	struct realdevice_data ax553x_reg_cfg[] ={
			{
				.reg_addr = 0x00,
				.reg_val = 0xff,
			},
			{
				.reg_addr = 0x01,
				.reg_val = 0x01,
			},
			{
				.reg_addr = 0x02,
				.reg_val = 0x1b,
			},
			{
				.reg_addr = 0x03,
				.reg_val = 0x00,
			},
			{
				.reg_addr = 0x04,
				.reg_val = 0x00,
			},
			{
				.reg_addr = 0x05,
				.reg_val = 0x00,
			},
			{
				.reg_addr = 0x06,
				.reg_val = 0x00,
			},
			{
				.reg_addr = 0x07,
				.reg_val = 0x00,
			},
	};

	for(i=0;i<realdevice_REG_MAX;i++)
	{
		ret=realdevice_write_reg(fd,SLAVE_I2C_ADDR,ax553x_reg_cfg[i].reg_addr,ax553x_reg_cfg[i].reg_val);
		printf("%s line%d reg 0x%x,reg_val is 0x%x\n",__func__,__LINE__,ax553x_reg_cfg[i].reg_addr,ax553x_reg_cfg[i].reg_val);
	}
	return ret;
}

static int do_rdwr(int fd, struct i2c_msg *msgs, int nmsgs)
{
	struct i2c_rdwr_ioctl_data msgset = {
		.msgs = msgs,
		.nmsgs = nmsgs,
	};

	if (msgs == NULL || nmsgs <= 0)
		return -1;

	if (ioctl(fd, I2C_RDWR, &msgset) < 0)
		return -1;

	return 0;
}

/**
 * do_basic_read: Read from device a single buffer
 * @return one on success or negative error code
 */
static int do_basic_read(int fd, uint16_t address, uint8_t *buf, uint16_t count)
{
	int ret;

	struct i2c_msg msgs[] = {
		{
			.addr = address,
			.flags = I2C_M_RD,
			.buf = (void *)buf,
			.len = count,
		},
	};
	struct i2c_rdwr_ioctl_data msgset = {
		.msgs = msgs,
		.nmsgs = ARRAY_SIZE(msgs),
	};

	ret = ioctl(fd, I2C_RDWR, &msgset);
	printf("Reading:%d bytes at addr:0x%x ret:%d\n",
			count, slave_address, ret);

	return ret;
}

static int test_scan_bus(int fd)
{
	int i;
	uint8_t data[1];

	printf("Scanning the bus...\n");

	for (i = 0; i < 8; ++i) {
		slave_address += i;
		if (!do_basic_read(fd, slave_address, data, sizeof(data)))
			printf("Found device at address 0x%x (0x%x/0x%x)\n",
				i, i*2, (i*2)+1);
	}

	return 0;
}

static void dump_test_config(void)
{
	const char *type = "NULL";

	/* dump verbose info */
	if (test_func == test_scan_bus)
		type = "Bus-Scan";
	else if (test_func == test_wr_rd)
		type = "Write and Read Test";

	pr_verbose("I2C unit test configuration summary:\n");
	pr_verbose("test type       : %s\n", type);
	pr_verbose("device          : %s\n", device_name);
	pr_verbose("address         : 0x%x\n", slave_address);
       if (test_func == test_wr_rd) {
		pr_verbose("write size      : %d\n", write_bc);
		pr_verbose("read size       : %d\n", read_bc);
		pr_verbose("rd_wr_cnt       : %d\n", rd_wr_cnt);
	};
	pr_verbose("iterations      : %d\n", iterations);
}


static int test_wr_rd(int fd)
{
	int i, j, k;
	int step;
	int n_msgs ;
	int rc = 0;
	/* read_buf includes multiple read buffers */
	uint8_t        *read_buf  = NULL;
	/* single write_buf is used for all writes */
	uint8_t        *write_buf = NULL;
	struct i2c_msg *msgs      = NULL;
	const char      *action = "";

	printf("%s start\n",__func__);

	if (verbose)
		dump_test_config();

	if (read_bc) {
			action = "read";
			step   = 1;
			n_msgs = rd_wr_cnt;
	} else {
		if (write_bc) {
			action = "write";
			step   = 1;
			n_msgs = rd_wr_cnt;
		} else {
			return 0;
		}
	}

	msgs = malloc(sizeof(*msgs) * n_msgs);
	if (!msgs) {
		printf("failed to allocated array of %d i2c_msg structs",
									n_msgs);
		return -ENOMEM;
	}

	/* allocate read buffer */
	if (read_bc) {
		read_buf = malloc(read_bc * rd_wr_cnt);
		if (!read_buf) {
			rc = -ENOMEM;
			printf("failed to allocate %d bytes for read buffer\n",
								read_bc);
			goto err_exit;
		}
	}
	/* allocate write buffer */
	if (write_bc) {
		write_buf = malloc(write_bc);
		if (!write_buf) {
			rc = -ENOMEM;
			printf("failed to allocate %d bytes for write buffer\n",
								write_bc);
			goto err_exit;
		}
	}

	pr_verbose("read_bc:%d write_bc:%d step:%d\n", read_bc, write_bc, step);

	/* populate i2c_msgs with write buffers */
	if (write_bc)
		for (i = 0 ; i < n_msgs; i += step)
			msgs[i] = (struct i2c_msg) {
				.addr  = slave_address,
				.len   = write_bc,
				.buf   = write_buf,
			};

	/* populate i2c_msgs with read buffers */
	if (read_bc) {
		uint8_t *cur_read_buf = read_buf;

		for (i = (write_bc ? 1 : 0); i < n_msgs; i += step,
							cur_read_buf += read_bc)
			msgs[i] = (struct i2c_msg) {
				.addr  = slave_address,
				.flags = I2C_M_RD,
				.len   = read_bc,
				.buf   = cur_read_buf,
			};
	}

	for (i = 0 ; i < n_msgs; ++i)
		pr_verbose("msg[%d] addr:0x%x len:%d is_inp:0x%x\n",
			i, msgs[i].addr, msgs[i].len,
			msgs[i].flags & I2C_M_RD);

	for (j = 0; j < iterations; j++) {
		rc = do_rdwr(fd, msgs, n_msgs);
		if (rc < 0) {
			printf("Could not %s buffers\n", action);
			goto err_exit;
		}

		if (read_bc && verbose) {
			uint8_t *cur_read_buf = read_buf;

			for (k = (write_bc ? 1 : 0); k < n_msgs; k += step,
						cur_read_buf += read_bc) {
				printf("Read buf:");
				for (i = 0; i < read_bc; ++i) {
					if (!(i % DUMP_BYTES_PER_LINE))
						printf("\n");
					printf("0x%x ", cur_read_buf[i]);
				}
				printf("\n");
			}
		}
	}

err_exit:
	free(read_buf);
	free(write_buf);
	free(msgs);

	printf("%s end rc=%d\n",__func__,rc);
	return rc;
}

static void print_usage(const char *prog)
{
	printf("Usage: %s [-Dsbnoiealv]\n", prog);
	puts(
	"  -D --device      device to use (default /dev/i2c-1)\n"
	"  -s --slave_addr  address of the i2c slave device\n"
	"  -i --iterations        number of times to repeat the test\n"
	"  -l --scan        scans the i2c bus for slaves (lists devices)\n"
	"  -v --verbose     verbose\n"
	"\n"
	"*ONLY* use with an i2c-bus *ANALYSER/EMULATOR* device:\n"
	"  -w --write       number of bytes to write for write-then-read\n"
	"                   test (not eeprom test). default is no write.\n"
	"  -r --read        number of bytes to read for write-then-read\n"
	"                   test (not eeprom test). default is no read.\n"
	"  -c --rd_wr_cnt   when --write is set this option sets the\n"
	"                   number of writes. When --read is set, this\n"
	"                   option sets the number of reads. When both\n"
	"                   are set, this option creates a mix array\n"
	"                   such that wr-0,rd-0,wr-1,rd-1,...wr-n,rd-n.\n"
	"                   When none is set, this option has no effect.\n");
	exit(1);
}

static int parse_args(int argc, char **argv)
{
	struct option lopts[] = {
		{ "device",      required_argument, NULL, 'D'},
		{ "slave_addr",  required_argument, NULL, 's'},
		{ "iterations",  required_argument, NULL, 'i'},
		{ "scan",        no_argument,       NULL, 'l'},
		#if DEBUG4realdevice
		{ "write",       no_argument, NULL, 'w'},
		{ "read",        no_argument, NULL, 'r'},
		#else
		{ "write",       required_argument, NULL, 'w'},
		{ "read",        required_argument, NULL, 'r'},
		#endif
		{ "rd_wr_cnt",   required_argument, NULL, 'c'},
		{ "verbose",     no_argument,       NULL, 'v'},
		{ "help",        no_argument,       NULL, 'h'},
		{ NULL,          0,                 NULL,  0},
	};
	int command;
	#if DEBUG4realdevice
	const char *optstr = "D:s:i:lwrc:vhe";
	#else
	const char *optstr = "D:s:i:lw:r:c:vhe";
	#endif

	while ((command = getopt_long(argc, argv, optstr, lopts, NULL)) != -1) {
		printf("command=%c,optarg=0x%x\n",command,optarg);
		switch (command) {
		case 'D':
			device_name = optarg;
			break;
		case 's':
			slave_address = (uint16_t)(strtol(optarg, NULL, 0));
			break;
		case 'i':
			iterations = atoi(optarg);
			break;
		case 'l':
			test_func = test_scan_bus;
			break;
		case 'w':
			printf("w optarg=%s\n",optarg);
			#if DEBUG4realdevice
                        test_func = write_realdevice_reg;
			#else
			write_bc = atoi(optarg);
			if (write_bc > 0) {
				test_func = test_wr_rd;
			} else {
				printf("%s is invalid num of bytes to write",
				       optarg);
				write_bc = 0;
			}
			#endif
			break;
		case 'r':
			printf("r optarg=%s\n",optarg);
			#if DEBUG4realdevice
                        test_func = read_realdevice_reg;
			#else
			read_bc = atoi(optarg);
			if (read_bc > 0) {
				test_func = test_wr_rd;
			} else {
				printf("%s is invalid num of bytes to read",
				       optarg);
				read_bc = 0;
			}
			#endif
			break;
		case 'c':
			rd_wr_cnt = atoi(optarg);
			if (rd_wr_cnt < 0) {
				printf("%s is invalid num of bytes to read",
				       optarg);
				rd_wr_cnt = 1;
			}
			break;
		case 'v':
			verbose = true;
			break;
		case 'h':
		default:
			print_usage(argv[0]);
			break;
		}
	}

	return 0;
}

static int realdevice_i2c_Read(int fd, int chip_addr, char *writebuf,
		    int writelen, char *readbuf, int readlen)
{
	int ret;
	struct i2c_rdwr_ioctl_data ioctl_data;
	printf("%s: line%d writelen=%d\n",__func__,__LINE__,writelen);
	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
			 .addr = chip_addr,
			 .flags = 0,   //   write
			 .len = writelen,
			 .buf = writebuf,
			 },
			{
			 .addr = chip_addr,
			 .flags = I2C_M_RD,    //    0
			 .len = readlen,
			 .buf = readbuf,
			 },
		};

		ioctl_data.nmsgs = 1;
		ioctl_data.msgs  = &msgs[0];
	//	ret = i2c_transfer(client->adapter, msgs, 2);
		ret = ioctl(fd,I2C_RDWR,&ioctl_data);
		if (ret < 0)
			printf("f%s: i2c read error.\n",__func__);

		ioctl_data.nmsgs = 1;
		ioctl_data.msgs  = &msgs[1];
	//	ret = i2c_transfer(client->adapter, msgs, 2);
		ret = ioctl(fd,I2C_RDWR,&ioctl_data);
		if (ret < 0)
			printf("f%s: i2c read error.\n",__func__);
	} else {
		struct i2c_msg msgs[] = {
			{
			 .addr = chip_addr,
			 .flags = I2C_M_RD,   // 1
			 .len = readlen,
			 .buf = readbuf,
			 },
		};

		ioctl_data.nmsgs = 1;
		ioctl_data.msgs  = msgs;

	//	ret = i2c_transfer(client->adapter, msgs, 1);
		ret = ioctl(fd,I2C_RDWR,&ioctl_data);
		if (ret < 0)
			printf("f%s: i2c read error.\n",__func__);
		}

	printf("%s: end ret=%d\n",__func__,ret);
	return ret;
}


static int realdevice_i2c_Write(int fd, int chip_addr, char *writebuf, int writelen)
{
	int ret;

	struct i2c_rdwr_ioctl_data ioctl_data;
	struct i2c_msg msg[] = {
		{
		 .addr = chip_addr,
		 .flags = 0,  // write
		 .len = writelen,
		 .buf = writebuf,
		 },
	};

	ioctl_data.nmsgs = 1;
	ioctl_data.msgs  = msg;

//	ret = i2c_transfer(client->adapter, msgs, 1);
	ret = ioctl(fd,I2C_RDWR,&ioctl_data);
	if (ret < 0)
		printf("f%s: i2c write error.\n",__func__);

	printf("%s: end ret=%d\n",__func__,ret);
	return ret;
}

static int realdevice_read_reg(int fd,int chip_addr, u8 regaddr, u8 * regvalue)
{
	return realdevice_i2c_Read(fd, chip_addr, &regaddr, 1, regvalue, 1);
}

static int realdevice_write_reg(int fd,int chip_addr, u8 regaddr, u8 regvalue)
{
	unsigned char buf[2] = {0};
	buf[0] = regaddr;
	buf[1] = regvalue;

	return realdevice_i2c_Write(fd,chip_addr, buf, sizeof(buf));
}

int main(int argc, char *argv[])
{
	int fd=-1;
	int rc=-1;
	int count=0;
	unsigned char reg_data = 0;
	unsigned char reg_addr = 0;
	unsigned char operate = 0;

	if (parse_args(argc, argv))
		return -1;

	printf("open device_name is %s\n",device_name);

	fd = open(device_name,O_RDWR);
	if(fd == -1)
	{
		printf("open error\n");
		return -1;
	}

	printf("line%d test_func is %pF\n",__LINE__,test_func);
	rc = (*test_func)(fd);

	close(fd);

	return 0;   // sucessful


#if 0
	if(ioctl(fd,I2C_SLAVE,SLAVE_I2C_ADDR) < 0)
	{
		printf("set addr error\n");
		close(fd);
		return -1;
	}

	if(ioctl(fd,I2C_SLAVE_FORCE,SLAVE_I2C_ADDR) < 0)
	{
		printf("set addr error\n");
		close(fd);
		return -1;
	}
#endif

#if 0
        reg_addr=atoi(argv[1]);

	//reg_addr = *argv[1];
	//count = scanf("%c %x\n",&operate,&reg_addr);
	printf("operate=%c,reg_addr is 0x%x\n",operate,reg_addr);

	switch(operate)
	{
		case 'r':
		case 'R':
			realdevice_read_reg(fd, SLAVE_I2C_ADDR, reg_addr, &reg_data);
			printf("read reg_addr=0x%x,reg_data = 0x%x\n",reg_addr,reg_data);
			break;
		case 'w':
		case 'W':
			realdevice_write_reg(fd, SLAVE_I2C_ADDR, reg_addr, reg_data);
			printf("write reg_addr=0x%x,reg_data = 0x%x\n",reg_addr,reg_data);
			break;
		default:
			printf("error please input parameters\n");
	}
#endif
}
