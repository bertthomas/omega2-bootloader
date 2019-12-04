/*
 * (C) Copyright 2003
 * Wolfgang Denk, DENX Software Engineering, wd@denx.de.
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 *
 * Modified for Ultimaker S1
 */

#include <common.h>
#include <command.h>
#include <malloc.h>
#include <devices.h>
#include <version.h>
#include <net.h>
#include <environment.h>
#include <asm/mipsregs.h>
#include <rt_mmap.h>
#include <spi_api.h>
#include <nand_api.h>

DECLARE_GLOBAL_DATA_PTR;
#undef DEBUG

#define SDRAM_CFG1_REG RALINK_SYSCTL_BASE + 0x0304

int modifies= 0;

#ifdef DEBUG
   #define DATE      "05/25/2006"
   #define VERSION   "v0.00e04"
#endif
#define	TOTAL_MALLOC_LEN	(CFG_MALLOC_LEN + CFG_ENV_SIZE)
#define ARGV_LEN  128

extern int timer_init(void);

extern void  rt2880_eth_halt(struct eth_device* dev);

extern void setup_internal_gsw(void);
extern void setup_external_gsw(void);

extern int incaip_set_cpuclk(void);
extern int do_bootm (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[]);
extern int do_tftpb (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[]);
extern int do_mem_cp ( cmd_tbl_t *cmdtp, int flag, int argc, char *argv[]);
extern int flash_sect_protect (int p, ulong addr_first, ulong addr_last);
int flash_sect_erase (ulong addr_first, ulong addr_last);
int get_addr_boundary (ulong *addr);
extern int do_reset(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[]);
extern void input_value(u8 *str);
extern void rt305x_esw_init(void);
extern void LANWANPartition(void);

extern struct eth_device* 	rt2880_pdev;

extern ulong uboot_end_data;
extern ulong uboot_end;

extern int usb_stor_curr_dev;

ulong monitor_flash_len;

const char version_string[] =
	U_BOOT_VERSION" (" __DATE__ " - " __TIME__ ")";

extern ulong load_addr; /* Default Load Address */


unsigned long mips_cpu_feq;
unsigned long mips_bus_feq;


/*
 * Begin and End of memory area for malloc(), and current "brk"
 */
static ulong mem_malloc_start;
static ulong mem_malloc_end;
static ulong mem_malloc_brk;

static char  file_name_space[ARGV_LEN];

#define read_32bit_cp0_register_with_select1(source)            \
({ int __res;                                                   \
        __asm__ __volatile__(                                   \
        ".set\tpush\n\t"                                        \
        ".set\treorder\n\t"                                     \
        "mfc0\t%0,"STR(source)",1\n\t"                          \
        ".set\tpop"                                             \
        : "=r" (__res));                                        \
        __res;})
//added by mango
void gpio_init(void);
void led_on(void);
void led_off(void);

// Added by zh@onion.io
int detect_rst(void); // rename wps button to rst
void gpio_test(int vtest);
void set_gpio_led(int vreg,int vgpio) ;//jeff

static void Init_System_Mode(void)
{
	u32 reg;
	u8	clk_sel;

	reg = RALINK_REG(RT2880_SYSCFG_REG);

	/*
	 * CPU_CLK_SEL (bit 21:20)
	 */
	reg = RALINK_REG(RALINK_CLKCFG0_REG);
	if (reg & (0x1<<1)) {
		mips_cpu_feq = (480*1000*1000)/CPU_FRAC_DIV;
	}else if (reg & 0x1) {
		mips_cpu_feq = ((RALINK_REG(RALINK_SYSCTL_BASE+0x10)>>6)&0x1) ? (40*1000*1000)/CPU_FRAC_DIV \
					   : (25*1000*1000)/CPU_FRAC_DIV;
	}else {
		mips_cpu_feq = (575*1000*1000)/CPU_FRAC_DIV;
	}
	mips_bus_feq = mips_cpu_feq/3;

	/* in general, the spec define 8192 refresh cycles/64ms
	 * 64ms/8192 = 7.8us
	 * 7.8us * 106.7Mhz(SDRAM clock) = 832
	 * the value of refresh cycle shall smaller than 832.
	 * so we config it at 0x300 (suggested by ASIC)
	 */

}


/*
 * The Malloc area is immediately below the monitor copy in DRAM
 */
static void mem_malloc_init (void)
{

	ulong dest_addr = CFG_MONITOR_BASE + gd->reloc_off;

	mem_malloc_end = dest_addr;
	mem_malloc_start = dest_addr - TOTAL_MALLOC_LEN;
	mem_malloc_brk = mem_malloc_start;

	memset ((void *) mem_malloc_start,
		0,
		mem_malloc_end - mem_malloc_start);
}

void *sbrk (ptrdiff_t increment)
{
	ulong old = mem_malloc_brk;
	ulong new = old + increment;

	if ((new < mem_malloc_start) || (new > mem_malloc_end)) {
		return (NULL);
	}
	mem_malloc_brk = new;
	return ((void *) old);
}

static int init_func_ram (void)
{

	int board_type = 0;	/* use dummy arg */
	puts ("DRAM:  ");

/*init dram config*/

	if ((gd->ram_size = initdram (board_type)) > 0) {
		print_size (gd->ram_size, "\n");
		return (0);
	}
	puts ("*** failed ***\n");

	return (1);
}

static int display_banner(void)
{
    printf ("\n\n   ____       _             ____\n"
                    "  / __ \\___  (_)__  ___    / __ \\__ _  ___ ___ ____ _\n"
                    " / /_/ / _ \\/ / _ \\/ _ \\  / /_/ /  ' \\/ -_) _ `/ _ `/\n"
                    " \\____/_//_/_/\\___/_//_/  \\____/_/_/_/\\__/\\_, /\\_,_/\n"
                    " W H A T  W I L L  Y O U  I N V E N T ? /___/\"\n\n");

	return (0);
}

static int init_baudrate (void)
{
	gd->baudrate = CONFIG_BAUDRATE;
	return (0);
}


/*
 * Breath some life into the board...
 *
 * The first part of initialization is running from Flash memory;
 * its main purpose is to initialize the RAM so that we
 * can relocate the monitor code to RAM.
 */

/*
 * All attempts to come up with a "common" initialization sequence
 * that works for all boards and architectures failed: some of the
 * requirements are just _too_ different. To get rid of the resulting
 * mess of board dependend #ifdef'ed code we now make the whole
 * initialization sequence configurable to the user.
 *
 * The requirements for any new initalization function is simple: it
 * receives a pointer to the "global data" structure as it's only
 * argument, and returns an integer return code, where 0 means
 * "continue" and != 0 means "fatal error, hang the system".
 */

//
void board_init_f(ulong bootflag)
{
	gd_t gd_data, *id;
	bd_t *bd;
	//init_fnc_t **init_fnc_ptr;
	ulong addr, addr_sp, len = (ulong)&uboot_end - CFG_MONITOR_BASE;
	ulong *s;
	u32 value;
	u32 fdiv = 0, step = 0, frac = 0, i;

	value = le32_to_cpu(*(volatile u_long *)(RALINK_SPI_BASE + 0x3c));
	value &= ~(0xFFF);
	value |= 8;
	*(volatile u_long *)(RALINK_SPI_BASE + 0x3c) = cpu_to_le32(value);

	value = RALINK_REG(RALINK_DYN_CFG0_REG);
	fdiv = (unsigned long)((value>>8)&0x0F);
	if ((CPU_FRAC_DIV < 1) || (CPU_FRAC_DIV > 10))
	frac = (unsigned long)(value&0x0F);
	else
		frac = CPU_FRAC_DIV;
	i = 0;

	while(frac < fdiv) {
		value = RALINK_REG(RALINK_DYN_CFG0_REG);
		fdiv = ((value>>8)&0x0F);
		fdiv--;
		value &= ~(0x0F<<8);
		value |= (fdiv<<8);
		RALINK_REG(RALINK_DYN_CFG0_REG) = value;
		udelay(500);
		i++;
		value = RALINK_REG(RALINK_DYN_CFG0_REG);
		fdiv = ((value>>8)&0x0F);
		//frac = (unsigned long)(value&0x0F);
	}

	/* Pointer is writable since we allocated a register for it.
	 */
	gd = &gd_data;
	/* compiler optimization barrier needed for GCC >= 3.4 */
	__asm__ __volatile__("": : :"memory");


	memset ((void *)gd, 0, sizeof (gd_t));

	timer_init();
	env_init();		/* initialize environment */
	init_baudrate();		/* initialze baudrate settings */
	serial_init();		/* serial communications setup */
	console_init_f();
	display_banner();		/* say that we are here */
	checkboard();

	init_func_ram();

	/* reset Frame engine */
	value = le32_to_cpu(*(volatile u_long *)(RALINK_SYSCTL_BASE + 0x0034));
	udelay(100);
	//2880 -> 3052 reset Frame Engine from 18 to 21
	value |= (1 << 21);

	*(volatile u_long *)(RALINK_SYSCTL_BASE + 0x0034) = cpu_to_le32(value);
	value &= ~(1 << 21);
	*(volatile u_long *)(RALINK_SYSCTL_BASE + 0x0034) = cpu_to_le32(value);
	udelay(200);

	/*
	 * Now that we have DRAM mapped and working, we can
	 * relocate the code and continue running from DRAM.
	 */
	addr = CFG_SDRAM_BASE + gd->ram_size;

	/* We can reserve some RAM "on top" here.
	 */

	/* round down to next 4 kB limit.
	 */
	addr &= ~(4096 - 1);

	/* Reserve memory for U-Boot code, data & bss
	 * round down to next 16 kB limit
	 */
	addr -= len;
	addr &= ~(16 * 1024 - 1);
	 /* Reserve memory for malloc() arena.
	 */
	addr_sp = addr - TOTAL_MALLOC_LEN;
	/*
	 * (permanently) allocate a Board Info struct
	 * and a permanent copy of the "global" data
	 */
	addr_sp -= sizeof(bd_t);
	bd = (bd_t *)addr_sp;
	gd->bd = bd;
	addr_sp -= sizeof(gd_t);
	id = (gd_t *)addr_sp;
 	/* Reserve memory for boot params.
	 */
	addr_sp -= CFG_BOOTPARAMS_LEN;
	bd->bi_boot_params = addr_sp;
	/*
	 * Finally, we set up a new (bigger) stack.
	 *
	 * Leave some safety gap for SP, force alignment on 16 byte boundary
	 * Clear initial stack frame
	 */
	addr_sp -= 16;
	addr_sp &= ~0xF;
	s = (ulong *)addr_sp;
	*s-- = 0;
	*s-- = 0;
	addr_sp = (ulong)s;

	/*
	 * Save local variables to board info struct
	 */
	bd->bi_memstart	= CFG_SDRAM_BASE;	/* start of  DRAM memory */
	bd->bi_memsize	= gd->ram_size;		/* size  of  DRAM memory in bytes */
	bd->bi_baudrate	= gd->baudrate;		/* Console Baudrate */

	memcpy (id, (void *)gd, sizeof (gd_t));

	/* On the purple board we copy the code in a special way
	 * in order to solve flash problems
	 */

	debug ("relocate_code Pointer at: %08lx\n", addr);
	relocate_code (addr_sp, id, addr);

	/* NOTREACHED - relocate_code() does not return */
}

#define TEMP_MAINTENANCE

#define SEL_BOOT_FLASH                         -1
#define SEL_WEB_MODE                            0
#define SEL_ENTER_CLI                           1
#define SEL_LOAD_LINUX_USB                      2
#define SEL_LOAD_LINUX_SDRAM                    3
#define SEL_LOAD_LINUX_WRITE_FLASH_BY_SERIAL    4
#define SEL_LOAD_LINUX_WRITE_FLASH              5
#define SEL_LOAD_BOOT_USB                       6
#define SEL_LOAD_BOOT_SDRAM                     7
#define SEL_LOAD_BOOT_WRITE_FLASH_BY_SERIAL     8
#define SEL_LOAD_BOOT_WRITE_FLASH               9
//#define SEL_TEST_LEDS                         x

void OperationSelect(void)
{
	printf("\nPlease select option: \n");

    //printf("   %d: Enter led testing mode.\n", SEL_TEST_LEDS);
    // zh@onion.io
    // update boot menu format
    printf("   [ Enter ]: Boot Omega2.\n");

    // zh@onion.io
#ifdef ONION_WEB_FLASH
    printf("   [ %d ]: Start Web recovery mode.\n", SEL_WEB_MODE);
#endif //ONION_WEB_FLASH

#ifdef RALINK_CMDLINE
    printf("   [ %d ]: Start command line mode.\n", SEL_ENTER_CLI);
#endif // RALINK_CMDLINE //

#ifdef ONION_USB_FLASH
    printf("   [ %d ]: Flash firmware from USB storage. \n", SEL_LOAD_LINUX_USB);
#endif //ONION_USB_FLASH

#ifdef ONION_TFTP_FLASH_SDRAM
    printf("   [ %d ]: Flash firmware to SDRAM via TFTP. \n", SEL_LOAD_LINUX_SDRAM);
#endif // ONION_TFTP_FLASH

#ifdef RALINK_UPGRADE_BY_SERIAL
    printf("   [ %d ]: Flash firmware via Serial. \n", SEL_LOAD_LINUX_WRITE_FLASH_BY_SERIAL);
#endif // RALINK_UPGRADE_BY_SERIAL //

#ifdef ONION_TFTP_FLASH
    printf("   [ %d ]: Flash firmware via TFTP. \n", SEL_LOAD_LINUX_WRITE_FLASH);
#endif // ONION_TFTP_FLASH

// #ifdef ONION_USB_FLASH
//     printf("   [ %d ]: Flash boot loader code from USB. \n", SEL_LOAD_BOOT_USB);
// #endif

#ifdef ONION_TFTP_FLASH_SDRAM
    printf("   [ %d ]: Flash boot loader code then write to SDRAM via TFTP. \n", SEL_LOAD_BOOT_SDRAM);
#endif

#ifdef RALINK_UPGRADE_BY_SERIAL
	printf("   [ %d ]: Flash boot loader via Serial. \n", SEL_LOAD_BOOT_WRITE_FLASH_BY_SERIAL);
#endif // RALINK_UPGRADE_BY_SERIAL //

#ifdef ONION_TFTP_FLASH
	printf("   [ %d ]: Flash boot loader via TFTP. \n", SEL_LOAD_BOOT_WRITE_FLASH);
#endif // ONION_TFTP_FLASH
}

int tftp_config(int type, char *argv[])
{
	char *s;
	char default_file[ARGV_LEN], file[ARGV_LEN], devip[ARGV_LEN], srvip[ARGV_LEN], default_ip[ARGV_LEN];

	printf(" Please Input new ones /or Ctrl-C to discard\n");

	memset(default_file, 0, ARGV_LEN);
	memset(file, 0, ARGV_LEN);
	memset(devip, 0, ARGV_LEN);
	memset(srvip, 0, ARGV_LEN);
	memset(default_ip, 0, ARGV_LEN);

	printf("\tInput device IP ");
	s = getenv("ipaddr");
	memcpy(devip, s, strlen(s));
	memcpy(default_ip, s, strlen(s));

	printf("(%s) ", devip);
	input_value(devip);
	setenv("ipaddr", devip);
	if (strcmp(default_ip, devip) != 0)
		modifies++;

	printf("\tInput server IP ");
	s = getenv("serverip");
	memcpy(srvip, s, strlen(s));
	memset(default_ip, 0, ARGV_LEN);
	memcpy(default_ip, s, strlen(s));

	printf("(%s) ", srvip);
	input_value(srvip);
	setenv("serverip", srvip);

	if (strcmp(default_ip, srvip) != 0)
		modifies++;

	if(type == SEL_LOAD_BOOT_SDRAM
			|| type == SEL_LOAD_BOOT_WRITE_FLASH
			) {
		if(type == SEL_LOAD_BOOT_SDRAM)
		argv[1] = "0x80200000";
		else
		argv[1] = "0x80100000";
		printf("\tInput Uboot filename ");
		strncpy(argv[2], "uboot.bin", ARGV_LEN);
	}
	else if (type == SEL_LOAD_LINUX_WRITE_FLASH) {
		argv[1] = "0x80100000";
		printf("\tInput Linux Kernel filename ");
		//argv[2] = "uImage"; winfred: use strncpy instead to prevent the buffer overflow at copy_filename later
		strncpy(argv[2], "uImage", ARGV_LEN);
	}
	else if (type == SEL_LOAD_LINUX_SDRAM ) {
		/* bruce to support ramdisk */
		argv[1] = "0x80A00000";
		printf("\tInput Linux Kernel filename ");
		//argv[2] = "uImage";
		strncpy(argv[2], "uImage", ARGV_LEN);
	}

	s = getenv("bootfile");
	if (s != NULL) {
		memcpy(file, s, strlen(s));
		memcpy(default_file, s, strlen(s));
	}
	printf("(%s) ", file);
	input_value(file);
	if (file == NULL)
		return 1;
	copy_filename (argv[2], file, sizeof(file));
	setenv("bootfile", file);
	if (strcmp(default_file, file) != 0)
		modifies++;

	return 0;
}

void trigger_hw_reset(void)
{
}



/************************************************************************
 *
 * This is the next part if the initialization sequence: we are now
 * running from RAM and have a "normal" C environment, i. e. global
 * data can be written, BSS has been cleared, the stack size in not
 * that critical any more, etc.
 *
 ************************************************************************
 */

gd_t gd_data;

void board_init_r (gd_t *id, ulong dest_addr)
{
	cmd_tbl_t *cmdtp;
	ulong size;
	extern void malloc_bin_reloc (void);
	extern char * env_name_spec;
	char *s, *e;
	bd_t *bd;
	int i;
	int timer1= CONFIG_BOOTDELAY;
	unsigned char BootType='3', confirm=0;
	int my_tmp;
	char addr_str[11];

	u32 config1,lsize,icache_linesz,icache_sets,icache_ways,icache_size;
	u32 dcache_linesz,dcache_sets,dcache_ways,dcache_size;

	memcpy((void *)(CFG_SDRAM_BASE + DRAM_SIZE*0x100000 - 0x10000), (void *)gd, sizeof(gd_t));
	gd = (gd_t *)(CFG_SDRAM_BASE + DRAM_SIZE*0x100000- 0x10000);//&gd_data;

	gd->flags |= GD_FLG_RELOC;	/* tell others: relocation done */

	Init_System_Mode(); /*  Get CPU rate */

  // lazar@onion.io: this might be changing ephy p1-p4 to analog momentarily
	RALINK_REG(RALINK_SYSCTL_BASE+0x3C)|= (1<<8);  // AGPIO_CFG register
	RALINK_REG(RALINK_SYSCTL_BASE+0x64)&= ~((0x3<<16)|(0x3));

	void config_usb_ehciohci(void);
	config_usb_ehciohci();
	void disable_pcie();
	disable_pcie();

	u32 reg = RALINK_REG(RT2880_RSTSTAT_REG);
	if(reg & RT2880_WDRST ){
		printf("***********************\n");
		printf("Watchdog Reset Occurred\n");
		printf("***********************\n");
		RALINK_REG(RT2880_RSTSTAT_REG)|=RT2880_WDRST;
		RALINK_REG(RT2880_RSTSTAT_REG)&=~RT2880_WDRST;
		trigger_hw_reset();
	}else if(reg & RT2880_SWSYSRST){
		printf("******************************\n");
		printf("Software System Reset Occurred\n");
		printf("******************************\n");
		RALINK_REG(RT2880_RSTSTAT_REG)|=RT2880_SWSYSRST;
		RALINK_REG(RT2880_RSTSTAT_REG)&=~RT2880_SWSYSRST;
		trigger_hw_reset();
	}else if (reg & RT2880_SWCPURST){
		printf("***************************\n");
		printf("Software CPU Reset Occurred\n");
		printf("***************************\n");
		RALINK_REG(RT2880_RSTSTAT_REG)|=RT2880_SWCPURST;
		RALINK_REG(RT2880_RSTSTAT_REG)&=~RT2880_SWCPURST;
		trigger_hw_reset();
	}

#ifdef DEBUG
	debug ("Now running in RAM - U-Boot at: %08lx\n", dest_addr);
#endif
	gd->reloc_off = dest_addr - CFG_MONITOR_BASE;

	monitor_flash_len = (ulong)&uboot_end_data - dest_addr;
#ifdef DEBUG
	debug("\n monitor_flash_len =%d \n",monitor_flash_len);
#endif
	/*
	 * We have to relocate the command table manually
	 */
	for (cmdtp = &__u_boot_cmd_start; cmdtp !=  &__u_boot_cmd_end; cmdtp++) {
		ulong addr;

		addr = (ulong) (cmdtp->cmd) + gd->reloc_off;
#ifdef DEBUG
		printf ("Command \"%s\": 0x%08lx => 0x%08lx\n",
				cmdtp->name, (ulong) (cmdtp->cmd), addr);
#endif
		cmdtp->cmd =
			(int (*)(struct cmd_tbl_s *, int, int, char *[]))addr;

		addr = (ulong)(cmdtp->name) + gd->reloc_off;
		cmdtp->name = (char *)addr;

		if (cmdtp->usage) {
			addr = (ulong)(cmdtp->usage) + gd->reloc_off;
			cmdtp->usage = (char *)addr;
		}
		if (cmdtp->help) {
			addr = (ulong)(cmdtp->help) + gd->reloc_off;
			cmdtp->help = (char *)addr;
		}

	}
	/* there are some other pointer constants we must deal with */
	env_name_spec += gd->reloc_off;

	bd = gd->bd;
	if ((size = raspi_init()) == (ulong)-1) {
		printf("ra_spi_init fail\n");
		while(1);
	}
	bd->bi_flashstart = 0;
	bd->bi_flashsize = size;
	bd->bi_flashoffset = 0;

	/* Enable ePA/eLNA share pin */
	{
		char ee35;
		raspi_read((char *)&ee35, CFG_FACTORY_ADDR-CFG_FLASH_BASE+0x35, 1);
		if (ee35 & 0x2)
		{
			RALINK_REG(RALINK_SYSCTL_BASE+0x60)|= ((0x3<<24)|(0x3 << 6));
		}
	}

	// reset MIPS now also reset Andes
	RALINK_REG(RALINK_SYSCTL_BASE+0x38)|= 0x200;

	/* initialize malloc() area */
	mem_malloc_init();
	malloc_bin_reloc();

	spi_env_init();


	/* relocate environment function pointers etc. */
	env_relocate();

	/* board MAC address */
	s = getenv ("ethaddr");
	for (i = 0; i < 6; ++i) {
		bd->bi_enetaddr[i] = s ? simple_strtoul (s, &e, 16) : 0;
		if (s)
			s = (*e) ? e + 1 : e;
	}

	/* IP Address */
	bd->bi_ip_addr = getenv_IPaddr("ipaddr");


	/** leave this here (after malloc(), environment and PCI are working) **/
	/* Initialize devices */
	devices_init ();

	jumptable_init ();

	/* Initialize the console (after the relocation and devices init) */
	console_init_r ();

	/* Initialize from environment */
	if ((s = getenv ("loadaddr")) != NULL) {
		load_addr = simple_strtoul (s, NULL, 16);
	}
	if ((s = getenv ("bootfile")) != NULL) {
		copy_filename (BootFile, s, sizeof (BootFile));
	}

	/* RT2880 Boot Loader Menu */
	SHOW_VER_STR();




	config1 = read_32bit_cp0_register_with_select1(CP0_CONFIG);

	if ((lsize = ((config1 >> 19) & 7)))
		icache_linesz = 2 << lsize;
	else
		icache_linesz = lsize;
	icache_sets = 64 << ((config1 >> 22) & 7);
	icache_ways = 1 + ((config1 >> 16) & 7);

	icache_size = icache_sets *
		icache_ways *
		icache_linesz;

	printf("icache: sets:%d, ways:%d, linesz:%d ,total:%d\n",
			icache_sets, icache_ways, icache_linesz, icache_size);

	/*
	 * Now probe the MIPS32 / MIPS64 data cache.
	 */

	if ((lsize = ((config1 >> 10) & 7)))
		dcache_linesz = 2 << lsize;
	else
		dcache_linesz = lsize;
	dcache_sets = 64 << ((config1 >> 13) & 7);
	dcache_ways = 1 + ((config1 >> 7) & 7);

	dcache_size = dcache_sets *
		dcache_ways *
		dcache_linesz;

	printf("dcache: sets:%d, ways:%d, linesz:%d ,total:%d \n",
			dcache_sets, dcache_ways, dcache_linesz, dcache_size);


	debug("CPU freq = %d MHZ\n",mips_cpu_feq/1000/1000);

	debug("Estimated memory size = %d Mbytes\n",gd->ram_size /1024/1024 );

	rt305x_esw_init();   // lazar@onion.io: ethernet init
	LANWANPartition();

/*config bootdelay via environment parameter: bootdelay */
	{
	    char * s;
	    s = getenv ("bootdelay");
	    timer1 = s ? (int)simple_strtol(s, NULL, 10) : CONFIG_BOOTDELAY;
	}
	gpio_init(); // lazar@onion.io: gpio init

    // zh@onion.io
    // made message shorter
    printf("\n");
    printf("\n");
    printf("**************************************\n");
    printf("* Hold Reset button for more options *\n");
    printf("**************************************\n");
    printf("\n");
    printf("\n");

/* BT20190402 HACK configure GPIO15 as digital pad */
   RALINK_REG(RT2880_AGPIOCFG_REG) |= 0x1e0000;

/* BT20190408 HACK configure GPIO19 and GPIO17 as output */
   RALINK_REG(RT2880_REG_PIODIR) |= 0xA0000;

   /* set GPIO19, reset GPIO17 */
   RALINK_REG(RT2880_REG_PIODATA) |= 0x80000;
   RALINK_REG(RT2880_REG_PIODATA) &= ~0x20000;

#define GPIO1_MODE 0x10000060
   RALINK_REG(GPIO1_MODE) |= 0x40000000;

    // zh@onion.io
    // enter boot menu only when reset button is pressed
    if (detect_rst())
    {

	/* changed this from original code to reflect real value */
        printf("You have %d seconds left to select a menu option...\n\n", timer1);

        OperationSelect();

restart:
        //default
        BootType = '2';

        // zh@onion.io
        // wait for user input
        while (timer1 > 0)
        {
            --timer1;
            /* delay 16 * 60ms = ~1s */
            for (i = 0; i < 16; ++i)
            {

                led_on();

                if ((my_tmp = tstc()) != 0)
                {    /* we got a key press	*/
                    BootType = getc();
                    timer1 = 0;

                    printf("\n\rOption [%c][%d] selected.\n", BootType,BootType);
                    break;
                }

                udelay(30000);
                led_off();
                udelay(30000);

            }
        }

        char *argv[5];
        int argc = 3;

        switch (BootType)
        {
            // zh@onion.io
            // added ethernet bootsafe as option 0
// #ifdef ONION_WEB_FLASH // failsafe
            case '0':
                eth_initialize(gd->bd);
                NetLoopHttpd();
                break;
// #endif //ONION_WEB_FLASH


            // zh@onion.io
            // enable gpio test option
            case 't':
                //gpio_test(0);
                //break;

            // enable Omega2s gpio test option
            case 's':
                gpio_test(1);
                break;

            case '1':
                printf("   \n%d: System Enter Boot Command Line Interface.\n", SEL_ENTER_CLI);
                printf ("\n%s\n", version_string);
                /* main_loop() can return to retry autoboot, if so just run it again. */
                for (;;) {
                    main_loop ();
                }
                break;

                        case '2':
                            printf("System Load Linux then write to Flash via USB Storage. \n");
                            printf("Looking for a USB Storage. \n");
                            printf("If suitable image is found on USB Storage writing to Flash will be attempted. \n");
                            printf("U-Boot will look for a FAT file system. \n");

                            argc = 2;
                            argv[1] = "start";

                            do_usb(cmdtp, 0, argc, argv);

                            if( usb_stor_curr_dev < 0){
                                printf("No USB Storage found. No F/W upgrade from USB Storage will be attempted.\n");
                                break;
                            }

                            argc= 5;
                            argv[1] = "usb";
                            argv[2] = "0";

                            sprintf(addr_str, "0x%X", CFG_LOAD_ADDR);

                            argv[3] = &addr_str[0];


                            argv[4] = "omega2.bin";
                            setenv("autostart", "no");

                            printf("\n");
                            printf("\n");
                            printf("***************************************\n");
                            printf("* [!] This will take several minutes  *\n");
                            printf("* please do not power off your Omega2 *\n");
                            printf("***************************************\n");
                            printf("\n");
                            printf("\n");

                            if(do_fat_fsload(cmdtp, 0, argc, argv)){
                                printf("Upgrade F/W from USB storage failed.\n");
                                break;
                            }

                            NetBootFileXferSize=simple_strtoul(getenv("filesize"), NULL, 16);
                            raspi_erase_write((char *)CFG_LOAD_ADDR, CFG_KERN_ADDR-CFG_FLASH_BASE, NetBootFileXferSize);

                            //reset
                            do_reset(cmdtp, 0, argc, argv);
                            break;

            default:
                timer1 = CONFIG_BOOTDELAY;
                goto restart;

                printf("\nBoot Linux from Flash.\n");

                char *argv_normal[2];
                sprintf(addr_str, "0x%X", CFG_KERN_ADDR);
                argv_normal[1] = &addr_str[0];

                do_bootm(cmdtp, 0, 2, argv_normal);

                break;

        } /* end of switch */

        do_reset(cmdtp, 0, argc, argv);

    }
    else
    {
        printf("\nBoot Linux from Flash NO RESET PRESSED.\n");

        char *argv_normal[2];
        sprintf(addr_str, "0x%X", CFG_KERN_ADDR);
        argv_normal[1] = &addr_str[0];

        do_bootm(cmdtp, 0, 2, argv_normal);
    }
	/* NOTREACHED - no way out of command loop except booting */
}

void hang (void)
{
	puts ("### ERROR ### Please RESET the board ###\n");
	for (;;);
}

#define RF_CSR_CFG      0xb0180500
#define RF_CSR_KICK     (1<<17)
int rw_rf_reg(int write, int reg, int *data)
{
	u32	rfcsr, i = 0;

	while (1) {
		rfcsr = RALINK_REG(RF_CSR_CFG);
		if (! (rfcsr & (u32)RF_CSR_KICK) )
			break;
		if (++i > 10000) {
			puts("Warning: Abort rw rf register: too busy\n");
			return -1;
		}
	}


	rfcsr = (u32)(RF_CSR_KICK | ((reg & 0x3f) << 8)  | (*data & 0xff));
	if (write)
		rfcsr |= 0x10000;

	RALINK_REG(RF_CSR_CFG) = cpu_to_le32(rfcsr);

	i = 0;
	while (1) {
		rfcsr = RALINK_REG(RF_CSR_CFG);
		if (! (rfcsr & (u32)RF_CSR_KICK) )
			break;
		if (++i > 10000) {
			puts("Warning: still busy\n");
			return -1;
		}
	}

	rfcsr = RALINK_REG(RF_CSR_CFG);

	if (((rfcsr&0x1f00) >> 8) != (reg & 0x1f)) {
		puts("Error: rw register failed\n");
		return -1;
	}
	*data = (int)(rfcsr & 0xff);

	return 0;
}

int do_rw_rf(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	int write, reg, data;

	if ((argv[1][0] == 'r' || argv[1][0] == 'R') && (argc == 3)) {
		write = 0;
		reg = (int)simple_strtoul(argv[2], NULL, 10);
		data = 0;
	}
	else if ((argv[1][0] == 'w' || argv[1][0] == 'W') && (argc == 4)) {
		write = 1;
		reg = (int)simple_strtoul(argv[2], NULL, 10);
		data = (int)simple_strtoul(argv[3], NULL, 16);
	}
	else {
		printf ("Usage:\n%s\n", cmdtp->usage);
		return 1;
	}

	rw_rf_reg(write, reg, &data);
	if (!write)
		printf("rf reg <%d> = 0x%x\n", reg, data);
	return 0;
}

U_BOOT_CMD(
	rf,     4,     1,      do_rw_rf,
	"rf      - read/write rf register\n",
	"usage:\n"
	"rf r <reg>        - read rf register\n"
	"rf w <reg> <data> - write rf register (reg: decimal, data: hex)\n"
);

/*
 * enter power saving mode
 */
void config_usb_ehciohci(void)
{
	u32 val;

	val = RALINK_REG(RT2880_RSTCTRL_REG);    // toggle host & device RST bit
	val = val | RALINK_UHST_RST | RALINK_UDEV_RST;
	RALINK_REG(RT2880_RSTCTRL_REG) = val;

	val = RALINK_REG(RT2880_CLKCFG1_REG);
	val = val & ~(RALINK_UPHY0_CLK_EN | RALINK_UPHY1_CLK_EN) ;  // disable USB port0 & port1 PHY.
	RALINK_REG(RT2880_CLKCFG1_REG) = val;
}


void disable_pcie(void)
{
	u32 val;

	val = RALINK_REG(RT2880_RSTCTRL_REG);    // assert RC RST
	val |= RALINK_PCIE0_RST;
	RALINK_REG(RT2880_RSTCTRL_REG) = val;

	val = RALINK_REG(RT2880_CLKCFG1_REG);    // disable PCIe clock
	val &= ~RALINK_PCIE_CLK_EN;
	RALINK_REG(RT2880_CLKCFG1_REG) = val;
}

//added by mango 20160120
//wled_n GPIO44 WLAN_AN_MODE 2b01
//WDT GPIO38 WDT_MODE 1b1
//modified by Onion 20181128
//GPIO43 P0_LED_AN_MODE 2b01 - GPIO (not ephy activity LED)
void gpio_init(void)
{
	u32 val;
	printf( "Initializing MT7688 GPIO system.\n" );
	//set gpio2_mode 1:0=2b01 wled,p0,p1,p2,p3,p4 as gpio
	val = 0x555;
	RALINK_REG(RT2880_SYS_CNTL_BASE+0x64)=val; // GPIO2_MODE register
	RALINK_REG(0xb0000644)=0x0f<<7;
	//gpio44 output gpio_ctrl_1 bit3=1
	val=RALINK_REG(RT2880_REG_PIODIR+0x04);
	val|=1<<12;
	RALINK_REG(RT2880_REG_PIODIR+0x04)=val;
	//set gpio1_mode 14=1b1
	val=RALINK_REG(RT2880_SYS_CNTL_BASE+0x60);
	val|=1<<14;
	RALINK_REG(RT2880_SYS_CNTL_BASE+0x60)=val;
	//gpio38 input gpio_ctrl_1 bit5=0
	val=RALINK_REG(RT2880_REG_PIODIR+0x04);
	val&=~1<<6;

	RALINK_REG(RT2880_REG_PIODIR+0x04)=val;


  //zh@onion.io
  //setting GPIO 11 High, required for the reset button to work
  val=RALINK_REG(RT2880_REG_PIODIR);
  val|=1<<11;
  RALINK_REG(RT2880_REG_PIODIR) = val; // GPIO 11 direction output
  val=RALINK_REG(RT2880_REG_PIODATA);
  val|=1<<11;
  RALINK_REG(RT2880_REG_PIODATA) = val; // GPIO 11 High

  //jeffzhou@onion.io
  //adding for read wifi MAC address.
  unsigned char macbuf[6];
	raspi_read(macbuf, CFG_FACTORY_ADDR - CFG_FLASH_BASE + 0x04, 6);
	printf("wifi mac address = %02X%02X%02X%02X%02X%02X.\n",
      macbuf[0],macbuf[1],macbuf[2],macbuf[3],macbuf[4],macbuf[5]);
}

void led_on( void )
{
	//gpio44 gpio_dclr_1 644 clear bit12
	RALINK_REG(0xb0000644)=1<<12;
}

void led_off( void )
{
	//gpio44 gpio_dset_1 634 set bit12
	RALINK_REG(0xb0000634)=1<<12;
}

int detect_rst( void )
{
	u32 val;
	val=RALINK_REG(0xb0000624);// Read GPIO 44 (reset button)

    if(val&1<<6)
    {
    	return 1;
    }
    else
    {
        return 0;
    }

}

void gpio_test( int vtest ) //Test Omega2 GPIO
{
	u32 agpio_cfg,gpio1_mode,gpio2_mode,val;
	u32 gpio_ctrl0,gpio_ctrl1,gpio_dat0,gpio_dat1;
	u8 i=0;
	agpio_cfg = RALINK_REG(RT2880_SYS_CNTL_BASE+0x3c);
	gpio1_mode= RALINK_REG(RT2880_SYS_CNTL_BASE+0x60);
	gpio2_mode= RALINK_REG(RT2880_SYS_CNTL_BASE+0x64);
	gpio_ctrl0= RALINK_REG(0xb0000600);
	gpio_ctrl1= RALINK_REG(0xb0000604);
	gpio_dat0 = RALINK_REG(0xb0000620);
	gpio_dat1 = RALINK_REG(0xb0000624);
	//agpio
	val=0;
	val|=0x0f<<17;//ephy p1-p4 selection digital PAD
	val|=0x1f;//refclk,i2s digital PAD #GPIO37 GPIO0~3
	RALINK_REG(RT2880_SYS_CNTL_BASE+0x3c)=val;
	//gpio1_mode
	val=0;
	val|=0x05<<28;//pwm0,pwm1 #GPIO18 19
	val|=0x05<<24;//uart1,uart2 #GPIO45 46,GPIO20 21
	val|=0x01<<20;//i2c_mode #GPIO4 5
	val|=0x01<<18;//refclk   #GPIO37
	val|=0x01<<14;//wdt_mode #GPIO38
	//val|=0x01<<10;//sd_mode  #GPIO22~29
	val|=0x01<<8;//uart0 GPIO12 13
	val|=0x01<<6;//i2s GPIO0~3
	val|=0x01<<4;//cs1 GPIO6
	val|=0x01<<2;//spis GPIO14~17
	RALINK_REG(RT2880_SYS_CNTL_BASE+0x60)=val;
	//gpio2_mode
	val=0;
	val|=0x01<<10;//p4led GPIO39
	val|=0x01<<8;//p3 led GPIO40
	val|=0x01<<6;//p2 led GPIo41
	val|=0x01<<4;//p1 led GPIo42
	val|=0x01<<2;//p0 led Gpio43
	val|=0x01<<0;//wled   GPIO44
	RALINK_REG(RT2880_SYS_CNTL_BASE+0x64)=val;
	//ctrl0,ctrl1
	RALINK_REG(0xb0000600)=0xffffffff;
	RALINK_REG(0xb0000604)=0xffffffff;
	RALINK_REG(0xb0000604)&=~0x01<<6;

	udelay(600000);

	for(i=0;i<1;i++)
	{
		printf("\nall led off GPIO high\n");
		RALINK_REG(0xb0000620)=0xffffffff;
		RALINK_REG(0xb0000624)=0xffffffff;
		udelay(1000000);

		printf("\nall led on GPIO low\n");
		RALINK_REG(0xb0000620)=0x0;
		RALINK_REG(0xb0000624)=0x0;
		udelay(400000);

		//==========
		printf("\nall led off GPIO high\n");
		RALINK_REG(0xb0000620)=0xffffffff;
		RALINK_REG(0xb0000624)=0xffffffff;
		udelay(1000000);

		printf("\nall led on GPIO low\n");
		RALINK_REG(0xb0000620)=0x0;
		RALINK_REG(0xb0000624)=0x0;
		udelay(300000);

      		if (vtest == 0)
		{
		//========Test Omega2==start========
		//G11 G3 G2 G17 16 15 G46 G45 G6 G1 G0
	  	RALINK_REG(0xb0000620)=0x800;		//G11
		udelay(300000);
		RALINK_REG(0xb0000620)=0x0;
		udelay(200000);

		RALINK_REG(0xb0000620)=0x8;		//G3
		udelay(300000);
		RALINK_REG(0xb0000620)=0x0;
		udelay(200000);

	  	RALINK_REG(0xb0000620)=0x4;		//G2
		udelay(300000);
		RALINK_REG(0xb0000620)=0x0;
		udelay(200000);
		//==
		RALINK_REG(0xb0000620)=0x20000;		//G17
		udelay(300000);
		RALINK_REG(0xb0000620)=0x0;
		udelay(200000);

	  	RALINK_REG(0xb0000620)=0x10000;		//G16
		udelay(300000);
		RALINK_REG(0xb0000620)=0x0;
		udelay(200000);

	  	RALINK_REG(0xb0000620)=0x8000;		//G15
		udelay(300000);
		RALINK_REG(0xb0000620)=0x0;
		udelay(200000);
		//==

		RALINK_REG(0xb0000624)=0x4000; 		//G46
		udelay(300000);
		RALINK_REG(0xb0000624)=0x0;
		udelay(200000);

		RALINK_REG(0xb0000624)=0x2000;		//G45
		udelay(300000);
		RALINK_REG(0xb0000624)=0x0;
		udelay(200000);

		RALINK_REG(0xb0000620)=0x40;		//G6
		udelay(300000);
		RALINK_REG(0xb0000620)=0x0;
		udelay(200000);

	  	RALINK_REG(0xb0000620)=0x2;		//G1
		udelay(300000);
		RALINK_REG(0xb0000620)=0x0;
		udelay(100000);

		RALINK_REG(0xb0000620)=0x1;		//G0
		udelay(300000);
		RALINK_REG(0xb0000620)=0x0;
		udelay(200000);

		//G5 G4 G19 G18 G12 13

		RALINK_REG(0xb0000620)=0x20;		//G5
		udelay(300000);
		RALINK_REG(0xb0000620)=0x0;
		udelay(200000);

	  	RALINK_REG(0xb0000620)=0x10;		//G4
		udelay(300000);
		RALINK_REG(0xb0000620)=0x0;
		udelay(200000);

		RALINK_REG(0xb0000620)=0x80000;		//G19
		udelay(300000);
		RALINK_REG(0xb0000620)=0x0;
		udelay(200000);

	  	RALINK_REG(0xb0000620)=0x40000;		//G18
		udelay(300000);
		RALINK_REG(0xb0000620)=0x0;
		udelay(200000);

		//========Test Omega2==end========
		}
		else if(vtest == 1)
		{
		//=========Test Omega2S===start===
		RALINK_REG(0xb0000620)=0x1;		//G0
		udelay(300000);
		RALINK_REG(0xb0000620)=0x0;
		udelay(200000);

	  	RALINK_REG(0xb0000620)=0x2;		//G1
		udelay(300000);
		RALINK_REG(0xb0000620)=0x0;
		udelay(100000);

	 	RALINK_REG(0xb0000620)=0x4;		//G2
		udelay(300000);
		RALINK_REG(0xb0000620)=0x0;
		udelay(200000);

		RALINK_REG(0xb0000620)=0x8;		//G3
		udelay(300000);
		RALINK_REG(0xb0000620)=0x0;
		udelay(200000);

	  	RALINK_REG(0xb0000620)=0x10;		//G4
		udelay(300000);
		RALINK_REG(0xb0000620)=0x0;
		udelay(200000);

		RALINK_REG(0xb0000620)=0x20;		//G5
		udelay(300000);
		RALINK_REG(0xb0000620)=0x0;
		udelay(200000);

		RALINK_REG(0xb0000620)=0x40;		//G6
		udelay(300000);
		RALINK_REG(0xb0000620)=0x0;
		udelay(200000);

	    	RALINK_REG(0xb0000620)=0x800;		//G11
		udelay(300000);
		RALINK_REG(0xb0000620)=0x0;
		udelay(200000);



		//-------------
	  	RALINK_REG(0xb0000620)=0x4000;		//G14
		udelay(300000);
		RALINK_REG(0xb0000620)=0x0;
		udelay(200000);

	  	RALINK_REG(0xb0000620)=0x8000;		//G15
		udelay(300000);
		RALINK_REG(0xb0000620)=0x0;
		udelay(200000);

	  	RALINK_REG(0xb0000620)=0x10000;		//G16
		udelay(300000);
		RALINK_REG(0xb0000620)=0x0;
		udelay(200000);

		RALINK_REG(0xb0000620)=0x20000;		//G17
		udelay(300000);
		RALINK_REG(0xb0000620)=0x0;
		udelay(200000);

	  	RALINK_REG(0xb0000620)=0x40000;		//G18
		udelay(300000);
		RALINK_REG(0xb0000620)=0x0;
		udelay(200000);

		RALINK_REG(0xb0000620)=0x80000;		//G19
		udelay(300000);
		RALINK_REG(0xb0000620)=0x0;
		udelay(200000);

		RALINK_REG(0xb0000620)=0x100000;	//G20
		udelay(300000);
		RALINK_REG(0xb0000620)=0x0;
		udelay(200000);

		RALINK_REG(0xb0000620)=0x200000;	//G21
		udelay(300000);
		RALINK_REG(0xb0000620)=0x0;
		udelay(200000);

		//----------------
		RALINK_REG(0xb0000624)=0x10;		//G36
		udelay(300000);
		RALINK_REG(0xb0000624)=0x0;
		udelay(200000);

		RALINK_REG(0xb0000624)=0x20;		//G37
		udelay(300000);
		RALINK_REG(0xb0000624)=0x0;
		udelay(200000);


		RALINK_REG(0xb0000624)=0x80;		//G39
		udelay(300000);
		RALINK_REG(0xb0000624)=0x0;
		udelay(200000);

		RALINK_REG(0xb0000624)=0x100;		//G40
		udelay(300000);
		RALINK_REG(0xb0000624)=0x0;
		udelay(200000);

		RALINK_REG(0xb0000624)=0x200;		//G41
		udelay(300000);
		RALINK_REG(0xb0000624)=0x0;
		udelay(200000);

		RALINK_REG(0xb0000624)=0x400;		//G42
		udelay(300000);
		RALINK_REG(0xb0000624)=0x0;
		udelay(200000);

		RALINK_REG(0xb0000624)=0x800;		//G43
		udelay(300000);
		RALINK_REG(0xb0000624)=0x0;
		udelay(200000);

		RALINK_REG(0xb0000624)=0x1000;		//G44
		udelay(300000);
		RALINK_REG(0xb0000624)=0x0;
		udelay(200000);

		RALINK_REG(0xb0000624)=0x2000;		//G45
		udelay(300000);
		RALINK_REG(0xb0000624)=0x0;
		udelay(200000);

		RALINK_REG(0xb0000624)=0x4000; 		//G46
		udelay(300000);
		RALINK_REG(0xb0000624)=0x0;
		udelay(200000);

		//=========Test Omega2S===end===
		}
	}

	RALINK_REG(RT2880_SYS_CNTL_BASE+0x3c)=agpio_cfg;
	RALINK_REG(RT2880_SYS_CNTL_BASE+0x60)=gpio1_mode;
	RALINK_REG(RT2880_SYS_CNTL_BASE+0x64)=gpio2_mode;
	RALINK_REG(0xb0000600)=gpio_ctrl0;
	RALINK_REG(0xb0000604)=gpio_ctrl1;
	RALINK_REG(0xb0000620)=gpio_dat0;
	RALINK_REG(0xb0000624)=gpio_dat1;
}

