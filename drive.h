// drive.h class for driving the gpio-pins
//
//

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

// rewritten from an example, (I think) orginal written by http://www.pieter-jan.com/
class gpio_driver_class {
public:
	gpio_driver_class() {
		mem_fd = 0;
	}
	~gpio_driver_class() {
		if (mem_fd != 0) {
			munmap(gpio_map, BLOCK_SIZE);
			close(mem_fd);
		}
	}
public:
	long BCM2708_PERI_BASE = 0x3F000000; 					// #define BCM2708_PERI_BASE         0x3F000000
	long GPIO_BASE	= BCM2708_PERI_BASE +  0x200000 ;		// #define GPIO_BASE (BCM2708_PERI_BASE + 0x200000) /* GPIO controller */
	int PAGE_SIZE =  4*1024;
	int BLOCK_SIZE =  4*1024;
	int  mem_fd;
	void *gpio_map;
	// I/O access
	volatile unsigned int *gpio;
public:
	// GPIO setup macros.
	bool setup();
	// direction: input
	bool set_inp(int g) {
		*(gpio+((g)/10)) &= ~(7<<(((g)%10)*3));
		return true;
	}
	// direction output
	bool set_out(int g) {
		// Always use INP_GPIO(x) before using OUT_GPIO(x) or SET_GPIO_ALT(x,y)
		set_inp(g);
		*(gpio+((g)/10)) |=  (1<<(((g)%10)*3));
		return true;
	}
	// sets bits which are 1 ignores bits which are 0
	inline void bit_set(int g) {
		*(gpio + 7) = g;
	}
	// clears bits which are 1 ignores bits which are 0
	inline void bit_clr(int g) {
		*(gpio + 10) = g;
	}
	// 0 if LOW, (1<<g) if HIGH
	inline int bit_get(int g) {
		// 0 if LOW, (1<<g) if HIGH
		return (*(gpio+13)&(1<<g));
	}
	inline int bits_get(int g) {
		return (*(gpio+13) & g);
	}
	/*
	#define OUT_GPIO(g) *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))
	#define SET_GPIO_ALT(g,a) *(gpio+(((g)/10))) |= (((a)<=3?(a)+4:(a)==4?3:2)<<(((g)%10)*3))
	#define GPIO_SET *(gpio+7)  // sets   bits which are 1 ignores bits which are 0
	#define GPIO_CLR *(gpio+10) // clears bits which are 1 ignores bits which are 0
	#define GET_GPIO(g) (*(gpio+13)&(1<<g)) // 0 if LOW, (1<<g) if HIGH
	#define GPIO_PULL *(gpio+37) // Pull up/pull down
	#define GPIO_PULLCLK0 *(gpio+38) // Pull up/pull down clock
	*/
};
//
// Set up a memory regions to access GPIO
//
bool gpio_driver_class::setup()
{
	// open /dev/mem
	if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0) {
		printf("can't open /dev/mem \n");
		exit(-1);
	}
	// mmap GPIO
	gpio_map = mmap(
	               NULL,             // Any adddress in our space will do
	               BLOCK_SIZE,       // Map length
	               PROT_READ|PROT_WRITE,// Enable reading & writting to mapped memory
	               MAP_SHARED,       // Shared with other processes
	               mem_fd,           // File to map
	               GPIO_BASE         // Offset to GPIO peripheral
	           );
	if (gpio_map == MAP_FAILED) {
		printf("mmap error %d\n", (int) gpio_map); // errno also set!
		return false;
	}
	// Always use volatile pointer!
	gpio = (volatile unsigned int*) gpio_map;
	return true;
}


