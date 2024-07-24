/*
 * riscv-esp32s3r8.c - RISCv emulator running Linux
 *
 * Written by Leonardo Soares <leonardobsi@gmail.com>
 *
 */

#include <stdio.h>
#include <string.h>

#include <driver/uart.h>
#include <driver/gpio.h>
#include "esp_littlefs.h"
#include "esp_private/wifi.h"

#include "esp_timer.h"

#include "tinyusb.h"
#include "tusb_cdc_acm.h"
#include "tusb_console.h"

#define MIN(x, y) (x < y ? x : y)
#define MAX(x, y) (x > y ? x : y)

#define MAX_VQUEUE_SIZE 32 /*Number max of items in the queue (a power of 2)*/

struct virtq_used_elem_t {
	uint32_t id;
	uint32_t len;
};

struct virtq_used_t {
	uint16_t flags;
	uint16_t idx;
	struct virtq_used_elem_t ring[MAX_VQUEUE_SIZE];
	uint16_t used_event; /* Only if VIRTIO_F_EVENT_IDX */
};

struct virtq_avail_t {
	uint16_t flags;
	uint16_t idx;
	uint16_t ring[MAX_VQUEUE_SIZE];
	uint16_t avail_event; /* Only if VIRTIO_F_EVENT_IDX */
};

struct virtq_desc_t {
	uint64_t addr;
	uint32_t len;
	uint16_t flags;
	uint16_t next;
};

struct virtio_queue_t {
	uint32_t num;
	uint32_t ready;
	uint32_t desc_addr;
	uint32_t driver_addr;
	uint32_t dev_addr;
	uint8_t notify;
};

struct virtio_t {
	uint32_t dev_id;
	uint32_t dev_status;
	uint32_t dev_feat_sel;
	uint64_t dev_feat;
	uint32_t guest_feat_sel;
	uint64_t guest_feat;
	uint32_t int_status;
	uint32_t queue_sel;
	uint8_t *config_space;
	struct virtio_queue_t queue[3 /*Block Device... Console Device... GPIO Device...*/];
} virtio8_net = { .dev_id = 1 }, virtio2_blk = { .dev_id = 2 }, virtio1_console = { .dev_id = 3 }, virtio4_i2c = { .dev_id = 34 }, virtio3_gpio = { .dev_id = 41 };

struct uart_t {
	uint8_t ier /*Interrupt Enable Register*/;
	uint8_t fcr /*FIFO Control Register*/;
	uint8_t lcr /*Line Control Register*/;
	uint8_t lsr /*Line Status Register*/;
	uint8_t isr /*Interrupt Status Register*/;
	uint8_t mcr /*Modem Control Register*/;
	uint8_t msr /*Modem Status Register*/;
	uint8_t dll /*Divisor LSB*/;
	uint8_t dlm /*Divisor MSB*/;
	uint8_t thr /*Transmit Holding Register*/;
	uint8_t rhr /*Receive Holding Register*/;
	uint8_t spr /*Scratch Pad Register*/;
} uart_com_1 = { .lsr = 0 | (1 << 5 /*THR Empty*/) };

struct clint_t {
	uint64_t time, timecmp;
} clint;

struct csr_t {
	struct satp_t {
		uint32_t ppn :22;
		uint16_t asid :9;
		uint8_t mode :1;
	} satp;
	uint32_t fflags;
	uint32_t frm;
	uint32_t fcsr;
	uint32_t misa;
	uint32_t mstatus, sstatus;
	uint32_t mhartid;
	uint32_t mscratch, sscratch;
	uint32_t mtvec, stvec;
	uint32_t mcountern, scounteren;
	uint32_t mie, sie;
	uint32_t mip, sip;
	uint32_t pmpaddr0;
	uint32_t pmpcfg0;
	uint32_t mepc, sepc;
	uint32_t mcause, scause;
	uint32_t mtval, stval;
	uint64_t *time;
	uint32_t mideleg;
	uint32_t medeleg;
} csr = { .time = &clint.time, .mhartid = 0x00 };

enum RISCV_SHIFT_EXCEPTIONS {
	INSTRUCTION_ADDRESS_MISALIGNED = 0x0, 	/*Instruction address misaligned: indicates that an instruction was fetched from an address that is not aligned to the instruction width*/
	INSTRUCTION_ACCESS_FAULT = 0x1, 		/*Instruction access fault: indicates that an attempt to fetch an instruction failed due to an access violation*/
	ILLEGAL_INSTRUCTION = 0x2, 				/*Illegal instruction: indicates that an invalid or unsupported instruction was encountered*/
	BREAKPOINT = 0x3, 						/*Breakpoint: indicates that the debug interrupt (breakpoint) instruction was encountered*/
	LOAD_ADDRESS_MISALIGNED = 0x4, 			/*Load address misaligned: indicates that a memory read operation was attempted at an address that is not aligned to the operation size*/
	LOAD_ACCESS_FAULT = 0x5, 				/*Load access fault: indicates that a memory read operation failed due to an access violation*/
	STORE_AMO_ADDRESS_MISALIGNED = 0x6, 	/*Store/AMO address misaligned: indicates that a memory write operation or an atomic operation was attempted at an address that is not aligned to the operation size*/
	STORE_AMO_ACCESS_FAULT = 0x7, 			/*Store/AMO access fault: indicates that a memory write operation or an atomic operation failed due to an access violation*/
	ENV_CALL_FROM_UMODE = 0x8, 				/*Environment call from U-mode: indicates that a system call was made in user mode (U-mode)*/
	ENV_CALL_FROM_SMODE = 0x9, 				/*Environment call from S-mode: indicates that a system call was made in supervisor mode (S-mode)*/
	ENV_CALL_FROM_MMODE = 0xb, 				/*Environment call from M-mode: indicates that a system call was made in machine mode (M-mode)*/
	INSTRUCTION_PAGE_FAULT = 0xc, 			/*Instruction page fault: indicates that an attempt to fetch an instruction failed due to a page fault*/
	LOAD_PAGE_FAULT = 0xd, 					/*Load page fault: indicates that a memory read operation failed due to a page fault*/
	STORE_AMO_PAGE_FAULT = 0xf 				/*Store/AMO page fault: indicates that a memory write operation or an atomic operation failed due to a page fault*/
};

enum RISCV_SHIFT_INTERRUPTIONS {
	SUPERVISOR_SOFTWARE_INTERRUPT = 0x1,
	MACHINE_SOFTWARE_INTERRUPT = 0x3,
	SUPERVISOR_TIMER_INTERRUPT = 0x5,
	MACHINE_TIMER_INTERRUPT = 0x7,
	SUPERVISOR_EXTERNAL_INTERRUPT = 0x9,
	MACHINE_EXTERNAL_INTERRUPT = 0xb
};

struct plic_t {
	uint32_t sclaim;
} plic;

enum RISCV_BASEADDRS {
	RAM_BASE = 		0x80000000,
	PLIC_BASE = 	0x40100000,
	VIRTIO8_BASE = 	0x40080000,
	SYSCON_BASE = 	0x40060000,
	UART1_BASE = 	0x40050000,
	VIRTIO4_BASE = 	0x40040000,
	VIRTIO3_BASE = 	0x40030000,
	VIRTIO2_BASE = 	0x40020000,
	VIRTIO1_BASE = 	0x40010000,
	CLINT_BASE = 	0x02000000,
	DTB_BASE = 		0x00000000
};

typedef enum RISCV_PRIVILEGES {
	USER = 0x00, SUPERVISOR = 0x01, MACHINE = 0x03
} privelege_t;

struct cpu_t {
	uint64_t freg[32];
	uint32_t pc, xreg[32], reserved;
	privelege_t privilege;
} cpu = { .pc = 0x80000000, .xreg = { [10 /*a0*/]= 0x00, [11 /*a1*/]= DTB_BASE }, .privilege = MACHINE };

FILE *rootfs;

uint8_t *low_memory;

const esp_partition_t *partition_memory;

#define LRU_SEC_SIZE (1 << 12 /*4Kb*/)

uint32_t lru_time;

struct window_t {
	uint8_t w;
	uint8_t *data;
	uint16_t sector;
	uint32_t time;
};
struct window_t window_[0xf00000 /*15MB*/ / LRU_SEC_SIZE];

struct window_t* cpu_lru_translate(uint32_t addr) {

	struct window_t *win = &window_[addr / LRU_SEC_SIZE];

	win->time = ++lru_time;

	if (win->data)
		return win;

	win->data= heap_caps_malloc_prefer(LRU_SEC_SIZE, 2, MALLOC_CAP_SPIRAM, MALLOC_CAP_INTERNAL);

	esp_partition_read(partition_memory, win->sector * LRU_SEC_SIZE, win->data, LRU_SEC_SIZE);

	struct window_t *tmp= (struct window_t*) 0;
	for(uint32_t x = 0; x < (0xf00000 / LRU_SEC_SIZE); x++ )
		if(window_[x].data && (!tmp || window_[x].time < tmp->time)){
			tmp = &window_[x];
		}

	tmp->sector = win->sector;

	if(tmp->w){
		esp_partition_erase_range(partition_memory, tmp->sector * LRU_SEC_SIZE, LRU_SEC_SIZE);
		esp_partition_write(partition_memory, tmp->sector * LRU_SEC_SIZE, tmp->data, LRU_SEC_SIZE);
	}
	heap_caps_free(tmp->data);

	tmp->data = (uint8_t*) 0;

	return win;
}

void stream_lru_read(uint32_t addr, uint32_t size, uint8_t *buff) {

	struct window_t *win = cpu_lru_translate( addr );

	uint16_t reading= MIN(LRU_SEC_SIZE - (addr % LRU_SEC_SIZE), size);
	memcpy(buff, win->data + (addr % LRU_SEC_SIZE), reading);

	if (size - reading)
		stream_lru_read(addr + reading, size - reading, buff + reading);
}

void stream_lru_write(uint32_t addr, uint32_t size, uint8_t *buff) {

	struct window_t *win = cpu_lru_translate( addr );
	win->w |= 0x01;

	uint16_t reading= MIN(LRU_SEC_SIZE - (addr % LRU_SEC_SIZE), size);
	memcpy(win->data + (addr % LRU_SEC_SIZE), buff, reading);

	if (size - reading)
		stream_lru_write(addr + reading, size - reading, buff + reading);
}

void cpu_take_trap(uint32_t cause, uint32_t tval) {

	if (cpu.privilege <= SUPERVISOR
			&& (((cause & 0x80000000) && ((csr.mideleg >> (cause & 0x7FFFFFFF)) & 1)) || ((~cause & 0x80000000) && ((csr.medeleg >> cause) & 1)))) {

		csr.sepc = cpu.pc;

		if (cause & (1 << 31)) {
			/*This bit is set when the exception was triggered by an interrupt.*/

			uint32_t vectored = (csr.stvec & 3) ? ((cause & 0x7FFFFFFF) << 2) : 0;
			cpu.pc = (csr.stvec & ~3) + vectored;

		} else {

			cpu.pc = csr.stvec & ~3;
		}

		csr.scause = cause;
		csr.stval = tval;

		csr.sstatus &= ~(1 << 5 /*SPIE*/);                         	// clear SPIE
		csr.sstatus |= ((csr.sstatus >> cpu.privilege) & 1) << 5; 	// copies SIE into SPIE
		csr.sstatus &= ~(1 << cpu.privilege /*SIE*/);               // clear SIE

		csr.sstatus &= ~(1 << 8 /*SPP*/);          		// clear SPP
		csr.sstatus |= (cpu.privilege << 8 /*SPP*/); 	// copies current privilege into SPP

		cpu.privilege = SUPERVISOR;

	} else {

		csr.mepc = cpu.pc;

		if (cause & (1 << 31)) {
			/*This bit is set when the exception was triggered by an interrupt.*/

			uint32_t vectored = (csr.mtvec & 3) ? ((cause & 0x7FFFFFFF) << 2) : 0;
			cpu.pc = (csr.mtvec & ~3) + vectored;

		} else {

			cpu.pc = csr.mtvec & ~3;
		}

		csr.mcause = cause;
		csr.mtval = tval;

		csr.mstatus &= ~(1 << 7 /*MPIE*/);                        	// clear MPIE
		csr.mstatus |= ((csr.mstatus >> cpu.privilege) & 1) << 7; 	// copies MIE into MPIE
		csr.mstatus &= ~(1 << cpu.privilege /*MIE*/);             	// clear MIE

		csr.mstatus &= ~(3 << 11 /*MPP*/);              // clear MPP
		csr.mstatus |= (cpu.privilege << 11 /*MPP*/); 	// copies current privilege into MPP

		cpu.privilege = MACHINE;
	}
}

#define TLB

/* The TLB is a high-level cache of the page table, which stores the most recently used translations and makes them quickly and efficiently accessible.
 * Instead of accessing the page table in main memory, the processor first checks the TLB to see if the translation of the virtual address is already stored there.
 * If it is, the translation is used directly, without the need to access the page table in main memory.
 * This reduces memory access latency and increases system performance. */

#define TLB_SIZE 89

struct tlb_t {
	uint32_t vaddr;
	uint32_t paddr;
	uint8_t except;
} tlb_[TLB_SIZE], *tlb;

int8_t cpu_addr_translate(uint32_t *addr, uint8_t except) {

	tlb = &tlb_[(*addr >> 12) % TLB_SIZE];

#ifdef TLB
	if (tlb->vaddr == (*addr & ~0xfff) && tlb->except == except) {
		*addr = (tlb->paddr | (*addr & 0xfff));
		return 1;
	}
#endif

	if (csr.satp.mode && cpu.privilege <= SUPERVISOR) {

		/* ppn - physical page number
		 * vpn - virtual page number
		 * pte - page table entry */

		struct sv32_va_t {
			uint32_t offset :12;
			uint32_t vpn_0 :10;
			uint32_t vpn_1 :10;
		} *sv32_va = (struct sv32_va_t*) addr;

		uint32_t vpn[2] = { sv32_va->vpn_0, sv32_va->vpn_1 };

		struct sv32_pte_t {
			uint8_t v :1;
			uint8_t r :1;
			uint8_t w :1;
			uint8_t x :1;
			uint8_t u :1;
			uint8_t g :1;
			uint8_t a :1;
			uint8_t d :1;
			uint8_t rsw :2;
			uint32_t ppn_0 :10;
			uint32_t ppn_1 :12;
		} sv32_pte;

		uint32_t a = csr.satp.ppn << 12;
		for (int i = 1; i >= 0 /*sv32 2 levels*/; i--) {

			stream_lru_read((a | vpn[i] << 2) - RAM_BASE, sizeof(struct sv32_pte_t), (uint8_t*) &sv32_pte);

			if (!sv32_pte.v) {
				cpu_take_trap(except, *addr);
				return 0;
			}

			if (sv32_pte.r || sv32_pte.x) {

				if (except == STORE_AMO_PAGE_FAULT && !sv32_pte.w) {
					cpu_take_trap(except, *addr);
					return 0;
				}

				if (!sv32_pte.a) {

					sv32_pte.a = 1;
					stream_lru_write((a | vpn[i] << 2) - RAM_BASE, sizeof(struct sv32_pte_t), (uint8_t*) &sv32_pte);
				}

				if (except == STORE_AMO_PAGE_FAULT && !sv32_pte.d) {

					sv32_pte.d = 1;
					stream_lru_write((a | vpn[i] << 2) - RAM_BASE, sizeof(struct sv32_pte_t), (uint8_t*) &sv32_pte);
				}

//				assert( !(except == STORE_AMO_PAGE_FAULT && !sv32_pte.w) );
//
//				assert( !(except == INSTRUCTION_PAGE_FAULT && !sv32_pte.x) );
//				assert(!(csr.sstatus & (1 << 19 /*mxr*/)));
//
//				assert( !(except == LOAD_PAGE_FAULT && !sv32_pte.r) );
//				assert( !(sv32_pte.a == 0) );
//				assert( !(except == STORE_AMO_PAGE_FAULT && !sv32_pte.d) );
//
//				uint8_t privilege = cpu.privilege;
//
//				if (((csr.sstatus >> 17) & 1 /*MPRV*/) && (except != INSTRUCTION_PAGE_FAULT)) {
//					privilege = ((csr.sstatus >> 11) & 3 /*MPP*/);
//				}
//				assert( !(privilege == USER && !sv32_pte.u) );
//
//				assert( !((privilege == SUPERVISOR) && sv32_pte.u && !(csr.sstatus & (1 << 18 /*sum*/))) );

				tlb->vaddr = (*addr & ~0xfff);

				switch (i) {
				case 1:
					*addr = ((sv32_pte.ppn_1 << 10) | sv32_va->vpn_0) << 12 | sv32_va->offset;
					break;
				case 0:
					*addr = ((sv32_pte.ppn_1 << 10) | sv32_pte.ppn_0) << 12 | sv32_va->offset;
				}

				tlb->paddr = (*addr & ~0xfff);
				tlb->except = except;

				return 1;
			}
			a = ((sv32_pte.ppn_1 << 10) | sv32_pte.ppn_0) << 12;
		}

		cpu_take_trap(except, *addr);

		return 0;
	}

	return 1;
}

void csr_read(uint32_t offset, uint32_t *value) {
//	assert( cpu.privilege >= ((offset >> 8) & 3 /*priv*/) );

	switch (offset) {
	case 0x001:
		*value = csr.fflags;
		break;
	case 0x002:
		*value = csr.frm;
		break;
	case 0x003:
		*value = csr.fcsr;
		break;
	case 0x100:
		*value = csr.sstatus;
		break;
	case 0x104:
		*value = csr.sie;
		break;
	case 0x105:
		*value = csr.stvec;
		break;
	case 0x106:
		*value = csr.scounteren;
		break;
	case 0x140:
		*value = csr.sscratch;
		break;
	case 0x141:
		*value = csr.sepc;
		break;
	case 0x142:
		*value = csr.scause;
		break;
	case 0x143:
		*value = csr.stval;
		break;
	case 0x144:
		*value = csr.sip;
		break;
	case 0x180:
		memcpy(value, &csr.satp, sizeof(uint32_t));
		break;
	case 0x300:
		*value = csr.mstatus;
		break;
	case 0x301:
		*value = csr.misa;
		break;
	case 0x302:
		*value = csr.medeleg;
		break;
	case 0x303:
		*value = csr.mideleg;
		break;
	case 0x304:
		*value = csr.mie;
		break;
	case 0x305:
		*value = csr.mtvec;
		break;
	case 0x306:
		*value = csr.mcountern;
		break;
	case 0x340:
		*value = csr.mscratch;
		break;
	case 0x341:
		*value = csr.mepc;
		break;
	case 0x342:
		*value = csr.mcause;
		break;
	case 0x343:
		*value = csr.mtval;
		break;
	case 0x344:
		*value = csr.mip;
		break;
	case 0x3a0:
		*value = csr.pmpcfg0;
		break;
	case 0x3b0:
		*value = csr.pmpaddr0;
		break;
	case 0xc01:
		*value = *csr.time;
		break;
	case 0xc81:
		*value = ( *csr.time >> 32);
		break;
	case 0xf14:
		*value = csr.mhartid;
		break;
	default:
		assert(0);
	}
}

void csr_write(uint32_t offset, uint32_t value) {
//	assert( cpu.privilege >= ((offset >> 8) & 3 /*priv*/) );

	switch (offset) {
	case 0x001:
		csr.fflags = value;
		break;
	case 0x002:
		csr.frm = value;
		break;
	case 0x003:
		csr.fcsr = value;
		break;
	case 0x100:
		csr.sstatus = value;
		break;
	case 0x104:
		csr.sie = value;
		break;
	case 0x105:
		csr.stvec = value;
		break;
	case 0x106:
		csr.scounteren = value;
		break;
	case 0x140:
		csr.sscratch = value;
		break;
	case 0x141:
		csr.sepc = value;
		break;
	case 0x142:
		csr.scause = value;
		break;
	case 0x143:
		csr.stval = value;
		break;
	case 0x144:
		csr.sip = value;
		break;
	case 0x180:

		if (csr.satp.asid != ((value >> 22) & 0x1ff) /*asid*/) {
			// tlb flush...
			memset(&tlb_, -1, TLB_SIZE * sizeof(struct tlb_t));
		}

		csr.satp = *(struct satp_t*) &value;
		break;
	case 0x300:
		csr.mstatus = value;
		break;
	case 0x301:
		csr.misa = value;
		break;
	case 0x302:
		csr.medeleg = value;
		break;
	case 0x303:
		csr.mideleg = value;
		break;
	case 0x304:
		csr.mie = value;
		break;
	case 0x305:
		csr.mtvec = value;
		break;
	case 0x306:
		csr.mcountern = value;
		break;
	case 0x340:
		csr.mscratch = value;
		break;
	case 0x341:
		csr.mepc = value;
		break;
	case 0x342:
		csr.mcause = value;
		break;
	case 0x343:
		csr.mtval = value;
		break;
	case 0x344:
		csr.mip = value;
		break;
	case 0x3a0:
		csr.pmpcfg0 = value;
		break;
	case 0x3b0:
		csr.pmpaddr0 = value;
		break;
	case 0xc01:
		/*time*/
		break;
	case 0xc81:
		/*timeh*/
		break;
	case 0xf14:
		csr.mhartid = value;
		break;
	default:
		assert(0);
	}
}

void plic_read(uint32_t offset, uint32_t *value) {

	switch (offset) {
	case 0x00200000:
		*value = 0;
		break;
	case 0x00200004:

		*value = 0;

		for(int irq= 1; irq < 32; irq++)
			if (plic.sclaim & (1 << irq))
				*value = irq;

		break;
	default:
		*value = 0;
	}
}

void plic_write(uint32_t offset, uint32_t value) {

	switch (offset) {
	case 0x200004: /*Claim acknowledge*/
		plic.sclaim &= ~(1 << value);
		break;
	}
}

void clint_read(uint32_t offset, uint32_t *value) {

	switch (offset) {
	case 0x00004000:
		*value = clint.timecmp;
		break;
	case 0x00004004:
		*value = *((uint32_t*) &clint.timecmp + 1);
		break;
	default:
		*value = 0;
		break;
	}
}

void clint_write(uint32_t offset, uint32_t value) {

	switch (offset) {
	case 0x00004000:
		*(uint32_t*) &clint.timecmp= value;
		break;
	case 0x00004004:
		*((uint32_t*) &clint.timecmp + 1)= value;
		break;
	default:
		break;
	}
}

void uart1_write(uint32_t offset, uint8_t value) {

	switch (offset) {
	case 0b000:

		if (~uart_com_1.lcr & (1 << 7 /*Divisor Enable*/)) {
			uart_com_1.thr = value;
			uart_write_bytes(UART_NUM_1, &uart_com_1.thr, sizeof(uart_com_1.thr));
		} else {
			uart_com_1.dll = value;
			ESP_ERROR_CHECK( uart_set_baudrate(UART_NUM_1, 3686400 / (16 * uart_com_1.dll)));
		}
		break;
	case 0b001:
		(uart_com_1.lcr & (1 << 7 /*Divisor Enable*/) ? (uart_com_1.dlm = value) : (uart_com_1.ier = value));

		if (uart_com_1.ier & (1 << 1 /*TX (THR)*/)) {

			plic.sclaim |= (1 << 0x05 /*uart_com_1's irq*/);
			csr.sip |= (1 << SUPERVISOR_EXTERNAL_INTERRUPT);
		}

		break;
	case 0b010:
		uart_com_1.fcr = value;
		break;
	case 0b011:
		uart_com_1.lcr = value;

		ESP_ERROR_CHECK(uart_set_word_length(UART_NUM_1, uart_com_1.lcr & 0x03));

		if ((uart_com_1.lcr >> 3) /*Parity Enable*/& 1) {
			ESP_ERROR_CHECK( uart_set_parity(UART_NUM_1, ((uart_com_1.lcr >> 4) /*Even Parity*/& 1) ? UART_PARITY_EVEN : UART_PARITY_ODD));
		} else
			ESP_ERROR_CHECK (uart_set_parity( UART_NUM_1, UART_PARITY_DISABLE));

		uint8_t stop_bits[2] = { UART_STOP_BITS_1, UART_STOP_BITS_2 };
		ESP_ERROR_CHECK( uart_set_stop_bits(UART_NUM_1, stop_bits[(uart_com_1.lcr >> 2 /*Stop bits*/) & 1]));

		break;
	case 0b100:
		uart_com_1.mcr = value;
		break;
	case 0b111:
		uart_com_1.spr = value;
		break;
	default:
		assert(0);
	}
}

void uart1_read(uint32_t offset, uint8_t *value) {

	switch(offset){
	case 0b000:
		(uart_com_1.lcr & (1 << 7 /*Divisor Enable*/) ? (*value= uart_com_1.dll) : (*value= uart_com_1.rhr));

		uart_com_1.lsr &= ~1;

		break;
	case 0b001:
		(uart_com_1.lcr & (1 << 7 /*Divisor Enable*/) ? (*value= uart_com_1.dlm) : (*value= uart_com_1.ier));
		break;
	case 0b010:
		*value= uart_com_1.isr;
		break;
	case 0b011:
		*value= uart_com_1.lcr;
		break;
	case 0b100:
		*value= uart_com_1.mcr;
		break;
	case 0b101:
		*value= uart_com_1.lsr;
		break;
	case 0b110:
		*value= uart_com_1.msr;
		break;
	case 0b111:
		*value= uart_com_1.spr;
		break;
	default:
		assert(0);
	}
}

void virtio_notified(){

	if (virtio1_console.queue[1 /*transmitq*/].notify) {
		struct virtio_queue_t *virtio_queue = &virtio1_console.queue[1];

		virtio_queue->notify = 0;

		struct virtq_avail_t virtq_avail;
		stream_lru_read(virtio_queue->driver_addr - RAM_BASE, sizeof(struct virtq_avail_t), (uint8_t*) &virtq_avail);

		struct virtq_used_t virtq_used;
		stream_lru_read(virtio_queue->dev_addr - RAM_BASE, sizeof(struct virtq_used_t), (uint8_t*) &virtq_used);

		while (virtq_used.idx != virtq_avail.idx) {

			struct virtq_desc_t virtq_desc;
			stream_lru_read( virtio_queue->desc_addr + virtq_avail.ring[virtq_used.idx % virtio_queue->num] * sizeof(struct virtq_desc_t) - RAM_BASE, sizeof(struct virtq_desc_t), (uint8_t*) &virtq_desc);

			// ######################################################################
	//		assert(~virtq_desc.flags & (2 /*VRING_DESC_F_WRITE*/));

			uint8_t buff[virtq_desc.len];
			stream_lru_read(virtq_desc.addr - RAM_BASE, sizeof(buff), (uint8_t*) &buff);

			fwrite(&buff, sizeof(uint8_t), sizeof(buff), stdout);
			// ######################################################################

			virtq_used.ring[virtq_used.idx % virtio_queue->num].id = virtq_avail.ring[virtq_used.idx % virtio_queue->num];
			virtq_used.ring[virtq_used.idx % virtio_queue->num].len = virtq_desc.len;

			++virtq_used.idx;
		}
		stream_lru_write(virtio_queue->dev_addr - RAM_BASE, sizeof(struct virtq_used_t), (uint8_t*) &virtq_used);
//
//		assert(~virtq_avail.flags & (1 /*VIRTQ_AVAIL_F_NO_INTERRUPT*/));
//
//		plic.sclaim |= (1 << 0x01 /*console's irq*/);
//		csr.sip |= (1 << SUPERVISOR_EXTERNAL_INTERRUPT);
//
//		virtio1_console.int_status |= 1;
	}

	// #################################################
	if (virtio3_gpio.queue[0 /*requestq*/].notify) {
		struct virtio_queue_t *virtio_queue = &virtio3_gpio.queue[0];

		virtio_queue->notify = 0;

		struct virtq_avail_t virtq_avail;
		stream_lru_read(virtio_queue->driver_addr - RAM_BASE, sizeof(struct virtq_avail_t), (uint8_t*) &virtq_avail);

		struct virtq_used_t virtq_used;
		stream_lru_read(virtio_queue->dev_addr - RAM_BASE, sizeof(struct virtq_used_t), (uint8_t*) &virtq_used);

		struct virtq_desc_t virtq_desc;
		stream_lru_read( virtio_queue->desc_addr + virtq_avail.ring[virtq_used.idx % virtio_queue->num] * sizeof(struct virtq_desc_t) - RAM_BASE, sizeof(struct virtq_desc_t), (uint8_t*) &virtq_desc);

		struct virtq_desc_t virtq_desc_1;
		stream_lru_read( virtio_queue->desc_addr + virtq_desc.next * sizeof(struct virtq_desc_t) - RAM_BASE, sizeof(struct virtq_desc_t), (uint8_t*) &virtq_desc_1);
		// ######################################################################

//		assert(~virtq_desc.flags & (2 /*VRING_DESC_F_WRITE*/));

		struct virtio_gpio_resp_t {
			uint8_t status;
			uint8_t value;
		};

		struct virtio_gpio_resp_t virtio_gpio_resp= {
				.status= (0x01 /* VIRTIO_GPIO_STATUS_ERR*/),
				.value= 0x00
		};

		struct virtio_gpio_req_t {
			uint16_t type;
			uint16_t gpio;
			uint32_t value;
		} virtio_gpio_req;

		stream_lru_read(virtq_desc.addr - RAM_BASE, sizeof(struct virtio_gpio_req_t), (uint8_t*) &virtio_gpio_req);

		do {

			gpio_num_t gpio_num= -1;

			if (virtio_gpio_req.gpio == 0) {
				gpio_num = GPIO_NUM_21;

			} else if (virtio_gpio_req.gpio == 1) {
//				gpio_num = GPIO_NUM_22;

			} else break;

			// ---
			if (virtio_gpio_req.type == 0x002 /*VIRTIO_GPIO_MSG_GET_DIRECTION*/) {
				// none...

			} else if (virtio_gpio_req.type == 0x003 /*VIRTIO_GPIO_MSG_SET_DIRECTION*/) {

				switch (virtio_gpio_req.value) {
				case 0x01:
					gpio_set_direction(gpio_num, GPIO_MODE_OUTPUT);
					break;
				case 0x02:
					gpio_set_direction(gpio_num, GPIO_MODE_INPUT);
					break;
				default:
					gpio_set_direction(gpio_num, GPIO_MODE_DISABLE);
				}

				gpio_set_pull_mode(gpio_num, GPIO_FLOATING);

			} else if (virtio_gpio_req.type == 0x004 /*VIRTIO_GPIO_MSG_GET_VALUE*/) {
				virtio_gpio_resp.value = gpio_get_level(gpio_num);

			} else if (virtio_gpio_req.type == 0x005 /*VIRTIO_GPIO_MSG_SET_VALUE*/) {
				gpio_set_level(gpio_num, virtio_gpio_req.value);

			} else break;

			virtio_gpio_resp.status = (0x00 /*VIRTIO_GPIO_STATUS_OK*/);

		} while (0);

		stream_lru_write(virtq_desc_1.addr - RAM_BASE, sizeof(struct virtio_gpio_resp_t), (uint8_t*) &virtio_gpio_resp);
		// ######################################################################

		virtq_used.ring[virtq_used.idx % virtio_queue->num].id = virtq_avail.ring[virtq_used.idx % virtio_queue->num];
		virtq_used.ring[virtq_used.idx % virtio_queue->num].len = sizeof(struct virtio_gpio_resp_t);
		++virtq_used.idx;

		stream_lru_write(virtio_queue->dev_addr - RAM_BASE, sizeof(struct virtq_used_t), (uint8_t*) &virtq_used);

//		assert(~virtq_avail.flags & (1 /*VIRTQ_AVAIL_F_NO_INTERRUPT*/));

		plic.sclaim |= (1 << 0x03 /*gpio's irq*/);
		csr.sip |= (1 << SUPERVISOR_EXTERNAL_INTERRUPT);

		virtio3_gpio.int_status |= 1;
	}

	// #################################################
	if (virtio2_blk.queue[0 /*requestq*/].notify) {
		struct virtio_queue_t *virtio_queue = &virtio2_blk.queue[0];

		virtio_queue->notify = 0;

		struct virtq_avail_t virtq_avail;
		stream_lru_read(virtio_queue->driver_addr - RAM_BASE, sizeof(struct virtq_avail_t), (uint8_t*) &virtq_avail);

		struct virtq_used_t virtq_used;
		stream_lru_read(virtio_queue->dev_addr - RAM_BASE, sizeof(struct virtq_used_t), (uint8_t*) &virtq_used);

		while (virtq_used.idx != virtq_avail.idx) {

			struct virtq_desc_t virtq_desc;
			stream_lru_read( virtio_queue->desc_addr + virtq_avail.ring[virtq_used.idx % virtio_queue->num] * sizeof(struct virtq_desc_t) - RAM_BASE, sizeof(struct virtq_desc_t), (uint8_t*) &virtq_desc);

			struct virtq_desc_t virtq_desc_1;
			stream_lru_read( virtio_queue->desc_addr + virtq_desc.next * sizeof(struct virtq_desc_t) - RAM_BASE, sizeof(struct virtq_desc_t), (uint8_t*) &virtq_desc_1);

			struct virtq_desc_t virtq_desc_2;
			stream_lru_read( virtio_queue->desc_addr + virtq_desc_1.next * sizeof(struct virtq_desc_t) - RAM_BASE, sizeof(struct virtq_desc_t), (uint8_t*) &virtq_desc_2);

			struct virtio_blk_req_t {
				uint32_t type;
				uint32_t unused0;
				uint64_t sector;

			} virtio_blk_req;

			stream_lru_read(virtq_desc.addr - RAM_BASE, sizeof(struct virtio_blk_req_t), (uint8_t*) &virtio_blk_req);

			fseek(rootfs, virtio_blk_req.sector * 512, SEEK_SET);

			uint8_t buff[1024];
			switch(virtq_desc_1.flags & (2 /*VRING_DESC_F_WRITE*/)){
			case 0:
//				assert(virtio_blk_req.type == (1 /*VIRTIO_BLK_T_OUT*/));

				for(uint32_t i= 0; i < virtq_desc_1.len; i += MIN(virtq_desc_1.len - i, sizeof(buff)) ){

					stream_lru_read(virtq_desc_1.addr + i - RAM_BASE, MIN(virtq_desc_1.len - i, sizeof(buff)), (uint8_t*) &buff);
					fwrite(&buff, sizeof(uint8_t), MIN(virtq_desc_1.len - i, sizeof(buff)), rootfs);
				}

				break;
			case 2:
//				assert(virtio_blk_req.type == (0 /*VIRTIO_BLK_T_IN*/));

				for(uint32_t i= 0; i < virtq_desc_1.len; i += MIN(virtq_desc_1.len - i, sizeof(buff)) ){

					fread(&buff, sizeof(uint8_t), MIN(virtq_desc_1.len - i, sizeof(buff)), rootfs);
					stream_lru_write(virtq_desc_1.addr + i - RAM_BASE, MIN(virtq_desc_1.len - i, sizeof(buff)), (uint8_t*) &buff);
				}
			}

			uint8_t status[1] = { 0x00 /*VIRTIO_BLK_S_OK*/};
			stream_lru_write(virtq_desc_2.addr - RAM_BASE, 1, (uint8_t*) &status);
			// ######################################################################

			virtq_used.ring[virtq_used.idx % virtio_queue->num].id = virtq_avail.ring[virtq_used.idx % virtio_queue->num];
			virtq_used.ring[virtq_used.idx % virtio_queue->num].len = virtq_desc_1.len;

			++virtq_used.idx;
		}

		stream_lru_write(virtio_queue->dev_addr - RAM_BASE, sizeof(struct virtq_used_t), (uint8_t*) &virtq_used);

//		assert(~virtq_avail.flags & (1 /*VIRTQ_AVAIL_F_NO_INTERRUPT*/));

		plic.sclaim |= (1 << 0x02 /*block's irq*/);
		csr.sip |= (1 << SUPERVISOR_EXTERNAL_INTERRUPT);

		virtio2_blk.int_status |= 1;
	}

	// #################################################
	if (virtio8_net.queue[1 /*transmitq1*/].notify) {
		struct virtio_queue_t *virtio_queue = &virtio8_net.queue[1];

		virtio_queue->notify = 0;

		struct virtq_avail_t virtq_avail;
		stream_lru_read(virtio_queue->driver_addr - RAM_BASE, sizeof(struct virtq_avail_t), (uint8_t*) &virtq_avail);

		struct virtq_used_t virtq_used;
		stream_lru_read(virtio_queue->dev_addr - RAM_BASE, sizeof(struct virtq_used_t), (uint8_t*) &virtq_used);

		struct virtio_net_hdr_t {
			uint8_t flags;
			uint8_t gso_type;
			uint16_t hdr_len;
			uint16_t gso_size;
			uint16_t csum_start;
			uint16_t csum_offset;
			uint16_t num_buffers;
		};

		while (virtq_used.idx != virtq_avail.idx) {

			struct virtq_desc_t virtq_desc;
			stream_lru_read( virtio_queue->desc_addr + virtq_avail.ring[virtq_used.idx % virtio_queue->num] * sizeof(struct virtq_desc_t) - RAM_BASE, sizeof(struct virtq_desc_t), (uint8_t*) &virtq_desc);

			// ######################################################################
//			assert(~virtq_desc.flags & (2 /*VRING_DESC_F_WRITE*/));

			struct virtio_net_packet_t {
				struct virtio_net_hdr_t hdr;
				uint8_t data[virtq_desc.len - sizeof(struct virtio_net_hdr_t)];
			} virtio_net_packet;

			stream_lru_read(virtq_desc.addr - RAM_BASE, sizeof(struct virtio_net_packet_t), (uint8_t*) &virtio_net_packet);

			esp_wifi_internal_tx(WIFI_IF_STA, &virtio_net_packet.data, sizeof(virtio_net_packet.data));

			// ######################################################################
			virtq_used.ring[virtq_used.idx % virtio_queue->num].id = virtq_avail.ring[virtq_used.idx % virtio_queue->num];
			virtq_used.ring[virtq_used.idx % virtio_queue->num].len = virtq_desc.len;

			++virtq_used.idx;
		}

		stream_lru_write(virtio_queue->dev_addr - RAM_BASE, sizeof(struct virtq_used_t), (uint8_t*) &virtq_used);
//
//		assert(~virtq_avail.flags & (1 /*VIRTQ_AVAIL_F_NO_INTERRUPT*/));

//		plic.sclaim |= (1 << 0x08 /*net's irq*/);
//		csr.sip |= (1 << SUPERVISOR_EXTERNAL_INTERRUPT);
//
//		virtio8_net.int_status |= 1;
	}
}

void virtio_read(uint32_t offset, uint8_t size, uint32_t *value, struct virtio_t *virtio) {

	switch (offset) {
	case 0x000: /*Magic value - R*/
		*value = 0x74726976;
		break;
	case 0x004: /*Device version number - R*/
		*value = 2;
		break;
	case 0x008: /*Virtio Subsystem Device ID - R*/
		*value = virtio->dev_id;
		break;
	case 0x010: /*Flags representing features the device supports - R*/
		if (virtio->dev_feat_sel == 0){
			*value = virtio->dev_feat;
		} else
			*value = *((uint32_t*) &virtio->dev_feat + 1);
		break;
	case 0x00c: /*Virtio Subsystem Vendor ID - R*/
		*value = 0x0000ffff;
		break;
	case 0x034: /*Maximum virtual queue size - R*/
		*value = MAX_VQUEUE_SIZE;
		break;
	case 0x044:
		*value = virtio->queue[virtio->queue_sel].ready;
		break;
	case 0x060: /*Interrupt status - R*/
		*value = virtio->int_status;
		break;
	case 0x070: /*Device status - RW*/
		*value = virtio->dev_status;
		break;
	case 0x0fc: /*Configuration atomicity value - R*/
		*value = 0;
		break;
	default:

		if (offset >= 0x100) {
			memcpy(value, virtio->config_space + (offset - 0x100), size);
			return;
		}

		assert(0); // ...virtio address not found
	}
}

void virtio_write(uint32_t offset, uint8_t size, uint32_t value, struct virtio_t *virtio) {

	switch (offset) {
	case 0x014: /*Device (host) features word selection - W*/
		virtio->dev_feat_sel = value;
		break;
	case 0x020: /*Flags representing device features understood and activated by the driver - W*/

		if (virtio->guest_feat_sel == 0) {
			*(uint32_t*) &virtio->guest_feat = value;
		} else
			*((uint32_t*) &virtio->guest_feat + 1)= value;

		break;
	case 0x024: /*Activated (guest) features word selection - W*/
		virtio->guest_feat_sel = value;
		break;
	case 0x030: /*Virtual queue index - W*/
//		assert(value < (sizeof(virtio->queue)/sizeof(struct virtio_queue_t)));

		virtio->queue_sel = value;
		break;
	case 0x038: /*Virtual queue size - W*/
		virtio->queue[virtio->queue_sel].num = value;
		break;
	case 0x044:
		virtio->queue[virtio->queue_sel].ready = value;
		break;
	case 0x050: /*Queue notifier - W*/
		virtio->queue[value].notify = 1;

		/* This function was previously located in the main loop to continuously monitor events.
		 * However, for performance improvement, the decision was made to move this function to the specific event handler.
		 * */
		virtio_notified();
		break;
	case 0x064: /*Interrupt acknowledge - W*/
		virtio->int_status &= ~value;
		break;
	case 0x070: /*Device status - RW*/

//		assert(!(value == 0 && virtio->dev_status != 0));

		// #ACKNOWLEDGE (1 << 0)
		// #DRIVER (1 << 1)
		// #DRIVER_OK (1 << 3)
		// #FEATURES_OK (1 << 4)
		// #DEVICE_NEEDS_RESET (1 << 7)
		// #FAILED (1 << 8)

		virtio->dev_status = value;

		break;
	case 0x080:
		*(uint32_t*) &virtio->queue[virtio->queue_sel].desc_addr = value;
		break;
	case 0x084:
		*((uint32_t*) &virtio->queue[virtio->queue_sel].desc_addr + 1)= value;
		break;
	case 0x090:
		*(uint32_t*) &virtio->queue[virtio->queue_sel].driver_addr = value;
		break;
	case 0x094:
		*((uint32_t*) &virtio->queue[virtio->queue_sel].driver_addr + 1)= value;
		break;
	case 0x0a0:
		*(uint32_t*) &virtio->queue[virtio->queue_sel].dev_addr = value;
		break;
	case 0x0a4:
		*((uint32_t*) &virtio->queue[virtio->queue_sel].dev_addr + 1)= value;
		break;
	default:
		assert(0); // ...virtio address not found
	}
}

void bus_read(uint32_t addr, uint8_t size, uint8_t *value) {

	if (addr >= RAM_BASE) {
		stream_lru_read(addr - RAM_BASE, size, value);
		return;
	}

	if (addr >= CLINT_BASE && addr <= (CLINT_BASE + 0xc0000)) {
		clint_read(addr - CLINT_BASE, (uint32_t*) value);
		return;
	}

	if (addr >= PLIC_BASE && addr <= (PLIC_BASE + 0x400000)) {
		plic_read(addr - PLIC_BASE, (uint32_t*) value);
		return;
	}

	if (addr >= VIRTIO8_BASE && addr <= (VIRTIO8_BASE + 0x1000)) {
		virtio_read(addr - VIRTIO8_BASE, size, (uint32_t*) value, &virtio8_net);
		return;
	}

	if (addr >= UART1_BASE && addr <= (UART1_BASE + 0x100)) {
		uart1_read(addr - UART1_BASE, value);
		return;
	}

	if (addr >= VIRTIO4_BASE && addr <= (VIRTIO4_BASE + 0x1000)) {
		virtio_read(addr - VIRTIO4_BASE, size, (uint32_t*) value, &virtio4_i2c);
		return;
	}

	if (addr >= SYSCON_BASE && addr <= (SYSCON_BASE + 0x1000)) {
		*((uint32_t*) value) = 0x00000000;
		return;
	}

	if (addr >= VIRTIO3_BASE && addr <= (VIRTIO3_BASE + 0x1000)) {
		virtio_read(addr - VIRTIO3_BASE, size, (uint32_t*) value, &virtio3_gpio);
		return;
	}

	if (addr >= VIRTIO2_BASE && addr <= (VIRTIO2_BASE + 0x1000)) {
		virtio_read(addr - VIRTIO2_BASE, size, (uint32_t*) value, &virtio2_blk);
		return;
	}

	if (addr >= VIRTIO1_BASE && addr <= (VIRTIO1_BASE + 0x1000)) {
		virtio_read(addr - VIRTIO1_BASE, size, (uint32_t*) value, &virtio1_console);
		return;
	}

	if (addr >= DTB_BASE && addr <= (DTB_BASE + 4096 /*4Kb*/)) {
		memcpy((uint8_t*) value, (uint8_t*) (low_memory + (addr - DTB_BASE)), size);
		return;
	}

	cpu_take_trap(LOAD_ACCESS_FAULT, addr);
}

void bus_write(uint32_t addr, uint8_t size, uint64_t value) {

	if (addr >= RAM_BASE) {
		stream_lru_write(addr - RAM_BASE, size, (uint8_t*) &value);
		return;
	}

	if (addr >= CLINT_BASE && addr <= (CLINT_BASE + 0xc0000)) {
		clint_write(addr - CLINT_BASE, value);
		return;
	}

	if (addr >= PLIC_BASE && addr <= (PLIC_BASE + 0x400000)) {
		plic_write(addr - PLIC_BASE, value);
		return;
	}

	if (addr >= VIRTIO8_BASE && addr <= (VIRTIO8_BASE + 0x1000)) {
		virtio_write(addr - VIRTIO8_BASE, size, value, &virtio8_net);
		return;
	}

	if (addr >= UART1_BASE && addr <= (UART1_BASE + 0x100)) {
		uart1_write(addr - UART1_BASE, value);
		return;
	}

	if (addr >= VIRTIO4_BASE && addr <= (VIRTIO4_BASE + 0x1000)) {
		virtio_write(addr - VIRTIO4_BASE, size, value, &virtio4_i2c);
		return;
	}

	if (addr >= SYSCON_BASE && addr <= (SYSCON_BASE + 0x1000)) {

		switch(addr - SYSCON_BASE){
		case 0x00 /*offset*/:
			if (value == 0x00007777 /*syscon-reboot*/
					|| value == 0x00005555 /*syscon-poweroff*/) {
				esp_restart();
			}
		}

		return;
	}

	if (addr >= VIRTIO3_BASE && addr <= (VIRTIO3_BASE + 0x1000)) {
		virtio_write(addr - VIRTIO3_BASE, size, value, &virtio3_gpio);
		return;
	}

	if (addr >= VIRTIO2_BASE && addr <= (VIRTIO2_BASE + 0x1000)) {
		virtio_write(addr - VIRTIO2_BASE, size, value, &virtio2_blk);
		return;
	}

	if (addr >= VIRTIO1_BASE && addr <= (VIRTIO1_BASE + 0x1000)) {
		virtio_write(addr - VIRTIO1_BASE, size, value, &virtio1_console);
		return;
	}

	if (addr >= DTB_BASE && addr <= (DTB_BASE + 4096 /*4Kb*/)) {
		memcpy((uint8_t*) (low_memory + (addr - DTB_BASE)), (uint8_t*) &value, size);
		return;
	}

	cpu_take_trap(STORE_AMO_ACCESS_FAULT, addr);
}

uint64_t pack754_64(double d){ return *((uint64_t*) &d); }
double unpack754_64(uint64_t ieee754){ return *((double*) &ieee754); }

uint32_t pack754_32(float f){ return *(uint32_t*) &f; }
float unpack754_32(uint32_t ieee754){ return *(float*) &ieee754; }

void cpu_execute(uint32_t inst) {
	/*RV32I RV32M RV32A RV32F RV32C*/

	cpu.xreg[0] = 0;

	if ((inst & 3) == 3) {

		if ((inst & 0b00000000000000000111000001111111) == 0b00000000000000000010000000000011) {
			/*lw*/
			uint32_t imm = /*imm[11:0]*/(int32_t) inst >> 20;

			uint32_t addr= cpu.xreg[(inst >> 15) & 0b11111 /*rs1*/] + imm;
			if (cpu_addr_translate(&addr, LOAD_PAGE_FAULT)) {
				cpu.pc += 4;

				bus_read(addr, 4, (uint8_t*) &cpu.xreg[(inst >> 7) & 0b11111 /*rd*/]);
			}

			return;
		}
		if ((inst & 0b00000000000000000111000001111111) == 0b00000000000000000010000000100011) {
			/*sw*/
			uint32_t imm = /*imm[11:5] imm[4:0]*/
					((inst & 0b11111110000000000000000000000000) >> 20) |
					((inst & 0b00000000000000000000111110000000) >> 7);

			uint32_t addr= cpu.xreg[(inst >> 15) & 0b11111 /*rs1*/] + ((int32_t) (imm << 20) >> 20);
			if (cpu_addr_translate(&addr, STORE_AMO_PAGE_FAULT)) {
				cpu.pc += 4;

				bus_write(addr, 4, cpu.xreg[(inst >> 20) & 0b11111 /*rs2*/]);
			}

			return;
		}
		if ((inst & 0b00000000000000000111000001111111) == 0b00000000000000000000000000010011) {
			/*addi*/
			cpu.pc += 4;

			uint32_t imm = /*imm[11:0]*/(int32_t) inst >> 20;
			cpu.xreg[(inst >> 7) & 0b11111 /*rd*/] = cpu.xreg[(inst >> 15) & 0b11111 /*rs1*/] + imm;
			return;
		}
		if ((inst & 0b00000000000000000111000001111111) == 0b00000000000000000001000001100011) {
			/*bne*/

			if (cpu.xreg[(inst >> 15) & 0b11111 /*rs1*/] != cpu.xreg[(inst >> 20) & 0b11111 /*rs2*/]) {

				uint32_t imm = /*imm[12|10:5] imm[4:1|11]*/
						((inst & 0b10000000000000000000000000000000) >> 19) |
						((inst & 0b01111110000000000000000000000000) >> 20) |
						((inst & 0b00000000000000000000111100000000) >> 7) |
						((inst & 0b00000000000000000000000010000000) << 4);

				cpu.pc += ((int32_t) (imm << 19) >> 19);

			} else cpu.pc += 4;

			return;
		}
		if ((inst & 0b00000000000000000111000001111111) == 0b00000000000000000100000000000011) {
			/*lbu*/
			uint32_t imm = /*imm[11:0]*/(int32_t) inst >> 20;

			uint32_t addr= cpu.xreg[(inst >> 15) & 0b11111 /*rs1*/] + imm;
			if (cpu_addr_translate(&addr, LOAD_PAGE_FAULT)) {
				cpu.pc += 4;

				uint8_t value;
				bus_read(addr, 1, &value);

				cpu.xreg[(inst >> 7) & 0b11111 /*rd*/] = value;
			}

			return;
		}
		if ((inst & 0b00000000000000000000000001111111) == 0b00000000000000000000000000110111) {
			/*lui*/
			cpu.pc += 4;

			uint32_t imm = /*imm[31:12]*/inst & 0xfffff000;
			cpu.xreg[(inst >> 7) & 0b11111 /*rd*/] = imm;
			return;
		}
		if ((inst & 0b00000000000000000000000001111111) == 0b00000000000000000000000001101111) {
			/*jal*/

			uint32_t imm = /*imm[20|10:1|11|19:12]*/
					((inst & 0b10000000000000000000000000000000) >> 11) |
					((inst & 0b01111111111000000000000000000000) >> 20) |
					((inst & 0b00000000000100000000000000000000) >> 9) |
					(inst & 0b00000000000011111111000000000000);

			cpu.xreg[(inst >> 7) & 0b11111 /*rd*/] = cpu.pc + 4;
			cpu.pc += ((int32_t) (imm << 11) >> 11);
			return;
		}
		if ((inst & 0b00000000000000000111000001111111) == 0b00000000000000000000000001100011) {
			/*beq*/
			if (cpu.xreg[(inst >> 15) & 0b11111 /*rs1*/] == cpu.xreg[(inst >> 20) & 0b11111 /*rs2*/]) {

				uint32_t imm = /*imm[12|10:5] imm[4:1|11]*/
						((inst & 0b10000000000000000000000000000000) >> 19) |
						((inst & 0b01111110000000000000000000000000) >> 20) |
						((inst & 0b00000000000000000000111100000000) >> 7) |
						((inst & 0b00000000000000000000000010000000) << 4);

				cpu.pc += ((int32_t) (imm << 19) >> 19);
			} else cpu.pc += 4;

			return;
		}
		if ((inst & 0b00000000000000000111000001111111) == 0b00000000000000000111000000010011) {
			/*andi*/
			cpu.pc += 4;

			uint32_t imm = /*imm[11:0]*/(int32_t) inst >> 20;
			cpu.xreg[(inst >> 7) & 0b11111 /*rd*/] = cpu.xreg[(inst >> 15) & 0b11111 /*rs1*/] & imm;
			return;
		}
		if ((inst & 0b00000000000000000111000001111111) == 0b00000000000000000110000001100011) {
			/*bltu*/
			if (cpu.xreg[(inst >> 15) & 0b11111 /*rs1*/] < cpu.xreg[(inst >> 20) & 0b11111 /*rs2*/]) {

				uint32_t imm = /*imm[12|10:5] imm[4:1|11]*/
						((inst & 0b10000000000000000000000000000000) >> 19) |
						((inst & 0b01111110000000000000000000000000) >> 20) |
						((inst & 0b00000000000000000000111100000000) >> 7) |
						((inst & 0b00000000000000000000000010000000) << 4);

				cpu.pc += ((int32_t) (imm << 19) >> 19);

			} else cpu.pc += 4;

			return;
		}
		if ((inst & 0b11111110000000000111000001111111) == 0b01000000000000000000000000110011) {
			/*sub*/
			cpu.pc += 4;

			cpu.xreg[(inst >> 7) & 0b11111 /*rd*/] = cpu.xreg[(inst >> 15) & 0b11111 /*rs1*/] - cpu.xreg[(inst >> 20) & 0b11111 /*rs2*/];
			return;
		}
		if ((inst & 0b11111110000000000111000001111111) == 0b00000000000000000000000000110011) {
			/*add*/
			cpu.pc += 4;

			cpu.xreg[(inst >> 7) & 0b11111 /*rd*/] = cpu.xreg[(inst >> 15) & 0b11111 /*rs1*/] + cpu.xreg[(inst >> 20) & 0b11111 /*rs2*/];
			return;
		}
		if ((inst & 0b11111000000000000111000001111111) == 0b00000000000000000001000000010011) {
			/*slli*/
			cpu.pc += 4;

			uint32_t imm = /*imm[4:0]*/(inst & 0x1f00000) >> 20;
			cpu.xreg[(inst >> 7) & 0b11111 /*rd*/] = cpu.xreg[(inst >> 15) & 0b11111 /*rs1*/] << imm;
			return;
		}
		if ((inst & 0b00000000000000000111000001111111) == 0b00000000000000000111000001100011) {
			/*bgeu*/
			if (cpu.xreg[(inst >> 15) & 0b11111 /*rs1*/] >= cpu.xreg[(inst >> 20) & 0b11111 /*rs2*/]) {

				uint32_t imm = /*imm[12|10:5] imm[4:1|11]*/
						((inst & 0b10000000000000000000000000000000) >> 19) |
						((inst & 0b01111110000000000000000000000000) >> 20) |
						((inst & 0b00000000000000000000111100000000) >> 7) |
						((inst & 0b00000000000000000000000010000000) << 4);

				cpu.pc += ((int32_t) (imm << 19) >> 19);

			} else cpu.pc += 4;

			return;
		}

		if ((inst & 0b11111110000000000111000001111111) == 0b00000000000000000011000000110011) {
			/*sltu*/
			cpu.pc += 4;

			cpu.xreg[(inst >> 7) & 0b11111 /*rd*/] = cpu.xreg[(inst >> 15) & 0b11111 /*rs1*/] < cpu.xreg[(inst >> 20) & 0b11111 /*rs2*/];
			return;
		}
		if ((inst & 0b11111000000000000111000001111111) == 0b00000000000000000101000000010011) {
			/*srli*/
			cpu.pc += 4;

			uint32_t imm = /*imm[4:0]*/(inst & 0x1f00000) >> 20;
			cpu.xreg[(inst >> 7) & 0b11111 /*rd*/] = cpu.xreg[(inst >> 15) & 0b11111 /*rs1*/] >> imm;
			return;
		}
		if ((inst & 0b00000000000000000111000001111111) == 0b00000000000000000000000000100011) {
			/*sb*/

			uint32_t imm = /*imm[11:5] imm[4:0]*/
					((inst & 0b11111110000000000000000000000000) >> 20) |
					((inst & 0b00000000000000000000111110000000) >> 7);

			uint32_t addr= cpu.xreg[(inst >> 15) & 0b11111 /*rs1*/] + ((int32_t) (imm << 20) >> 20);
			if (cpu_addr_translate(&addr, STORE_AMO_PAGE_FAULT)) {
				cpu.pc += 4;

				bus_write(addr, 1, cpu.xreg[(inst >> 20) & 0b11111 /*rs2*/]);
			}

			return;
		}
		if ((inst & 0b00000000000000000000000001111111) == 0b00000000000000000000000000010111) {
			/*auipc*/

			uint32_t imm = /*imm[31:12]*/inst & 0xfffff000;
			cpu.xreg[(inst >> 7) & 0b11111 /*rd*/] = cpu.pc + imm;

			cpu.pc += 4;

			return;
		}
		if ((inst & 0b00000000000000000111000001111111) == 0b00000000000000000100000001100011) {
			/*blt*/
			if ((int32_t) cpu.xreg[(inst >> 15) & 0b11111 /*rs1*/] < (int32_t) cpu.xreg[(inst >> 20) & 0b11111 /*rs2*/]) {

				uint32_t imm = /*imm[12|10:5] imm[4:1|11]*/
						((inst & 0b10000000000000000000000000000000) >> 19) |
						((inst & 0b01111110000000000000000000000000) >> 20) |
						((inst & 0b00000000000000000000111100000000) >> 7) |
						((inst & 0b00000000000000000000000010000000) << 4);

				cpu.pc += ((int32_t) (imm << 19) >> 19);

			} else cpu.pc += 4;

			return;
		}
		if ((inst & 0b00000000000000000111000001111111) == 0b00000000000000000000000001100111) {
			/*jalr*/
			uint32_t imm = /*imm[11:0]*/(int32_t) inst >> 20;

			uint32_t tmp = cpu.pc + 4;
			cpu.pc = (cpu.xreg[(inst >> 15) & 0b11111 /*rs1*/] + imm) & ~1;
			cpu.xreg[(inst >> 7) & 0b11111 /*rd*/] = tmp;

			return;
		}
		if ((inst & 0b00000000000000000111000001111111) == 0b00000000000000000100000000010011) {
			/*xori*/
			cpu.pc += 4;

			uint32_t imm = /*imm[11:0]*/(int32_t) inst >> 20;
			cpu.xreg[(inst >> 7) & 0b11111 /*rd*/] = cpu.xreg[(inst >> 15) & 0b11111 /*rs1*/] ^ imm;
			return;
		}
		if ((inst & 0b00000000000000000111000001111111) == 0b00000000000000000101000001100011) {
			/*bge*/

			if ((int32_t) cpu.xreg[(inst >> 15) & 0b11111 /*rs1*/] >= (int32_t) cpu.xreg[(inst >> 20) & 0b11111 /*rs2*/]) {

				uint32_t imm = /*imm[12|10:5] imm[4:1|11]*/
						((inst & 0b10000000000000000000000000000000) >> 19) |
						((inst & 0b01111110000000000000000000000000) >> 20) |
						((inst & 0b00000000000000000000111100000000) >> 7) |
						((inst & 0b00000000000000000000000010000000) << 4);

				cpu.pc += ((int32_t) (imm << 19) >> 19);

			} else cpu.pc += 4;

			return;
		}
		if ((inst & 0b11111110000000000111000001111111) == 0b00000000000000000110000000110011) {
			/*or*/
			cpu.pc += 4;

			cpu.xreg[(inst >> 7) & 0b11111 /*rd*/] = cpu.xreg[(inst >> 15) & 0b11111 /*rs1*/] | cpu.xreg[(inst >> 20) & 0b11111 /*rs2*/];
			return;
		}
		if ((inst & 0b11111110000000000111000001111111) == 0b00000010000000000000000000110011) {
			/*mul*/
			cpu.pc += 4;

			cpu.xreg[(inst >> 7) & 0b11111 /*rd*/] = (int32_t) cpu.xreg[(inst >> 15) & 0b11111 /*rs1*/] * (int32_t) cpu.xreg[(inst >> 20) & 0b11111 /*rs2*/];
			return;
		}
		if ((inst & 0b11111110000000000111000001111111) == 0b00000000000000000100000000110011) {
			/*xor*/
			cpu.pc += 4;

			cpu.xreg[(inst >> 7) & 0b11111 /*rd*/] = cpu.xreg[(inst >> 15) & 0b11111 /*rs1*/] ^ cpu.xreg[(inst >> 20) & 0b11111 /*rs2*/];
			return;
		}
		if ((inst & 0b00000000000000000111000001111111) == 0b00000000000000000111000001110011) {
			/*csrrci*/
			cpu.pc += 4;

			uint32_t imm = /*imm[4:0]*/((inst & 0b00000000000011111000000000000000) >> 15);

			uint32_t csr = /*csr[11:0]*/inst >> 20;

			uint32_t tmp;
			csr_read(csr, &tmp);

			csr_write(csr, tmp & ~imm);
			cpu.xreg[(inst >> 7) & 0b11111 /*rd*/] = tmp;
			return;
		}
		if ((inst & 0b00000000000000000111000001111111) == 0b00000000000000000101000000000011) {
			/*lhu*/
			uint32_t imm = /*imm[11:0]*/(int32_t) inst >> 20;

			uint32_t addr= cpu.xreg[(inst >> 15) & 0b11111 /*rs1*/] + imm;
			if (cpu_addr_translate(&addr, LOAD_PAGE_FAULT)) {
				cpu.pc += 4;

				uint16_t value;
				bus_read(addr, 2, (uint8_t*) &value);

				cpu.xreg[(inst >> 7) & 0b11111 /*rd*/]= value;
			}

			return;
		}
		if ((inst & 0b00000000000000000111000001111111) == 0b00000000000000000010000001110011) {
			/*csrrs*/
			cpu.pc += 4;

			uint32_t csr = /*csr[11:0]*/inst >> 20;

			uint32_t tmp;
			csr_read(csr, &tmp);

			csr_write(csr, tmp | cpu.xreg[(inst >> 15) & 0b11111 /*rs1*/]);
			cpu.xreg[(inst >> 7) & 0b11111 /*rd*/] = tmp;
			return;
		}
		if ((inst & 0b11111110000000000111000001111111) == 0b00000000000000000101000000110011) {
			/*srl*/
			cpu.pc += 4;

			cpu.xreg[(inst >> 7) & 0b11111 /*rd*/] = cpu.xreg[(inst >> 15) & 0b11111 /*rs1*/] >> (cpu.xreg[(inst >> 20) & 0b11111 /*rs2*/] & 0x1f);
			return;
		}
		if ((inst & 0b11111110000000000111000001111111) == 0b00000000000000000111000000110011) {
			/*and*/
			cpu.pc += 4;

			cpu.xreg[(inst >> 7) & 0b11111 /*rd*/] = cpu.xreg[(inst >> 15) & 0b11111 /*rs1*/] & cpu.xreg[(inst >> 20) & 0b11111 /*rs2*/];
			return;
		}
		if ((inst & 0b11111110000000000111000001111111) == 0b00000000000000000001000000110011) {
			/*sll*/
			cpu.pc += 4;

			cpu.xreg[(inst >> 7) & 0b11111 /*rd*/] = cpu.xreg[(inst >> 15) & 0b11111 /*rs1*/] << (cpu.xreg[(inst >> 20) & 0b11111 /*rs2*/] & 0x1f);
			return;
		}
		if ((inst & 0b11110000000011111111111111111111) == 0b00000000000000000000000000001111) {
			/*fence*/
			cpu.pc += 4;

			// uint8_t pred= (inst & 0b00001111000000000000000000000000) >> 24;
			// uint8_t succ= (inst & 0b00000000111100000000000000000000) >> 20;
			return;
		}
		if ((inst & 0b00000000000000000111000001111111) == 0b00000000000000000110000001110011) {
			/*csrrsi*/
			cpu.pc += 4;

			uint32_t imm = /*imm[4:0]*/((inst & 0b00000000000011111000000000000000) >> 15);

			uint32_t csr = /*csr[11:0]*/inst >> 20;

			uint32_t tmp;
			csr_read(csr, &tmp);

			csr_write(csr, tmp | imm);
			cpu.xreg[(inst >> 7) & 0b11111 /*rd*/] = tmp;
			return;
		}
		if ((inst & 0b11111000000000000111000001111111) == 0b00000000000000000010000000101111) {
			/*amoadd.w*/
			uint32_t addr= cpu.xreg[(inst >> 15) & 0b11111 /*rs1*/];
			if (cpu_addr_translate(&addr, STORE_AMO_PAGE_FAULT)) {
				cpu.pc += 4;

				uint32_t value;
				bus_read(addr, 4, (uint8_t*) &value);

				bus_write(addr, 4, value + cpu.xreg[(inst >> 20) & 0b11111 /*rs2*/]);
				cpu.xreg[(inst >> 7) & 0b11111 /*rd*/] = value;
			}

			return;
		}
		if ((inst & 0b00000000000000000111000001111111) == 0b00000000000000000000000000000011) {
			/*lb*/
			uint32_t imm = /*imm[11:0]*/(int32_t) inst >> 20;

			uint32_t addr= cpu.xreg[(inst >> 15) & 0b11111 /*rs1*/] + imm;
			if (cpu_addr_translate(&addr, LOAD_PAGE_FAULT)) {
				cpu.pc += 4;

				int8_t value;
				bus_read(addr, 1, (uint8_t*) &value);

				cpu.xreg[(inst >> 7) & 0b11111 /*rd*/] = value;
			}

			return;
		}
		if ((inst & 0b00000000000000000111000001111111) == 0b00000000000000000011000000010011) {
			/*sltiu*/
			cpu.pc += 4;

			uint32_t imm = /*imm[11:0]*/(int32_t) inst >> 20;
			cpu.xreg[(inst >> 7) & 0b11111 /*rd*/] = cpu.xreg[(inst >> 15) & 0b11111 /*rs1*/] < imm;
			return;
		}
		if ((inst & 0b00000000000000000111000001111111) == 0b00000000000000000001000000100011) {
			/*sh*/
			uint32_t imm = /*imm[11:5] imm[4:0]*/
					((inst & 0b11111110000000000000000000000000) >> 20) |
					((inst & 0b00000000000000000000111110000000) >> 7);

			uint32_t addr= cpu.xreg[(inst >> 15) & 0b11111 /*rs1*/] + ((int32_t) (imm << 20) >> 20);
			if (cpu_addr_translate(&addr, STORE_AMO_PAGE_FAULT)) {
				cpu.pc += 4;

				bus_write(addr, 2, cpu.xreg[(inst >> 20) & 0b11111 /*rs2*/]);
			}

			return;
		}
		if ((inst & 0b00000000000000000111000001111111) == 0b00000000000000000110000000010011) {
			/*ori*/
			cpu.pc += 4;

			uint32_t imm = /*imm[11:0]*/(int32_t) inst >> 20;
			cpu.xreg[(inst >> 7) & 0b11111 /*rd*/] = cpu.xreg[(inst >> 15) & 0b11111 /*rs1*/] | imm;
			return;
		}
		if ((inst & 0b11111110000000000111000001111111) == 0b00000010000000000011000000110011) {
			/*mulhu*/
			cpu.pc += 4;

			cpu.xreg[(inst >> 7) & 0b11111 /*rd*/] = ((uint64_t) cpu.xreg[(inst >> 15) & 0b11111 /*rs1*/] * (uint64_t) cpu.xreg[(inst >> 20) & 0b11111 /*rs2*/]) >> 32;
			return;
		}
		if ((inst & 0b11111000000000000111000001111111) == 0b01000000000000000101000000010011) {
			/*srai*/
			cpu.pc += 4;

			uint32_t imm = /*imm[4:0]*/(inst & 0x01f00000) >> 20;
			cpu.xreg[(inst >> 7) & 0b11111 /*rd*/] = (int32_t) cpu.xreg[(inst >> 15) & 0b11111 /*rs1*/] >> imm;
			return;
		}
		if ((inst & 0b11111000000000000111000001111111) == 0b00011000000000000010000000101111) {
			/*sc.w*/
			if (cpu.reserved == cpu.xreg[(inst >> 15) & 0b11111 /*rs1*/]) {

				uint32_t addr= cpu.xreg[(inst >> 15) & 0b11111 /*rs1*/];
				if (cpu_addr_translate(&addr, STORE_AMO_PAGE_FAULT)) {
					cpu.pc += 4;

					bus_write(addr, 4, cpu.xreg[(inst >> 20) & 0b11111 /*rs2*/]);
					cpu.xreg[(inst >> 7) & 0b11111 /*rd*/] = 0;
				}

			} else {

				cpu.pc += 4;

				cpu.xreg[(inst >> 7) & 0b11111 /*rd*/] = 1;
			}

			return;
		}
		if ((inst & 0b11111001111100000111000001111111) == 0b00010000000000000010000000101111) {
			/*lr.w*/
			uint32_t addr= cpu.reserved = cpu.xreg[(inst >> 15) & 0b11111 /*rs1*/];
			if (cpu_addr_translate(&addr, LOAD_PAGE_FAULT)) {
				cpu.pc += 4;

				bus_read(addr, 4, (uint8_t*) &cpu.xreg[(inst >> 7) & 0b11111 /*rd*/]);
			}

			return;
		}
		if ((inst & 0b11111000000000000111000001111111) == 0b01000000000000000010000000101111) {
			/*amoor.w*/
			uint32_t addr= cpu.xreg[(inst >> 15) & 0b11111 /*rs1*/];
			if (cpu_addr_translate(&addr, STORE_AMO_PAGE_FAULT)) {
				cpu.pc += 4;

				uint32_t value;
				bus_read(addr, 4, (uint8_t*) &value);

				bus_write(addr, 4, value | cpu.xreg[(inst >> 20) & 0b11111 /*rs2*/]);
				cpu.xreg[(inst >> 7) & 0b11111 /*rd*/] = value;
			}

			return;
		}
		if ((inst & 0b11111110000000000111000001111111) == 0b01000000000000000101000000110011) {
			/*sra*/
			cpu.pc += 4;

			cpu.xreg[(inst >> 7) & 0b11111 /*rd*/] = (int32_t) cpu.xreg[(inst >> 15) & 0b11111 /*rs1*/] >> (cpu.xreg[(inst >> 20) & 0b11111 /*rs2*/] & 0x1f);
			return;
		}
		if ((inst & 0b11111000000000000111000001111111) == 0b01100000000000000010000000101111) {
			/*amoand.w*/
			uint32_t addr= cpu.xreg[(inst >> 15) & 0b11111 /*rs1*/];
			if (cpu_addr_translate(&addr, STORE_AMO_PAGE_FAULT)) {
				cpu.pc += 4;

				uint32_t value;
				bus_read(addr, 4, (uint8_t*) &value);

				bus_write(addr, 4, value & cpu.xreg[(inst >> 20) & 0b11111 /*rs2*/]);
				cpu.xreg[(inst >> 7) & 0b11111 /*rd*/] = value;
			}

			return;
		}
		if ((inst & 0b00000000000000000111000001111111) == 0b00000000000000000001000000000011) {
			/*lh*/
			uint32_t imm = /*imm[11:0]*/(int32_t) inst >> 20;

			uint32_t addr= cpu.xreg[(inst >> 15) & 0b11111 /*rs1*/] + imm;
			if (cpu_addr_translate(&addr, LOAD_PAGE_FAULT)) {
				cpu.pc += 4;

				int16_t value;
				bus_read(addr, 2, (uint8_t*) &value);

				cpu.xreg[(inst >> 7) & 0b11111 /*rd*/] = value;
			}

			return;
		}
		if ((inst & 0b00000000000000000111000001111111) == 0b00000000000000000001000001110011) {
			/*csrrw*/
			cpu.pc += 4;

			uint32_t csr = /*csr[11:0]*/inst >> 20;

			uint32_t tmp;
			csr_read(csr, &tmp);

			csr_write(csr, cpu.xreg[(inst >> 15) & 0b11111 /*rs1*/]);
			cpu.xreg[(inst >> 7) & 0b11111 /*rd*/] = tmp;
			return;
		}
		if ((inst & 0b11111110000000000111000001111111) == 0b00000010000000000111000000110011) {
			/*remu*/
			cpu.pc += 4;

			cpu.xreg[(inst >> 7) & 0b11111 /*rd*/] = (cpu.xreg[(inst >> 20) & 0b11111 /*rs2*/] == 0) ? -1 : (cpu.xreg[(inst >> 15) & 0b11111 /*rs1*/] % cpu.xreg[(inst >> 20) & 0b11111 /*rs2*/]);
			return;
		}
		if ((inst & 0b00000000000000000111000001111111) == 0b00000000000000000011000001110011) {
			/*csrrc*/
			cpu.pc += 4;

			uint32_t csr = /*csr[11:0]*/inst >> 20;

			uint32_t tmp;
			csr_read(csr, &tmp);

			csr_write(csr, tmp & ~cpu.xreg[(inst >> 15) & 0b11111 /*rs1*/]);
			cpu.xreg[(inst >> 7) & 0b11111 /*rd*/] = tmp;
			return;
		}
		if ((inst & 0b11111110000000000111000001111111) == 0b00000010000000000101000000110011) {
			/*divu*/
			cpu.pc += 4;

			cpu.xreg[(inst >> 7) & 0b11111 /*rd*/] = (cpu.xreg[(inst >> 20) & 0b11111 /*rs2*/] == 0) ? -1 : (cpu.xreg[(inst >> 15) & 0b11111 /*rs1*/] / cpu.xreg[(inst >> 20) & 0b11111 /*rs2*/]);
			return;
		}
		if ((inst & 0b11111000000000000111000001111111) == 0b00001000000000000010000000101111) {
			/*amoswap.w*/
			uint32_t addr= cpu.xreg[(inst >> 15) & 0b11111 /*rs1*/];
			if (cpu_addr_translate(&addr, STORE_AMO_PAGE_FAULT)) {
				cpu.pc += 4;

				uint32_t value;
				bus_read(addr, 4, (uint8_t*) &value);

				bus_write(addr, 4, cpu.xreg[(inst >> 20) & 0b11111 /*rs2*/]);
				cpu.xreg[(inst >> 7) & 0b11111 /*rd*/] = value;
			}

			return;
		}
		if ((inst & 0b11111110000000000111000001111111) == 0b00010010000000000000000001110011) {
			/*sfence.vma*/
			cpu.pc += 4;

			memset(&tlb_, -1, TLB_SIZE * sizeof(struct tlb_t));

			return;
		}
		if ((inst & 0b11111110000000000111000001111111) == 0b00000010000000000100000000110011) {
			/*div*/
			cpu.pc += 4;

			cpu.xreg[(inst >> 7) & 0b11111 /*rd*/] = (cpu.xreg[(inst >> 20) & 0b11111 /*rs2*/] == 0) ? -1 : ((int32_t) cpu.xreg[(inst >> 15) & 0b11111 /*rs1*/] / (int32_t) cpu.xreg[(inst >> 20) & 0b11111 /*rs2*/]);
			return;
		}
		if (inst == 0b00000000000000000001000000001111) {
			/*fence.i*/
			cpu.pc += 4;
			return;
		}
		if (inst == 0b00010000001000000000000001110011) {
			/*sret*/

			// Return from traps in S-mode, and SRET copies SPIE into SIE, then sets SPIE.
			cpu.privilege = (csr.sstatus >> 8 /*SPP*/) & 1;
			csr.sstatus &= ~(1 << 8 /*SPP*/);

			csr.sstatus &= ~(1 << cpu.privilege /*SIE*/);	// clear SIE
			csr.sstatus |= ((csr.sstatus >> 5 /*SPIE*/) & 1) << cpu.privilege; // copies SPIE into SIE
			csr.sstatus |= (1 << 5 /*SPIE*/);               // set SPIE

			cpu.pc = csr.sepc;
			return;
		}
		if (inst == 0b00000000000000000000000001110011) {
			/*ecall*/

			// Environment call from {U,S,M}-mode
			cpu_take_trap(cpu.privilege + 8, 0);

			return;
		}
		if (inst == 0b00110000001000000000000001110011) {
			/*mret*/

			// Return from traps in M-mode, and MRET copies MPIE into MIE, then sets MPIE.
			cpu.privilege = (csr.mstatus >> 11 /*MPP*/) & 3;
			csr.mstatus &= ~(3 << 11 /*MPP*/);

			csr.mstatus &= ~(1 << cpu.privilege /*MIE*/); 	// clear MIE
			csr.mstatus |= ((csr.mstatus >> 7 /*MPIE*/) & 1) << cpu.privilege; // copies MPIE into MIE
			csr.mstatus |= (1 << 7 /*MPIE*/);               // sets MPIE

			cpu.pc = csr.mepc;
			return;
		}
		if ((inst & 0b11111110000000000111000001111111) == 0b00000000000000000010000000110011) {
			/*slt*/
			cpu.pc += 4;

			cpu.xreg[(inst >> 7) & 0b11111 /*rd*/] = (int32_t) cpu.xreg[(inst >> 15) & 0b11111 /*rs1*/] < (int32_t) cpu.xreg[(inst >> 20) & 0b11111 /*rs2*/];
			return;
		}
		if ((inst & 0b11111110000000000111000001111111) == 0b00000010000000000110000000110011) {
			/*rem*/
			cpu.pc += 4;

			cpu.xreg[(inst >> 7) & 0b11111 /*rd*/] = (cpu.xreg[(inst >> 20) & 0b11111 /*rs2*/] == 0) ? -1 : ((int32_t) cpu.xreg[(inst >> 15) & 0b11111 /*rs1*/] % (int32_t) cpu.xreg[(inst >> 20) & 0b11111 /*rs2*/]);
			return;
		}
		if ((inst & 0b11111110000000000111000001111111) == 0b00000010000000000001000000110011) {
			/*mulh*/
			cpu.pc += 4;

			cpu.xreg[(inst >> 7) & 0b11111 /*rd*/] = ((int64_t) cpu.xreg[(inst >> 15) & 0b11111 /*rs1*/] * (int64_t) cpu.xreg[(inst >> 20) & 0b11111 /*rs2*/]) >> 32;
			return;
		}
		if ((inst & 0b00000000000000000111000001111111) == 0b00000000000000000010000000010011) {
			/*slti*/
			cpu.pc += 4;

			uint32_t imm = /*imm[11:0]*/(int32_t) inst >> 20;
			cpu.xreg[(inst >> 7) & 0b11111 /*rd*/] = (int32_t) cpu.xreg[(inst >> 15) & 0b11111 /*rs1*/] < (int32_t) imm;
			return;
		}
		if ((inst & 0b00000000000000000111000001111111) == 0b00000000000000000101000001110011) {
			/*csrrwi*/
			cpu.pc += 4;

			uint32_t imm = /*imm[4:0]*/((inst & 0b00000000000011111000000000000000) >> 15);

			uint32_t csr = /*csr[11:0]*/inst >> 20;

			uint32_t tmp;
			csr_read(csr, &tmp);

			csr_write(csr, imm);
			cpu.xreg[(inst >> 7) & 0b11111 /*rd*/] = tmp;
			return;
		}
		if ((inst & 0b11111000000000000111000001111111) == 0b11100000000000000010000000101111) {
			/*amomaxu.w*/
			uint32_t addr= cpu.xreg[(inst >> 15) & 0b11111 /*rs1*/];
			if (cpu_addr_translate(&addr, STORE_AMO_PAGE_FAULT)) {
				cpu.pc += 4;

				uint32_t value;
				bus_read(addr, 4, (uint8_t*) &value);

				bus_write(addr, 4, MAX(value, cpu.xreg[(inst >> 20) & 0b11111 /*rs2*/]));
				cpu.xreg[(inst >> 7) & 0b11111 /*rd*/] = value;
			}

			return;
		}
		if ((inst & 0b11111110000000000111000001111111) == 0b00000010000000000010000000110011) {
			/*mulhsu*/
			cpu.pc += 4;

			cpu.xreg[(inst >> 7) & 0b11111 /*rd*/] = ((int64_t) cpu.xreg[(inst >> 15) & 0b11111 /*rs1*/] * (uint64_t) cpu.xreg[(inst >> 20) & 0b11111 /*rs2*/]) >> 32;
			return;
		}
		if ((inst & 0b00000000000000000111000001111111) == 0b00000000000000000011000000100111) {
			/*fsd*/
			uint32_t imm = /*imm[11:5] imm[4:0]*/
					((inst & 0b11111110000000000000000000000000) >> 20) |
					((inst & 0b00000000000000000000111110000000) >> 7);

			uint32_t addr= cpu.xreg[(inst >> 15) & 0b11111 /*rs1*/] + ((int32_t) (imm << 20) >> 20);
			if (cpu_addr_translate(&addr, STORE_AMO_PAGE_FAULT)) {
				cpu.pc += 4;

				bus_write(addr, 8, cpu.freg[(inst >> 20) & 0b11111 /*rs2*/] );
			}

			return;
		}
		if ((inst & 0b00000000000000000111000001111111) == 0b00000000000000000011000000000111) {
			/*fld*/
			uint32_t imm = /*imm[11:0]*/(int32_t) inst >> 20;

			uint32_t addr= cpu.xreg[(inst >> 15) & 0b11111 /*rs1*/] + imm;
			if (cpu_addr_translate(&addr, LOAD_PAGE_FAULT)) {
				cpu.pc += 4;

				bus_read(addr, 8, (uint8_t*) &cpu.freg[(inst >> 7) & 0b11111 /*rd*/]);
			}

			return;
		}
		if ((inst & 0b11111000000000000111000001111111) == 0b10100000000000000010000001010011) {
			/*feq.s*/
			/*feq.d*/
			cpu.pc += 4;

			cpu.xreg[(inst >> 7) & 0b11111 /*rd*/]= (cpu.freg[(inst >> 15) & 0b11111 /*rs1*/] == cpu.freg[(inst >> 20) & 0b11111 /*rs2*/]);
			return;
		}
		if ((inst & 0b11111110000000000111000001111111) == 0b10100010000000000001000001010011) {
			/*flt.d*/
			cpu.pc += 4;

			cpu.xreg[(inst >> 7) & 0b11111 /*rd*/]= unpack754_64(cpu.freg[(inst >> 15) & 0b11111 /*rs1*/]) < unpack754_64(cpu.freg[(inst >> 20) & 0b11111 /*rs2*/]);
			return;
		}
		if ((inst & 0b11111110000000000111000001111111) == 0b10100010000000000000000001010011) {
			/*fle.d*/
			cpu.pc += 4;

			cpu.xreg[(inst >> 7) & 0b11111 /*rd*/]= unpack754_64(cpu.freg[(inst >> 15) & 0b11111 /*rs1*/]) <= unpack754_64(cpu.freg[(inst >> 20) & 0b11111 /*rs2*/]);
			return;
		}
		if ((inst & 0b11111110000000000000000001111111) == 0b00000010000000000000000001010011) {
			/*fadd.d*/
			cpu.pc += 4;

			cpu.freg[(inst >> 7) & 0b11111 /*rd*/]= pack754_64(unpack754_64(cpu.freg[(inst >> 15) & 0b11111 /*rs1*/]) + unpack754_64(cpu.freg[(inst >> 20) & 0b11111 /*rs2*/]));
			return;
		}
		if ((inst & 0b11111110000000000000000001111111) == 0b00001010000000000000000001010011) {
			/*fsub.d*/
			cpu.pc += 4;

			cpu.freg[(inst >> 7) & 0b11111 /*rd*/]= pack754_64(unpack754_64(cpu.freg[(inst >> 15) & 0b11111 /*rs1*/]) - unpack754_64(cpu.freg[(inst >> 20) & 0b11111 /*rs2*/]));
			return;
		}
		if ((inst & 0b11111110000000000000000001111111) == 0b00010010000000000000000001010011) {
			/*fmul.d*/
			cpu.pc += 4;

			cpu.freg[(inst >> 7) & 0b11111 /*rd*/]= pack754_64(unpack754_64(cpu.freg[(inst >> 15) & 0b11111 /*rs1*/]) * unpack754_64(cpu.freg[(inst >> 20) & 0b11111 /*rs2*/]));
			return;
		}
		if ((inst & 0b11111110000000000000000001111111) == 0b00011010000000000000000001010011) {
			/*fdiv.d*/
			cpu.pc += 4;

			cpu.freg[(inst >> 7) & 0b11111 /*rd*/]= pack754_64( unpack754_64(cpu.freg[(inst >> 15) & 0b11111 /*rs1*/]) / unpack754_64(cpu.freg[(inst >> 20) & 0b11111 /*rs2*/]) );
			return;
		}
		if ((inst & 0b11111111111100000000000001111111) == 0b11010010000000000000000001010011) {
			/*fcvt.d.w*/
			cpu.pc += 4;

			cpu.freg[(inst >> 7) & 0b11111 /*rd*/]= pack754_64( (int32_t) cpu.xreg[(inst >> 15) & 0b11111 /*rs1*/] );
			return;
		}
		if ((inst & 0b11111111111100000000000001111111) == 0b11000010000000000000000001010011) {
			/*fcvt.w.d*/
			cpu.pc += 4;

			cpu.xreg[(inst >> 7) & 0b11111 /*rd*/]= (int32_t) unpack754_64(cpu.freg[(inst >> 15) & 0b11111 /*rs1*/]);
			return;
		}
		if ((inst & 0b11111111111100000000000001111111) == 0b11010010000100000000000001010011) {
			/*fcvt.d.wu*/
			cpu.pc += 4;

			cpu.freg[(inst >> 7) & 0b11111 /*rd*/]= pack754_64( (uint32_t) cpu.xreg[(inst >> 15) & 0b11111 /*rs1*/] );
			return;
		}
		if ((inst & 0b11111111111100000000000001111111) == 0b11000010000100000000000001010011) {
			/*fcvt.wu.d*/
			cpu.pc += 4;

			cpu.xreg[(inst >> 7) & 0b11111 /*rd*/]= (uint32_t) unpack754_64(cpu.freg[(inst >> 15) & 0b11111 /*rs1*/]);
			return;
		}
		if ((inst & 0b11111111111100000111000001111111) == 0b11110000000000000000000001010011) {
			/*fmv.w.x*/
			cpu.pc += 4;

			cpu.freg[(inst >> 7) & 0b11111 /*rd*/]= cpu.xreg[(inst >> 15) & 0b11111 /*rs1*/];
			return;
		}
		if ((inst & 0b11111111111100000111000001111111) == 0b11100000000000000000000001010011) {
			/*fmv.x.w*/
			cpu.pc += 4;

			cpu.xreg[(inst >> 7) & 0b11111 /*rd*/]= cpu.freg[(inst >> 15) & 0b11111 /*rs1*/];
			return;
		}
		if ((inst & 0b00000110000000000000000001111111) == 0b00000010000000000000000001001011) {
			/*fnmsub.d*/
			cpu.pc += 4;

			uint32_t rs3 = (inst >> 27) & 0b11111;

			cpu.freg[(inst >> 7) & 0b11111 /*rd*/]= pack754_64( -unpack754_64(cpu.freg[(inst >> 15) & 0b11111 /*rs1*/]) * unpack754_64(cpu.freg[(inst >> 20) & 0b11111 /*rs2*/]) + unpack754_64(cpu.freg[rs3]) );
			return;
		}
		if ((inst & 0b00000110000000000000000001111111) == 0b00000010000000000000000001000011) {
			/*fmadd.d*/
			cpu.pc += 4;

			uint32_t rs3 = (inst >> 27) & 0b11111;

			cpu.freg[(inst >> 7) & 0b11111 /*rd*/]= pack754_64( unpack754_64(cpu.freg[(inst >> 15) & 0b11111 /*rs1*/]) * unpack754_64(cpu.freg[(inst >> 20) & 0b11111 /*rs2*/]) + unpack754_64(cpu.freg[rs3]) );
			return;
		}
		if ((inst & 0b11111110000000000111000001111111) == 0b00100010000000000000000001010011) {
			/*fsgnj.d*/
			cpu.pc += 4;

			cpu.freg[(inst >> 7) & 0b11111 /*rd*/]= (cpu.freg[(inst >> 15) & 0b11111 /*rs1*/] & ~(1LL << 63)) | (cpu.freg[(inst >> 20) & 0b11111 /*rs2*/] & (1LL << 63));
			return;
		}
		if ((inst & 0b11111110000000000111000001111111) == 0b00100010000000000001000001010011) {
			/*fsgnjn.d*/
			cpu.pc += 4;

			cpu.freg[(inst >> 7) & 0b11111 /*rd*/]= (cpu.freg[(inst >> 15) & 0b11111 /*rs1*/] & ~(1LL << 63)) | (~cpu.freg[(inst >> 20) & 0b11111 /*rs2*/] & (1LL << 63));
			return;
		}
		if ((inst & 0b11111110000000000111000001111111) == 0b00100010000000000010000001010011) {
			/*fsgnjx.d*/
			cpu.pc += 4;

			cpu.freg[(inst >> 7) & 0b11111 /*rd*/]= (cpu.freg[(inst >> 15) & 0b11111 /*rs1*/] & ~(1LL << 63)) | ((cpu.freg[(inst >> 20) & 0b11111 /*rs2*/] ^ cpu.freg[(inst >> 15) & 0b11111 /*rs1*/]) & (1LL << 63));
			return;
		}
		if ((inst & 0b11111111111100000000000001111111) == 0b01000000000100000000000001010011) {
			/*fcvt.s.d*/
			cpu.pc += 4;

			cpu.freg[(inst >> 7) & 0b11111 /*rd*/]= pack754_32( (float) unpack754_64(cpu.freg[(inst >> 15) & 0b11111 /*rs1*/]) );
			return;
		}
		if ((inst & 0b11111111111100000000000001111111) == 0b01000010000000000000000001010011) {
			/*fcvt.d.s*/
			cpu.pc += 4;

			cpu.freg[(inst >> 7) & 0b11111 /*rd*/]= pack754_64( (double) unpack754_32(cpu.freg[(inst >> 15) & 0b11111 /*rs1*/]) );
			return;
		}
		/*if (inst == 0b00000000000100000000000001110011) {
			// ebreak

			cpu_take_trap(BREAKPOINT, cpu.pc + 4);
			return;
		}*/
		if (inst == 0b00010000010100000000000001110011) {
			/*wfi*/
			cpu.pc += 4;

			return;
		}

		cpu_take_trap(ILLEGAL_INSTRUCTION, inst);

	} else {

		if ((inst & 0b1110000000000011) == 0b1100000000000010) {
			/*c.swsp*/
			uint32_t imm = // imm[5:2|7:6]
					((inst & 0b0001111000000000) >> 7) |
					((inst & 0b0000000110000000) >> 1);

			uint32_t addr= cpu.xreg[2] + imm;
			if (cpu_addr_translate(&addr, STORE_AMO_PAGE_FAULT)) {
                cpu.pc += 2;

				bus_write(addr, 4, cpu.xreg[(inst >> 2) & 0b11111]);
			}

			return;
		}
		if ((inst & 0b1110000000000011) == 0b0100000000000010) {
			/*c.lwsp*/
			uint32_t imm = // imm[5] imm[4:2|7:6]
					((inst & 0b0001000000000000) >> 7) |
					((inst & 0b0000000001110000) >> 2) |
					((inst & 0b0000000000001100) << 4);

			uint32_t addr= cpu.xreg[2] + imm;
			if (cpu_addr_translate(&addr, LOAD_PAGE_FAULT)) {
				cpu.pc += 2;

				bus_read(addr, 4, (uint8_t*) &cpu.xreg[(inst >> 7) & 0b11111]);
			}

			return;
		}
		if ((inst & 0b1111000001111111) == 0b1000000000000010) {
			/*c.jr*/
			cpu.pc = cpu.xreg[(inst >> 7) & 0b11111];
			return;

		} else if ((inst & 0b1111000000000011) == 0b1000000000000010) {
			/*c.mv*/
			cpu.pc += 2;

			cpu.xreg[(inst >> 7) & 0b11111] = cpu.xreg[(inst >> 2) & 0b11111];
			return;
		}
		if ((inst & 0b1110000000000011) == 0b0000000000000001) {
			/*c.addi/c.nop*/
			cpu.pc += 2;

			uint32_t imm = // imm[5] imm[4:0]
					((inst & 0b0001000000000000) >> 7) |
					((inst & 0b0000000001111100) >> 2);

			cpu.xreg[(inst >> 7) & 0b11111] += ((int32_t) (imm << 26) >> 26);
			return;
		}
		if ((inst & 0b1110000000000011) == 0b0100000000000000) {
			/*c.lw*/
			uint32_t imm = // imm[5:3] imm[2|6]
					((inst & 0b0001110000000000) >> 7) |
					((inst & 0b0000000001000000) >> 4) |
					((inst & 0b0000000000100000) << 1);

			uint32_t addr= cpu.xreg[((inst >> 7) & 0b111) + 8] + imm;
			if (cpu_addr_translate(&addr, LOAD_PAGE_FAULT)) {
				cpu.pc += 2;

				bus_read(addr, 4, (uint8_t*) &cpu.xreg[((inst >> 2) & 0b111) + 8]);
			}

			return;
		}
		if ((inst & 0b1110000000000011) == 0b0100000000000001) {
			/*c.li*/
			cpu.pc += 2;

			uint32_t imm = // imm[5] imm[4:0]
					((inst & 0b0001000000000000) >> 7) |
					((inst & 0b0000000001111100) >> 2);

			cpu.xreg[(inst >> 7) & 0b11111] = ((int32_t) (imm << 26) >> 26);
			return;
		}
		if ((inst & 0b1110000000000011) == 0b1100000000000001) {
			/*c.beqz*/
			if (cpu.xreg[((inst >> 7) & 0b111) + 8] == 0) {

				uint32_t imm = // imm[8|4:3] imm[7:6|2:1|5]
						 ((inst & 0b0001000000000000) >> 4) |
						 ((inst & 0b0000110000000000) >> 7) |
						 ((inst & 0b0000000001100000) << 1) |
						 ((inst & 0b0000000000011000) >> 2) |
						 ((inst & 0b0000000000000100) << 3);

				cpu.pc += ((int32_t) (imm << 23) >> 23);

			} else cpu.pc += 2;

			return;
		}
		if ((inst & 0b1110000000000011) == 0b0000000000000000) {
			/*c.addi4spn*/
			cpu.pc += 2;

			uint32_t imm = // imm[5:4|9:6|2|3]
					((inst & 0b0001100000000000) >> 7) |
					((inst & 0b0000011110000000) >> 1) |
					((inst & 0b0000000001000000) >> 4) |
					((inst & 0b0000000000100000) >> 2);

			cpu.xreg[((inst >> 2) & 0b111) + 8] = cpu.xreg[2] + imm;
			return;
		}
		if ((inst & 0b1110000000000011) == 0b1110000000000001) {
			/*c.bnez*/
			if (cpu.xreg[((inst >> 7) & 0b111) + 8] != 0) {

				uint32_t imm = // imm[8|4:3] imm[7:6|2:1|5]
						((inst & 0b0001000000000000) >> 4) |
						((inst & 0b0000110000000000) >> 7) |
						((inst & 0b0000000001100000) << 1) |
						((inst & 0b0000000000011000) >> 2) |
						((inst & 0b0000000000000100) << 3);

				cpu.pc += ((int32_t) (imm << 23) >> 23);

			} else cpu.pc += 2;

			return;
		}
		if (inst == 0b1001000000000010) {
			/*c.ebreak*/

			cpu_take_trap(BREAKPOINT, cpu.pc + 2);
			return;

		} else if ((inst & 0b1111000001111111) == 0b1001000000000010) {
			/*c.jalr*/

			uint32_t tmp = cpu.pc + 2;
			cpu.pc = cpu.xreg[(inst >> 7) & 0b11111];

			cpu.xreg[1] = tmp;
			return;

		} else if ((inst & 0b1111000000000011) == 0b1001000000000010) {
			/*c.add*/
			cpu.pc += 2;

			cpu.xreg[(inst >> 7) & 0b11111] += cpu.xreg[(inst >> 2) & 0b11111];
			return;
		}
		if ((inst & 0b1110000000000011) == 0b1010000000000001) {
			/*c.j*/
			uint32_t imm = // imm[11|4|9:8|10|6|7|3:1|5]
					((inst & 0b0001000000000000) >> 1) |
					((inst & 0b0000100000000000) >> 7) |
					((inst & 0b0000011000000000) >> 1) |
					((inst & 0b0000000100000000) << 2) |
					((inst & 0b0000000010000000) >> 1) |
					((inst & 0b0000000001000000) << 1) |
					((inst & 0b0000000000111000) >> 2) |
					((inst & 0b0000000000000100) << 3);

			cpu.pc += ((int32_t) (imm << 20) >> 20);
			return;
		}
		if ((inst & 0b1110111110000011) == 0b0110000100000001) {
			/*c.addi16sp*/
			cpu.pc += 2;

			uint32_t imm = // imm[9] | imm[4|6|8:7|5]
					((inst & 0b0001000000000000) >> 3) |
					((inst & 0b0000000001000000) >> 2) |
					((inst & 0b0000000000100000) << 1) |
					((inst & 0b0000000000011000) << 4) |
					((inst & 0b0000000000000100) << 3);

			cpu.xreg[2] += ((int32_t) (imm << 22) >> 22);
			return;

		} else if ((inst & 0b1110000000000011) == 0b0110000000000001) {
			/*c.lui*/
			cpu.pc += 2;

			uint32_t imm = // imm[17] | imm[16:12]
					((inst & 0b0001000000000000) << 5) |
					((inst & 0b0000000001111100) << 10);

			cpu.xreg[(inst >> 7) & 0b11111] = ((int32_t) (imm << 14) >> 14);
			return;
		}
		if ((inst & 0b1110110000000011) == 0b1000100000000001) {
			/*c.andi*/
			cpu.pc += 2;

			uint32_t imm = // imm[5] imm[4:0]
					((inst & 0b0001000000000000) >> 7) |
					((inst & 0b0000000001111100) >> 2);

			cpu.xreg[((inst >> 7) & 0b111) + 8] &= ((int32_t) (imm << 26) >> 26);
			return;
		}
		if ((inst & 0b1110000000000011) == 0b0010000000000001) {
			/*c.jal*/
			uint32_t imm = // imm[11|4|9:8|10|6|7|3:1|5]
					((inst & 0b0001000000000000) >> 1) |
					((inst & 0b0000100000000000) >> 7) |
					((inst & 0b0000011000000000) >> 1) |
					((inst & 0b0000000100000000) << 2) |
					((inst & 0b0000000010000000) >> 1) |
					((inst & 0b0000000001000000) << 1) |
					((inst & 0b0000000000111000) >> 2) |
					((inst & 0b0000000000000100) << 3);

			cpu.xreg[1] = cpu.pc + 2;
			cpu.pc += ((int32_t) (imm << 20) >> 20);
			return;
		}
		if ((inst & 0b1110000000000011) == 0b1100000000000000) {
			/*c.sw*/
			uint32_t imm = // imm[5:3] imm[2|6]
					((inst & 0b0001110000000000) >> 7) |
					((inst & 0b0000000001000000) >> 4) |
					((inst & 0b0000000000100000) << 1);

			uint32_t addr= cpu.xreg[((inst >> 7) & 0b111) + 8] + imm;
			if (cpu_addr_translate(&addr, STORE_AMO_PAGE_FAULT)) {
				cpu.pc += 2;

				bus_write(addr, 4, cpu.xreg[((inst >> 2) & 0b111) + 8]);
			}
			return;
		}
		if ((inst & 0b1110000000000011) == 0b0000000000000010) {
			/*c.slli*/
			cpu.pc += 2;

			uint32_t imm = /*imm[4:0]*/ ((inst >> 2) & 0x1f);

			cpu.xreg[(inst >> 7) & 0b11111] <<= imm;
			return;
		}
		if ((inst & 0b1110110000000011) == 0b1000000000000001) {
			/*c.srli*/
			cpu.pc += 2;

			uint32_t imm = /*imm[4:0]*/ ((inst >> 2) & 0x1f);

			cpu.xreg[((inst >> 7) & 0b111) + 8] >>= imm;
			return;
		}
		if ((inst & 0b1111110001100011) == 0b1000110001000001) {
			/*c.or*/
			cpu.pc += 2;

			cpu.xreg[((inst >> 7) & 0b111) + 8] |= cpu.xreg[((inst >> 2) & 0b111) + 8];
			return;
		}
		if ((inst & 0b1111110001100011) == 0b1000110001100001) {
			/*c.and*/
			cpu.pc += 2;

			cpu.xreg[((inst >> 7) & 0b111) + 8] &= cpu.xreg[((inst >> 2) & 0b111) + 8];
			return;
		}
		if ((inst & 0b1111110001100011) == 0b1000110000000001) {
			/*c.sub*/
			cpu.pc += 2;

			cpu.xreg[((inst >> 7) & 0b111) + 8] -= cpu.xreg[((inst >> 2) & 0b111) + 8];
			return;
		}
		if ((inst & 0b1111110001100011) == 0b1000110000100001) {
			/*c.xor*/
			cpu.pc += 2;

			cpu.xreg[((inst >> 7) & 0b111) + 8] ^= cpu.xreg[((inst >> 2) & 0b111) + 8];
			return;
		}
		if ((inst & 0b1110110000000011) == 0b1000010000000001) {
			/*c.srai*/
			cpu.pc += 2;

			uint32_t imm = /*imm[4:0]*/ ((inst >> 2) & 0x1f);

			cpu.xreg[((inst >> 7) & 0b111) + 8] = (int32_t) cpu.xreg[((inst >> 7) & 0b111) + 8] >> imm;
			return;
		}
		if ((inst & 0b1110000000000011) == 0b0010000000000000) {
			/*c.fld*/
			uint32_t imm = // imm[5:3] imm[7:6]
					((inst & 0b0001110000000000) >> 7) |
					((inst & 0b0000000001100000) << 1);

			uint32_t addr= cpu.xreg[((inst >> 7) & 0b111) + 8] + imm;
			if (cpu_addr_translate(&addr, LOAD_PAGE_FAULT)) {
				cpu.pc += 2;

				bus_read(addr, 8, (uint8_t*) &cpu.freg[((inst >> 2) & 0b111) + 8]);
			}

			return;
		}

		cpu_take_trap(ILLEGAL_INSTRUCTION, inst & 0xffff);
	}
}

struct flow_wifi2eth_msg_t {
	void *packet, *eb;
	uint16_t length;
};

static QueueHandle_t net0_queue = NULL;

static int pkt_wifi2eth(void *buffer, uint16_t len, void *eb) {

	struct flow_wifi2eth_msg_t msg= { .length= len, .packet= buffer, .eb= eb};

	xQueueSend(net0_queue, &msg, portMAX_DELAY);

	return 0;
}

static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {

	switch (event_id) {
	case WIFI_EVENT_STA_START:
		esp_wifi_connect();
		break;
	case WIFI_EVENT_STA_CONNECTED:
		esp_wifi_internal_reg_rxcb(WIFI_IF_STA, pkt_wifi2eth);
		break;
	case WIFI_EVENT_STA_DISCONNECTED:
		esp_wifi_connect();
		break;
	}

}

void vTaskCode( void * pvParameters ){

	uint32_t addr, inst[2];

	for(;;){

		clint.time = esp_timer_get_time() /*microseconds*/ * 240 /*Mhz*/;

		if (clint.timecmp && ((uint32_t) clint.time > (uint32_t) clint.timecmp)) {
			clint.timecmp = 0;

			csr.sip |= (1 << SUPERVISOR_TIMER_INTERRUPT);
		}

		for(uint16_t x= 0; x < 10946; x++ /*only to performance, prioritizing the cpu_execute*/){

			addr= cpu.pc & ~3;
			if (cpu_addr_translate(&addr, INSTRUCTION_PAGE_FAULT)) {

				stream_lru_read(addr - RAM_BASE, sizeof(inst), (uint8_t*) &inst);

				if ((cpu.pc & 2) == 2) {

					inst[0] >>= 16;

					if ((inst[0] & 3) == 3) {

						inst[0] |= (inst[1] << 16);
					}
				}
				cpu_execute(inst[0]);
			}

			/*Check interrupt enabled*/
			if (cpu.privilege <= SUPERVISOR && !((csr.sstatus >> cpu.privilege) & 1))
				continue;

			if (cpu.privilege == MACHINE && !((csr.mstatus >> MACHINE) & 1))
				continue;

			/*Check pending interrupt*/
			if (csr.sie & csr.sip & (1 << SUPERVISOR_TIMER_INTERRUPT)) {
				csr.sip &= ~(1 << SUPERVISOR_TIMER_INTERRUPT);
				cpu_take_trap(SUPERVISOR_TIMER_INTERRUPT | (1 << 31), 0);

			} else if (csr.sie & csr.sip & (1 << SUPERVISOR_EXTERNAL_INTERRUPT)) {
				csr.sip &= ~(1 << SUPERVISOR_EXTERNAL_INTERRUPT);
				cpu_take_trap(SUPERVISOR_EXTERNAL_INTERRUPT | (1 << 31), 0);

			} /*else if (csr.sie & csr.sip & (1 << SUPERVISOR_SOFTWARE_INTERRUPT)) {
				csr.sip &= ~(1 << SUPERVISOR_SOFTWARE_INTERRUPT);
				cpu_take_trap(SUPERVISOR_SOFTWARE_INTERRUPT | (1 << 31), 0);

			} else if (csr.mie & csr.mip & (1 << MACHINE_EXTERNAL_INTERRUPT)) {
				csr.mip &= ~(1 << MACHINE_EXTERNAL_INTERRUPT);
				cpu_take_trap(MACHINE_EXTERNAL_INTERRUPT | (1 << 31), 0);

			} else if (csr.mie & csr.mip & (1 << MACHINE_TIMER_INTERRUPT)) {
				csr.mip &= ~(1 << MACHINE_TIMER_INTERRUPT);
				cpu_take_trap(MACHINE_TIMER_INTERRUPT | (1 << 31), 0);

			} else if (csr.mie & csr.mip & (1 << MACHINE_SOFTWARE_INTERRUPT)) {
				csr.mip &= ~(1 << MACHINE_SOFTWARE_INTERRUPT);
				cpu_take_trap(MACHINE_SOFTWARE_INTERRUPT | (1 << 31), 0);
			}*/
		}

		if (virtio8_net.queue[0 /*receiveq1*/].notify && uxQueueMessagesWaiting(net0_queue)){
			struct virtio_queue_t *virtio_queue = &virtio8_net.queue[0];

			virtio_queue->notify = 0;

			struct virtq_avail_t virtq_avail;
			stream_lru_read(virtio_queue->driver_addr - RAM_BASE, sizeof(struct virtq_avail_t), (uint8_t*) &virtq_avail);

			struct virtq_used_t virtq_used;
			stream_lru_read(virtio_queue->dev_addr - RAM_BASE, sizeof(struct virtq_used_t), (uint8_t*) &virtq_used);

			struct virtio_net_hdr_t {
				uint8_t flags;
				uint8_t gso_type;
				uint16_t hdr_len;
				uint16_t gso_size;
				uint16_t csum_start;
				uint16_t csum_offset;
				uint16_t num_buffers;
			};

			struct flow_wifi2eth_msg_t msg;
			while (virtq_used.idx != virtq_avail.idx) {

				if ( xQueueReceive(net0_queue, &msg, 0) ){

					struct virtq_desc_t virtq_desc;
					stream_lru_read( virtio_queue->desc_addr + virtq_avail.ring[virtq_used.idx % virtio_queue->num] * sizeof(struct virtq_desc_t) - RAM_BASE, sizeof(struct virtq_desc_t), (uint8_t*) &virtq_desc);

					// ######################################################################
//					assert(virtq_desc.flags & (2 /*VRING_DESC_F_WRITE*/));

					struct virtio_net_hdr_t virtio_net_hdr= {0x00, 0x00, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};
					stream_lru_write(virtq_desc.addr - RAM_BASE, sizeof(struct virtio_net_hdr_t), (uint8_t*) &virtio_net_hdr);

					stream_lru_write((virtq_desc.addr - RAM_BASE) + sizeof(struct virtio_net_hdr_t), msg.length, msg.packet);

					esp_wifi_internal_free_rx_buffer(msg.eb);

					// ######################################################################
					virtq_used.ring[virtq_used.idx % virtio_queue->num].id = virtq_avail.ring[virtq_used.idx % virtio_queue->num];
					virtq_used.ring[virtq_used.idx % virtio_queue->num].len = sizeof(struct virtio_net_hdr_t) + msg.length;

				} else {

					virtq_used.ring[virtq_used.idx % virtio_queue->num].id = virtq_avail.ring[virtq_used.idx % virtio_queue->num];
					virtq_used.ring[virtq_used.idx % virtio_queue->num].len = 0;
				}

				++virtq_used.idx;
			}
			stream_lru_write(virtio_queue->dev_addr - RAM_BASE, sizeof(struct virtq_used_t), (uint8_t*) &virtq_used);

//			assert(~virtq_avail.flags & (1 /*VIRTQ_AVAIL_F_NO_INTERRUPT*/));

			plic.sclaim |= (1 << 0x08 /*net's irq*/);
			csr.sip |= (1 << SUPERVISOR_EXTERNAL_INTERRUPT);

			virtio8_net.int_status |= 1;
		}

		uint8_t uart_buff[16], uart_reading;

		if ( uart_com_1.ier /*RX (RHR)*/ & ~uart_com_1.lsr /*RX Data Ready*/ & 1 ) {

			if ( uart_read_bytes(UART_NUM_1, &uart_buff, sizeof(uart_buff[0]), 0)){

				uart_com_1.rhr= uart_buff[0];
				uart_com_1.lsr |= (1 /*RX Data Ready*/);

				plic.sclaim |= (1 << 5 /*uart_com_1's irq*/);
				csr.sip |= (1 << SUPERVISOR_EXTERNAL_INTERRUPT);
			}
		}

		if (virtio1_console.queue[0 /*receiveq*/].notify && (uart_reading= fread(&uart_buff, sizeof(uint8_t), sizeof(uart_buff), stdin) ) ) {
			struct virtio_queue_t *virtio_queue = &virtio1_console.queue[0];

			virtio_queue->notify = 0;

			struct virtq_avail_t virtq_avail;
			stream_lru_read(virtio_queue->driver_addr - RAM_BASE, sizeof(struct virtq_avail_t), (uint8_t*) &virtq_avail);

			struct virtq_used_t virtq_used;
			stream_lru_read(virtio_queue->dev_addr - RAM_BASE, sizeof(struct virtq_used_t), (uint8_t*) &virtq_used);

			struct virtq_desc_t virtq_desc;
			stream_lru_read( virtio_queue->desc_addr + virtq_avail.ring[virtq_used.idx % virtio_queue->num] * sizeof(struct virtq_desc_t) - RAM_BASE, sizeof(struct virtq_desc_t), (uint8_t*) &virtq_desc);

//			assert(virtq_desc.flags & (2 /*VRING_DESC_F_WRITE*/));

			stream_lru_write(virtq_desc.addr - RAM_BASE, uart_reading, (uint8_t*) &uart_buff);
			// ######################################################################

			virtq_used.ring[virtq_used.idx % virtio_queue->num].id = virtq_avail.ring[virtq_used.idx % virtio_queue->num];
			virtq_used.ring[virtq_used.idx % virtio_queue->num].len = uart_reading;
			++virtq_used.idx;

			stream_lru_write(virtio_queue->dev_addr - RAM_BASE, sizeof(struct virtq_used_t), (uint8_t*) &virtq_used);

//			assert(~virtq_avail.flags & (1 /*VIRTQ_AVAIL_F_NO_INTERRUPT*/));

			plic.sclaim |= (1 << 0x01 /*console's irq*/);
			csr.sip |= (1 << SUPERVISOR_EXTERNAL_INTERRUPT);

			virtio1_console.int_status |= 1;
		}
	}
}

void app_main() {

	esp_vfs_littlefs_conf_t littlefs_conf = {
			.base_path = "/data",
			.partition_label = "_linux",
			.format_if_mount_failed = true,
			.dont_mount = false, };

	ESP_ERROR_CHECK(esp_vfs_littlefs_register(&littlefs_conf));

	partition_memory = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "_memory");

	for(uint32_t x= 0; x < (partition_memory->size / LRU_SEC_SIZE); x++) {

		struct window_t *win = &window_[ x ];
		win->sector = x;
		win->time = 0x0000;
		win->data = (uint8_t*) 0;
	}

	for(uint32_t x= 0; x < (0x800000 /*8MB*/ / LRU_SEC_SIZE); x++) {

		struct window_t *win = &window_[ x + (partition_memory->size / LRU_SEC_SIZE) ];
		win->sector = x + (partition_memory->size / LRU_SEC_SIZE);
		win->time = 0x0000;
		win->data = heap_caps_malloc_prefer(LRU_SEC_SIZE, 2, MALLOC_CAP_SPIRAM, MALLOC_CAP_INTERNAL);
	}

	{
		FILE *dtb = fopen("/data/riscv_emulator.dtb", "r");
		low_memory = (uint8_t*) malloc(4096 /*4Kb*/);

		fread(low_memory, sizeof(uint8_t), 4096, dtb);
		fclose(dtb);
	}

	// ################################################################
	const tinyusb_config_t tusb_cfg = {
			.device_descriptor = (void*) 0,
			.string_descriptor = (void*) 0,
			.external_phy = false,
			.configuration_descriptor = (void*) 0,
	};
	ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

	tinyusb_config_cdcacm_t acm_cfg = { 0 };
	ESP_ERROR_CHECK(tusb_cdc_acm_init(&acm_cfg));

    esp_tusb_init_console(TINYUSB_CDC_ACM_0);

	// ################################################################
	ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, 256, 256, 0, (void*) 0, 0));
	ESP_ERROR_CHECK (uart_set_pin( UART_NUM_1, GPIO_NUM_17, GPIO_NUM_18, -1, -1));

	uart_config_t uart_config_1 = {
			.baud_rate = 115200,
			.data_bits = UART_DATA_8_BITS,
			.parity = UART_PARITY_DISABLE,
			.stop_bits = UART_STOP_BITS_1,
			.flow_ctrl = UART_HW_FLOWCTRL_DISABLE };

	ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config_1));

	csr.misa =
			(1 << 0 /*Atomic extension*/) |
			(1 << 2 /*Compressed extension*/) |
			(1 << 3 /*Double-precision floating-point extension*/) |
//			(1 << 4 /*RV32E base ISA*/) |
//			(1 << 5 /*Single-precision floating-point extension*/) |
//			(1 << 7 /*Hypervisor extension*/) |
			(1 << 8 /*RV32I/64I/128I base ISA*/) |
			(1 << 12 /*Integer Multiply/Divide extension*/) |
//			(1 << 13 /*User-level interrupts supported*/) |
//			(1 << 16 /*Quad-precision floating-point extension*/) |
			(1 << 18 /*Supervisor mode implemented*/) |
			(1 << 20 /*User mode implemented*/) |
			(1 << 30 /*32bit 1[32] 2[64] 3[128]*/);

	{
		virtio1_console.dev_feat = 0 |
//				(1 << 0 /*VIRTIO_CONSOLE_F_SIZE*/) |
//				(1 << 1 /*VIRTIO_CONSOLE_F_MULTIPORT*/) |
//				(1 << 2 /*VIRTIO_CONSOLE_F_EMERG_WRITE*/) |
				((uint64_t) 1 << 32 /*VIRTIO_F_VERSION_1*/);

//		struct virtio_console_config {
//			uint16_t cols;
//			uint16_t rows;
//			uint32_t max_nr_ports;
//			uint32_t emerg_wr;
//		};
	}

	virtio4_i2c.dev_feat = 0 |
			(1 << 0 /*VIRTIO_I2C_F_ZERO_LENGTH_REQUEST*/) |
			((uint64_t) 1 << 32 /*VIRTIO_F_VERSION_1*/);

	{
		virtio3_gpio.dev_feat = 0 |
//				(1 << 0 /*VIRTIO_GPIO_F_IRQ*/) |
				((uint64_t) 1 << 32 /*VIRTIO_F_VERSION_1*/);

		struct virtio_gpio_config_t {
			uint16_t ngpio;
			uint8_t padding[2];
			uint32_t gpio_names_size;
		};

		virtio3_gpio.config_space = malloc(sizeof(struct virtio_gpio_config_t));
		memset(virtio3_gpio.config_space, 0, sizeof(struct virtio_gpio_config_t));

		struct virtio_gpio_config_t *virtio_gpio_config = (struct virtio_gpio_config_t*) virtio3_gpio.config_space;

		virtio_gpio_config->ngpio = 2; // ...total number of GPIO lines supported
	}

	// ################################################################
	net0_queue = xQueueCreate(21 /*queue-length*/, sizeof(struct flow_wifi2eth_msg_t));

	ESP_ERROR_CHECK( esp_event_loop_create_default() );
	ESP_ERROR_CHECK( esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, (void*) 0, (void*) 0) );

	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

	cfg.nvs_enable = 0;

	ESP_ERROR_CHECK( esp_wifi_init( &cfg ) );

	ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );

	wifi_config_t wifi_config = {
			.sta = {
					.ssid = "ALLREDE-SOARES-2G",
					.password = "julia2012",
					.threshold.authmode = WIFI_AUTH_WPA2_PSK,
			},
	};

	ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );

	ESP_ERROR_CHECK( esp_wifi_start() );

	{
		virtio8_net.dev_feat = 0 |
				((uint32_t) 1 << 3 /*VIRTIO_NET_F_MTU*/) |
				((uint32_t) 1 << 5 /*VIRTIO_NET_F_MAC*/) |
				((uint64_t) 1 << 32 /*VIRTIO_F_VERSION_1*/);

		struct virtio_net_config_t {
			uint8_t mac[6];
			uint16_t status;
			uint16_t max_virtqueue_pairs;
			uint16_t mtu;
			uint32_t speed;
			uint8_t duplex;
			uint8_t rss_max_key_size;
			uint16_t rss_max_indirection_table_length;
			uint32_t supported_hash_types;
		};

		virtio8_net.config_space= malloc( sizeof(struct virtio_net_config_t));
		memset(virtio8_net.config_space, 0, sizeof(struct virtio_net_config_t));

		struct virtio_net_config_t *virtio_net_config= (struct virtio_net_config_t*) virtio8_net.config_space;

		virtio_net_config->mtu= 576;

		ESP_ERROR_CHECK( esp_wifi_get_mac(WIFI_IF_STA, (uint8_t*) &virtio_net_config->mac) );
	}

	{
		virtio2_blk.dev_feat = 0 |
//				(1 << 1 /*VIRTIO_BLK_F_SIZE_MAX*/) |
//				(1 << 2 /*VIRTIO_BLK_F_SEG_MAX*/) |
//				(1 << 4 /*VIRTIO_BLK_F_GEOMETRY*/) |
//				(1 << 5 /*VIRTIO_BLK_F_RO*/) |
//				(1 << 6 /*VIRTIO_BLK_F_BLK_SIZE*/) |
//				(1 << 9 /*VIRTIO_BLK_F_FLUSH*/) |
//				(1 << 10 /*VIRTIO_BLK_F_TOPOLOGY*/) |
//				(1 << 11 /*VIRTIO_BLK_F_CONFIG_WCE*/) |
				((uint64_t) 1 << 32 /*VIRTIO_F_VERSION_1*/);

		struct virtio_blk_config_t {
			uint64_t capacity;
			uint32_t size_max;
			uint32_t seg_max;
			struct virtio_blk_geometry {
				uint16_t cylinders;
				uint8_t heads;
				uint8_t sectors;
			} geometry;
			uint32_t blk_size;
			struct virtio_blk_topology {
				uint8_t physical_block_exp;
				uint8_t alignment_offset;
				uint16_t min_io_size;
				uint32_t opt_io_size;
			} topology;
			uint8_t writeback;
			uint8_t unused0;
			uint16_t num_queues;
			uint32_t max_discard_sectors;
			uint32_t max_discard_seg;
			uint32_t discard_sector_alignment;
			uint32_t max_write_zeroes_sectors;
			uint32_t max_write_zeroes_seg;
			uint8_t write_zeroes_may_unmap;
			uint8_t unused1[3];
			uint32_t max_secure_erase_sectors;
			uint32_t max_secure_erase_seg;
			uint32_t secure_erase_sector_alignment;
		};

		virtio2_blk.config_space= malloc( sizeof(struct virtio_blk_config_t));
		memset(virtio2_blk.config_space, 0, sizeof(struct virtio_blk_config_t));

		struct virtio_blk_config_t *virtio_blk_config= (struct virtio_blk_config_t*) virtio2_blk.config_space;

		rootfs= fopen("/data/rootfs.ext2", "r");

		fseek(rootfs, 0L, SEEK_END);
		virtio_blk_config->capacity= ftell(rootfs)/ 512;
	}

	{
		uint8_t buff[1024 /*1KB*/];

		FILE *bbl32 = fopen("/data/bbl32.bin", "r");
		for(uint32_t addr= 0x00000000; ; addr+= sizeof(buff)){

			uint16_t reading= fread(buff, sizeof(uint8_t), sizeof(buff), bbl32);

			if (reading == 0) break;

			stream_lru_write(addr, reading, (uint8_t*) &buff);
		}
		fclose(bbl32);

		FILE *image = fopen("/data/Image", "r");
		for(uint32_t addr= 0x00400000; ; addr+= sizeof(buff)){

			uint16_t reading= fread(buff, sizeof(uint8_t), sizeof(buff), image);

			if (reading == 0) break;

			stream_lru_write(addr, reading, (uint8_t*) &buff);
		}
		fclose(image);
	}

	xTaskCreate( vTaskCode, "cpu_loop", 10946, (void *) 0, 1, (void*) 0 );
}
