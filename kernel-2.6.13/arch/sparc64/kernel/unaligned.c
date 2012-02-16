/* $Id: unaligned.c,v 1.24 2002/02/09 19:49:31 davem Exp $
 * unaligned.c: Unaligned load/store trap handling with special
 *              cases for the kernel to do them more quickly.
 *
 * Copyright (C) 1996 David S. Miller (davem@caip.rutgers.edu)
 * Copyright (C) 1996,1997 Jakub Jelinek (jj@sunsite.mff.cuni.cz)
 */


#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <asm/asi.h>
#include <asm/ptrace.h>
#include <asm/pstate.h>
#include <asm/processor.h>
#include <asm/system.h>
#include <asm/uaccess.h>
#include <linux/smp.h>
#include <linux/smp_lock.h>
#include <linux/bitops.h>
#include <asm/fpumacro.h>

/* #define DEBUG_MNA */

enum direction {
	load,    /* ld, ldd, ldh, ldsh */
	store,   /* st, std, sth, stsh */
	both,    /* Swap, ldstub, cas, ... */
	fpld,
	fpst,
	invalid,
};

#ifdef DEBUG_MNA
static char *dirstrings[] = {
  "load", "store", "both", "fpload", "fpstore", "invalid"
};
#endif

static inline enum direction decode_direction(unsigned int insn)
{
	unsigned long tmp = (insn >> 21) & 1;

	if (!tmp)
		return load;
	else {
		switch ((insn>>19)&0xf) {
		case 15: /* swap* */
			return both;
		default:
			return store;
		}
	}
}

/* 16 = double-word, 8 = extra-word, 4 = word, 2 = half-word */
static inline int decode_access_size(unsigned int insn)
{
	unsigned int tmp;

	tmp = ((insn >> 19) & 0xf);
	if (tmp == 11 || tmp == 14) /* ldx/stx */
		return 8;
	tmp &= 3;
	if (!tmp)
		return 4;
	else if (tmp == 3)
		return 16;	/* ldd/std - Although it is actually 8 */
	else if (tmp == 2)
		return 2;
	else {
		printk("Impossible unaligned trap. insn=%08x\n", insn);
		die_if_kernel("Byte sized unaligned access?!?!", current_thread_info()->kregs);

		/* GCC should never warn that control reaches the end
		 * of this function without returning a value because
		 * die_if_kernel() is marked with attribute 'noreturn'.
		 * Alas, some versions do...
		 */

		return 0;
	}
}

static inline int decode_asi(unsigned int insn, struct pt_regs *regs)
{
	if (insn & 0x800000) {
		if (insn & 0x2000)
			return (unsigned char)(regs->tstate >> 24);	/* %asi */
		else
			return (unsigned char)(insn >> 5);		/* imm_asi */
	} else
		return ASI_P;
}

/* 0x400000 = signed, 0 = unsigned */
static inline int decode_signedness(unsigned int insn)
{
	return (insn & 0x400000);
}

static inline void maybe_flush_windows(unsigned int rs1, unsigned int rs2,
				       unsigned int rd, int from_kernel)
{
	if (rs2 >= 16 || rs1 >= 16 || rd >= 16) {
		if (from_kernel != 0)
			__asm__ __volatile__("flushw");
		else
			flushw_user();
	}
}

static inline long sign_extend_imm13(long imm)
{
	return imm << 51 >> 51;
}

static unsigned long fetch_reg(unsigned int reg, struct pt_regs *regs)
{
	unsigned long value;
	
	if (reg < 16)
		return (!reg ? 0 : regs->u_regs[reg]);
	if (regs->tstate & TSTATE_PRIV) {
		struct reg_window *win;
		win = (struct reg_window *)(regs->u_regs[UREG_FP] + STACK_BIAS);
		value = win->locals[reg - 16];
	} else if (test_thread_flag(TIF_32BIT)) {
		struct reg_window32 __user *win32;
		win32 = (struct reg_window32 __user *)((unsigned long)((u32)regs->u_regs[UREG_FP]));
		get_user(value, &win32->locals[reg - 16]);
	} else {
		struct reg_window __user *win;
		win = (struct reg_window __user *)(regs->u_regs[UREG_FP] + STACK_BIAS);
		get_user(value, &win->locals[reg - 16]);
	}
	return value;
}

static unsigned long *fetch_reg_addr(unsigned int reg, struct pt_regs *regs)
{
	if (reg < 16)
		return &regs->u_regs[reg];
	if (regs->tstate & TSTATE_PRIV) {
		struct reg_window *win;
		win = (struct reg_window *)(regs->u_regs[UREG_FP] + STACK_BIAS);
		return &win->locals[reg - 16];
	} else if (test_thread_flag(TIF_32BIT)) {
		struct reg_window32 *win32;
		win32 = (struct reg_window32 *)((unsigned long)((u32)regs->u_regs[UREG_FP]));
		return (unsigned long *)&win32->locals[reg - 16];
	} else {
		struct reg_window *win;
		win = (struct reg_window *)(regs->u_regs[UREG_FP] + STACK_BIAS);
		return &win->locals[reg - 16];
	}
}

unsigned long compute_effective_address(struct pt_regs *regs,
					unsigned int insn, unsigned int rd)
{
	unsigned int rs1 = (insn >> 14) & 0x1f;
	unsigned int rs2 = insn & 0x1f;
	int from_kernel = (regs->tstate & TSTATE_PRIV) != 0;

	if (insn & 0x2000) {
		maybe_flush_windows(rs1, 0, rd, from_kernel);
		return (fetch_reg(rs1, regs) + sign_extend_imm13(insn));
	} else {
		maybe_flush_windows(rs1, rs2, rd, from_kernel);
		return (fetch_reg(rs1, regs) + fetch_reg(rs2, regs));
	}
}

/* This is just to make gcc think die_if_kernel does return... */
static void __attribute_used__ unaligned_panic(char *str, struct pt_regs *regs)
{
	die_if_kernel(str, regs);
}

extern void do_int_load(unsigned long *dest_reg, int size,
			unsigned long *saddr, int is_signed, int asi);
	
extern void __do_int_store(unsigned long *dst_addr, int size,
			   unsigned long *src_val, int asi);

static inline void do_int_store(int reg_num, int size, unsigned long *dst_addr,
				struct pt_regs *regs, int asi)
{
	unsigned long zero = 0;
	unsigned long *src_val = &zero;

	if (size == 16) {
		size = 8;
		zero = (((long)(reg_num ?
		        (unsigned)fetch_reg(reg_num, regs) : 0)) << 32) |
			(unsigned)fetch_reg(reg_num + 1, regs);
	} else if (reg_num) {
		src_val = fetch_reg_addr(reg_num, regs);
	}
	__do_int_store(dst_addr, size, src_val, asi);
}

static inline void advance(struct pt_regs *regs)
{
	regs->tpc   = regs->tnpc;
	regs->tnpc += 4;
	if (test_thread_flag(TIF_32BIT)) {
		regs->tpc &= 0xffffffff;
		regs->tnpc &= 0xffffffff;
	}
}

static inline int floating_point_load_or_store_p(unsigned int insn)
{
	return (insn >> 24) & 1;
}

static inline int ok_for_kernel(unsigned int insn)
{
	return !floating_point_load_or_store_p(insn);
}

void kernel_mna_trap_fault(void)
{
	struct pt_regs *regs = current_thread_info()->kern_una_regs;
	unsigned int insn = current_thread_info()->kern_una_insn;
	unsigned long g2 = regs->u_regs[UREG_G2];
	unsigned long fixup = search_extables_range(regs->tpc, &g2);

	if (!fixup) {
		unsigned long address;

		address = compute_effective_address(regs, insn,
						    ((insn >> 25) & 0x1f));
        	if (address < PAGE_SIZE) {
                	printk(KERN_ALERT "Unable to handle kernel NULL "
			       "pointer dereference in mna handler");
        	} else
                	printk(KERN_ALERT "Unable to handle kernel paging "
			       "request in mna handler");
	        printk(KERN_ALERT " at virtual address %016lx\n",address);
		printk(KERN_ALERT "current->{active_,}mm->context = %016lx\n",
			(current->mm ? CTX_HWBITS(current->mm->context) :
			CTX_HWBITS(current->active_mm->context)));
		printk(KERN_ALERT "current->{active_,}mm->pgd = %016lx\n",
			(current->mm ? (unsigned long) current->mm->pgd :
			(unsigned long) current->active_mm->pgd));
	        die_if_kernel("Oops", regs);
		/* Not reached */
	}
	regs->tpc = fixup;
	regs->tnpc = regs->tpc + 4;
	regs->u_regs [UREG_G2] = g2;

	regs->tstate &= ~TSTATE_ASI;
	regs->tstate |= (ASI_AIUS << 24UL);
}

asmlinkage void kernel_unaligned_trap(struct pt_regs *regs, unsigned int insn, unsigned long sfar, unsigned long sfsr)
{
	enum direction dir = decode_direction(insn);
	int size = decode_access_size(insn);

	current_thread_info()->kern_una_regs = regs;
	current_thread_info()->kern_una_insn = insn;

	if (!ok_for_kernel(insn) || dir == both) {
		printk("Unsupported unaligned load/store trap for kernel "
		       "at <%016lx>.\n", regs->tpc);
		unaligned_panic("Kernel does fpu/atomic "
				"unaligned load/store.", regs);

		kernel_mna_trap_fault();
	} else {
		unsigned long addr;

		addr = compute_effective_address(regs, insn,
						 ((insn >> 25) & 0x1f));
#ifdef DEBUG_MNA
		printk("KMNA: pc=%016lx [dir=%s addr=%016lx size=%d] "
		       "retpc[%016lx]\n",
		       regs->tpc, dirstrings[dir], addr, size,
		       regs->u_regs[UREG_RETPC]);
#endif
		switch (dir) {
		case load:
			do_int_load(fetch_reg_addr(((insn>>25)&0x1f), regs),
				    size, (unsigned long *) addr,
				    decode_signedness(insn),
				    decode_asi(insn, regs));
			break;

		case store:
			do_int_store(((insn>>25)&0x1f), size,
				     (unsigned long *) addr, regs,
				     decode_asi(insn, regs));
			break;

		default:
			panic("Impossible kernel unaligned trap.");
			/* Not reached... */
		}
		advance(regs);
	}
}

static char popc_helper[] = {
0, 1, 1, 2, 1, 2, 2, 3,
1, 2, 2, 3, 2, 3, 3, 4, 
};

int handle_popc(u32 insn, struct pt_regs *regs)
{
	u64 value;
	int ret, i, rd = ((insn >> 25) & 0x1f);
	int from_kernel = (regs->tstate & TSTATE_PRIV) != 0;
	                        
	if (insn & 0x2000) {
		maybe_flush_windows(0, 0, rd, from_kernel);
		value = sign_extend_imm13(insn);
	} else {
		maybe_flush_windows(0, insn & 0x1f, rd, from_kernel);
		value = fetch_reg(insn & 0x1f, regs);
	}
	for (ret = 0, i = 0; i < 16; i++) {
		ret += popc_helper[value & 0xf];
		value >>= 4;
	}
	if (rd < 16) {
		if (rd)
			regs->u_regs[rd] = ret;
	} else {
		if (test_thread_flag(TIF_32BIT)) {
			struct reg_window32 __user *win32;
			win32 = (struct reg_window32 __user *)((unsigned long)((u32)regs->u_regs[UREG_FP]));
			put_user(ret, &win32->locals[rd - 16]);
		} else {
			struct reg_window __user *win;
			win = (struct reg_window __user *)(regs->u_regs[UREG_FP] + STACK_BIAS);
			put_user(ret, &win->locals[rd - 16]);
		}
	}
	advance(regs);
	return 1;
}

extern void do_fpother(struct pt_regs *regs);
extern void do_privact(struct pt_regs *regs);
extern void data_access_exception(struct pt_regs *regs,
				  unsigned long sfsr,
				  unsigned long sfar);

int handle_ldf_stq(u32 insn, struct pt_regs *regs)
{
	unsigned long addr = compute_effective_address(regs, insn, 0);
	int freg = ((insn >> 25) & 0x1e) | ((insn >> 20) & 0x20);
	struct fpustate *f = FPUSTATE;
	int asi = decode_asi(insn, regs);
	int flag = (freg < 32) ? FPRS_DL : FPRS_DU;

	save_and_clear_fpu();
	current_thread_info()->xfsr[0] &= ~0x1c000;
	if (freg & 3) {
		current_thread_info()->xfsr[0] |= (6 << 14) /* invalid_fp_register */;
		do_fpother(regs);
		return 0;
	}
	if (insn & 0x200000) {
		/* STQ */
		u64 first = 0, second = 0;
		
		if (current_thread_info()->fpsaved[0] & flag) {
			first = *(u64 *)&f->regs[freg];
			second = *(u64 *)&f->regs[freg+2];
		}
		if (asi < 0x80) {
			do_privact(regs);
			return 1;
		}
		switch (asi) {
		case ASI_P:
		case ASI_S: break;
		case ASI_PL:
		case ASI_SL: 
			{
				/* Need to convert endians */
				u64 tmp = __swab64p(&first);
				
				first = __swab64p(&second);
				second = tmp;
				break;
			}
		default:
			data_access_exception(regs, 0, addr);
			return 1;
		}
		if (put_user (first >> 32, (u32 __user *)addr) ||
		    __put_user ((u32)first, (u32 __user *)(addr + 4)) ||
		    __put_user (second >> 32, (u32 __user *)(addr + 8)) ||
		    __put_user ((u32)second, (u32 __user *)(addr + 12))) {
		    	data_access_exception(regs, 0, addr);
		    	return 1;
		}
	} else {
		/* LDF, LDDF, LDQF */
		u32 data[4] __attribute__ ((aligned(8)));
		int size, i;
		int err;

		if (asi < 0x80) {
			do_privact(regs);
			return 1;
		} else if (asi > ASI_SNFL) {
			data_access_exception(regs, 0, addr);
			return 1;
		}
		switch (insn & 0x180000) {
		case 0x000000: size = 1; break;
		case 0x100000: size = 4; break;
		default: size = 2; break;
		}
		for (i = 0; i < size; i++)
			data[i] = 0;
		
		err = get_user (data[0], (u32 __user *) addr);
		if (!err) {
			for (i = 1; i < size; i++)
				err |= __get_user (data[i], (u32 __user *)(addr + 4*i));
		}
		if (err && !(asi & 0x2 /* NF */)) {
			data_access_exception(regs, 0, addr);
			return 1;
		}
		if (asi & 0x8) /* Little */ {
			u64 tmp;

			switch (size) {
			case 1: data[0] = le32_to_cpup(data + 0); break;
			default:*(u64 *)(data + 0) = le64_to_cpup((u64 *)(data + 0));
				break;
			case 4: tmp = le64_to_cpup((u64 *)(data + 0));
				*(u64 *)(data + 0) = le64_to_cpup((u64 *)(data + 2));
				*(u64 *)(data + 2) = tmp;
				break;
			}
		}
		if (!(current_thread_info()->fpsaved[0] & FPRS_FEF)) {
			current_thread_info()->fpsaved[0] = FPRS_FEF;
			current_thread_info()->gsr[0] = 0;
		}
		if (!(current_thread_info()->fpsaved[0] & flag)) {
			if (freg < 32)
				memset(f->regs, 0, 32*sizeof(u32));
			else
				memset(f->regs+32, 0, 32*sizeof(u32));
		}
		memcpy(f->regs + freg, data, size * 4);
		current_thread_info()->fpsaved[0] |= flag;
	}
	advance(regs);
	return 1;
}

void handle_ld_nf(u32 insn, struct pt_regs *regs)
{
	int rd = ((insn >> 25) & 0x1f);
	int from_kernel = (regs->tstate & TSTATE_PRIV) != 0;
	unsigned long *reg;
	                        
	maybe_flush_windows(0, 0, rd, from_kernel);
	reg = fetch_reg_addr(rd, regs);
	if (from_kernel || rd < 16) {
		reg[0] = 0;
		if ((insn & 0x780000) == 0x180000)
			reg[1] = 0;
	} else if (test_thread_flag(TIF_32BIT)) {
		put_user(0, (int __user *) reg);
		if ((insn & 0x780000) == 0x180000)
			put_user(0, ((int __user *) reg) + 1);
	} else {
		put_user(0, (unsigned long __user *) reg);
		if ((insn & 0x780000) == 0x180000)
			put_user(0, (unsigned long __user *) reg + 1);
	}
	advance(regs);
}

void handle_lddfmna(struct pt_regs *regs, unsigned long sfar, unsigned long sfsr)
{
	unsigned long pc = regs->tpc;
	unsigned long tstate = regs->tstate;
	u32 insn;
	u32 first, second;
	u64 value;
	u8 asi, freg;
	int flag;
	struct fpustate *f = FPUSTATE;

	if (tstate & TSTATE_PRIV)
		die_if_kernel("lddfmna from kernel", regs);
	if (test_thread_flag(TIF_32BIT))
		pc = (u32)pc;
	if (get_user(insn, (u32 __user *) pc) != -EFAULT) {
		asi = sfsr >> 16;
		if ((asi > ASI_SNFL) ||
		    (asi < ASI_P))
			goto daex;
		if (get_user(first, (u32 __user *)sfar) ||
		     get_user(second, (u32 __user *)(sfar + 4))) {
			if (asi & 0x2) /* NF */ {
				first = 0; second = 0;
			} else
				goto daex;
		}
		save_and_clear_fpu();
		freg = ((insn >> 25) & 0x1e) | ((insn >> 20) & 0x20);
		value = (((u64)first) << 32) | second;
		if (asi & 0x8) /* Little */
			value = __swab64p(&value);
		flag = (freg < 32) ? FPRS_DL : FPRS_DU;
		if (!(current_thread_info()->fpsaved[0] & FPRS_FEF)) {
			current_thread_info()->fpsaved[0] = FPRS_FEF;
			current_thread_info()->gsr[0] = 0;
		}
		if (!(current_thread_info()->fpsaved[0] & flag)) {
			if (freg < 32)
				memset(f->regs, 0, 32*sizeof(u32));
			else
				memset(f->regs+32, 0, 32*sizeof(u32));
		}
		*(u64 *)(f->regs + freg) = value;
		current_thread_info()->fpsaved[0] |= flag;
	} else {
daex:		data_access_exception(regs, sfsr, sfar);
		return;
	}
	advance(regs);
	return;
}

void handle_stdfmna(struct pt_regs *regs, unsigned long sfar, unsigned long sfsr)
{
	unsigned long pc = regs->tpc;
	unsigned long tstate = regs->tstate;
	u32 insn;
	u64 value;
	u8 asi, freg;
	int flag;
	struct fpustate *f = FPUSTATE;

	if (tstate & TSTATE_PRIV)
		die_if_kernel("stdfmna from kernel", regs);
	if (test_thread_flag(TIF_32BIT))
		pc = (u32)pc;
	if (get_user(insn, (u32 __user *) pc) != -EFAULT) {
		freg = ((insn >> 25) & 0x1e) | ((insn >> 20) & 0x20);
		asi = sfsr >> 16;
		value = 0;
		flag = (freg < 32) ? FPRS_DL : FPRS_DU;
		if ((asi > ASI_SNFL) ||
		    (asi < ASI_P))
			goto daex;
		save_and_clear_fpu();
		if (current_thread_info()->fpsaved[0] & flag)
			value = *(u64 *)&f->regs[freg];
		switch (asi) {
		case ASI_P:
		case ASI_S: break;
		case ASI_PL:
		case ASI_SL: 
			value = __swab64p(&value); break;
		default: goto daex;
		}
		if (put_user (value >> 32, (u32 __user *) sfar) ||
		    __put_user ((u32)value, (u32 __user *)(sfar + 4)))
			goto daex;
	} else {
daex:		data_access_exception(regs, sfsr, sfar);
		return;
	}
	advance(regs);
	return;
}
