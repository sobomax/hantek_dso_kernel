# 
# Copyright (C) 2002 - 2004 Jeff Dike (jdike@addtoit.com)
# Licensed under the GPL
#

obj-y := clone.o exec_kern.o mem.o mem_user.o mmu.o process.o process_kern.o \
	syscall_kern.o syscall_user.o tlb.o trap_user.o uaccess.o \

subdir- := util

USER_OBJS := process.o clone.o

include arch/um/scripts/Makefile.rules

# clone.o is in the stub, so it can't be built with profiling
$(obj)/clone.o : c_flags = -Wp,-MD,$(depfile) $(call unprofile,$(USER_CFLAGS))
