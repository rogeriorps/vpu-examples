all: vpu-examples

vpu-examples: example1.c
	$(CC) -g -O0 -o example1 example1.c -I /tftpboot/rootfs/usr/src/kernel/include/uapi/ \
	-I /tftpboot/rootfs/usr/src/kernel/include/ -lvpu -lipu -lrt -lpthread

clean:
	-rm example1

.PHONY: all clean
