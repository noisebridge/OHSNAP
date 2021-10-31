#!/bin/sh

# create build directory
rm -rf build
mkdir build
cd build

# create an empty flash memory image
touch flash.img
truncate -s 32M flash.img

# program the flash memory
dd if=../boot/u-boot-spl.bin of=flash.img conv=notrunc oflag=seek_bytes seek=0M
dd if=../boot/u-boot.bin     of=flash.img conv=notrunc oflag=seek_bytes seek=1M
dd if=../boot/980uimage      of=flash.img conv=notrunc oflag=seek_bytes seek=2M

# run QEMU
/opt/qemu/bin/qemu-system-arm -s -M chili -bios flash.img -serial stdio
