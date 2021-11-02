#!/bin/sh

# chdir into script directory
cd `dirname $0`

# create build directory
rm -rf build
mkdir build
cd build

# create an empty flash memory image
touch flash.img
truncate -s 32M flash.img

# program the flash memory
dd if=../boot/u-boot-spl.bin of=flash.img conv=notrunc oflag=seek_bytes seek=0K     2>/dev/null
dd if=../boot/u-boot-env.bin of=flash.img conv=notrunc oflag=seek_bytes seek=512K   2>/dev/null
dd if=../boot/u-boot.bin     of=flash.img conv=notrunc oflag=seek_bytes seek=1024K  2>/dev/null
dd if=../boot/980uimage      of=flash.img conv=notrunc oflag=seek_bytes seek=2048K  2>/dev/null
dd if=../boot/uInitrd        of=flash.img conv=notrunc oflag=seek_bytes seek=16M    2>/dev/null

# echo heading
echo ''
echo '+------------------------------------------------------------+'
echo '|                                                            |'
echo '|                OHSNAP QEMU-BASED EMULATOR                  |'
echo '|                                                            |'
echo '+------------------------------------------------------------+'

# run QEMU
/opt/qemu/bin/qemu-system-arm -s -M nuc980-soc -m 64M -bios flash.img -serial stdio -monitor none -nographic

