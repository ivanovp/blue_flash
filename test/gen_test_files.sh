#!/bin/sh
dd if=/dev/urandom of=512KiB.bin bs=1024 count=512
dd if=/dev/urandom of=1MiB.bin bs=1024 count=1024
dd if=/dev/urandom of=2MiB.bin bs=1024 count=2048
dd if=/dev/urandom of=4MiB.bin bs=1024 count=4096
dd if=/dev/urandom of=8MiB.bin bs=1024 count=8192
dd if=/dev/urandom of=16MiB.bin bs=1024 count=16384
dd if=/dev/urandom of=32MiB.bin bs=1024 count=32768
dd if=/dev/urandom of=64MiB.bin bs=1024 count=65536

