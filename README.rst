=====
Usage
=====

1. Using U-boot

1.1. Socket

.. code-block:: shell

    ### Host

    $ qemu-system-arm -machine virt,highmem=off -m 512 -nographic -bios u-boot.bin \
        -chardev socket,id=ch0,host=localhost,port=1111,nodelay \
        -device nvme-vpci,chardev=ch0 -serial mon:stdio

    ### Device

    $ qemu-system-arm -M cosmos-plus -nographic -m 1G -kernel CosmosPlusFirmware.elf \
        -chardev socket,id=ch0,host=0.0.0.0,port=1111,server,nodelay \
        -serial mon:stdio -serial chardev:ch0 -s

1.2. Pipe

.. code-block:: shell

    ### make fifo

    $ mkfifo p.in p.out
    $ ln -s p.out q.in
    $ ln -s p.in q.out

    ### Host

    $ qemu-system-arm -machine virt,highmem=off -m 512 -nographic -bios u-boot.bin \
        -chardev pipe,id=ch0,path=./q \
        -device nvme-vpci,chardev=ch0 -serial mon:stdio

    ### Device

    $ qemu-system-arm -M cosmos-plus -nographic -m 1G -kernel CosmosPlusFirmware.elf \
        -chardev pipe,id=ch0,path=./p \
        -serial mon:stdio -serial chardev:ch0 -s


2. Using Linux kernel

.. code-block:: shell

    ### Host

    $ qemu-system-x86_64 -m 2G -smp 2 -nographic -device rtl8139,netdev=net0 -netdev user,id=net0 \
        -drive file=./focal-server-cloudimg-amd64.img,format=qcow2 \
        -chardev socket,id=ch0,host=localhost,port=1111,nodelay \
        -device nvme-vpci,chardev=ch0 -serial mon:stdio

    ### Device

    $ qemu-system-arm -M cosmos-plus -nographic -m 1G -kernel CosmosPlusFirmware.elf \
        -chardev socket,id=ch0,host=0.0.0.0,port=1111,server,nodelay \
        -serial mon:stdio -serial chardev:ch0 -s

