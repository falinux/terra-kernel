1. fusing 방법

   dd if=u-boot.imx of=/dev/sdx bs=512 seek=4096 && sync
   or
   dd if=u-boot.imx of=/dev/mmcblk0 bs=512 seek=4096 && sync

