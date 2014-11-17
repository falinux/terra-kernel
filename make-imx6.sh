#!/bin/sh

build_path="../build_linux"
module_install="../out"
firmware_install="../out"
image_filename="$build_path/arch/arm/boot/uImage"
target_filename="uImage.imx6"

if [ -f .config ]; then
	echo ".....mrproper"
	make mrproper
fi

if [ ! -d $build_path ]; then
	mkdir $build_path
	chmod 777 $build_path
fi


#if [ ${PATH%%:*} != "/usr/gcc-linaro-arm-linux-gnueabihf-4.8-2014.03_linux/bin" ]; then
#    echo "$PATH"
#    echo ">> source env.sh <<"
#fi

if [ ! -f $build_path/.config ]; then
	echo ".....imx6 SIB-QCOM defconfig"	
	ARCH=arm make arm=ARM CROSS_COMPILE=arm-linux-gnueabihf- O=$build_path distclean 
	ARCH=arm make arm=ARM CROSS_COMPILE=arm-linux-gnueabihf- O=$build_path imx6_qcom_defconfig
#	echo ".....imx6 SIB-DQ221 defconfig"	
#	ARCH=arm make arm=ARM CROSS_COMPILE=arm-linux-gnueabihf- O=$build_path imx6_dq221_defconfig
#	ARCH=arm make arm=ARM CROSS_COMPILE=arm-linux-gnueabihf- O=$build_path imx6_defconfig
fi


if [ "$1" = "" ]; then
	ARCH=arm make arm=ARM CROSS_COMPILE=arm-linux-gnueabihf- O=$build_path -j4 uImage LOADADDR=0x10008000
else
#       ARCH=arm make O=$build_path $1 $2 $3
        ARCH=arm make arm=ARM CROSS_COMPILE=arm-linux-gnueabihf- O=$build_path $1 $2 $3
fi


# build kernel modules
if [ "$1" = "modules" ] ; then
	ARCH=arm INSTALL_MOD_PATH=$module_install make arm=ARM CROSS_COMPILE=arm-linux-gnueabihf-  O=$build_path modules
	ARCH=arm INSTALL_MOD_PATH=$module_install make arm=ARM CROSS_COMPILE=arm-linux-gnueabihf-  O=$build_path modules_install

#	ARCH=arm INSTALL_MOD_PATH=$firmware_install make arm=ARM CROSS_COMPILE=arm-linux-gnueabihf- O=$build_path firmware
	ARCH=arm INSTALL_MOD_PATH=$firmware_install make arm=ARM CROSS_COMPILE=arm-linux-gnueabihf- O=$build_path firmware_install

#	make -s ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- O=$build_paht firmware_install INSTALL_FW_PATH=../out

fi

if [ -f $image_filename ]; then
   echo "copy from $image_filename to /tftpboot/$target_filename"
   cp  $image_filename /tftpboot/$target_filename
   chmod 777 /tftpboot/$target_filename
fi

