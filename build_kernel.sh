#!/bin/bash

#Set CPU Environment Variable
if [ "$CPU_JOB_NUM" = "" ] ; then
        CPU_JOB_NUM=8
fi

Usage()
{
echo "build_kernel.sh - building script android kernel"
echo "  Usage: ./build_kernel.sh "
echo

exit 1
}

OPTION=-k
PRODUCT=r880

case "$PRODUCT" in

    r880)		
                MODULES_FAST="g2d g3d mfc jpeg cmm okmfc rotator"
                MODULES_STABLE="vibetonz bcm4325 btgpio camera dpram multipdp param pp rfs wlan xsr compcache"
                KERNEL_DEF_CONFIG=spica_android_defconfig
                ;;
    
	*)
		Usage
		;;
esac 

if [ ! $PWD_DIR ] ; then
	PWD_DIR=$(pwd)
fi

KERNEL_DIR=$PWD_DIR
MODULES_DIR=$PWD_DIR/../modules
CTNG_BIN_DIR=/usr/bin/

prepare_kernel()
{
	echo "*************************************"
	echo "*          prepare kernel           *"
	echo "*************************************"
	echo

	make -C $KERNEL_DIR ARCH=arm $KERNEL_DEF_CONFIG 
	if [ $? != 0 ] ; then
	    exit 1
	fi
	make -C $KERNEL_DIR ARCH=arm prepare 
	if [ $? != 0 ] ; then
	    exit 1
	fi
}

build_modules_stable()
{
    echo "*************************************"
    echo "*           build modules | stable  *"
    echo "*************************************"
    echo

    make -C $KERNEL_DIR ARCH=arm $KERNEL_DEF_CONFIG 
    if [ $? != 0 ] ; then
        exit 1
    fi
    make -C $KERNEL_DIR ARCH=arm KBUILD_MODPOST_WARN=1 modules 
    if [ $? != 0 ] ; then
        exit 1
    fi

    for module in $MODULES_STABLE
    do
        echo cd $MODULES_DIR/$module
        cd $MODULES_DIR/$module
        make KDIR=$KERNEL_DIR 
        if [ -e ./*.ko ]
        then
            cp ./*.ko  $KERNEL_DIR/../initramfs/lib/modules
        fi
    done

}


build_modules_fast()
{
	echo "*************************************"
	echo "*           build modules | fast    *"
	echo "*************************************"
	echo

	make -C $KERNEL_DIR ARCH=arm $KERNEL_DEF_CONFIG	CFLAGS="-Ofast \
                -marm \
                -march=armv6zk \
                -mtune=arm1176jzf-s \
                -mfpu=vfp \
                -mfloat-abi=softfp \
                -floop-interchange \
                -floop-strip-mine \
                -floop-block \
                -funsafe-loop-optimizations \
                -funsafe-math-optimizations \
                --param l1-cache-size=16 \
                --param l1-cache-line-size=32 \
                --param simultaneous-prefetches=6 \
                --param prefetch-latency=400"
	if [ $? != 0 ] ; then
	    exit 1
	fi
	make -C $KERNEL_DIR ARCH=arm KBUILD_MODPOST_WARN=1 modules CFLAGS="-Ofast \
                -marm \
                -march=armv6zk \
                -mtune=arm1176jzf-s \
                -mfpu=vfp \
                -mfloat-abi=softfp \
                -floop-interchange \
                -floop-strip-mine \
                -floop-block \
                -funsafe-loop-optimizations \
                -funsafe-math-optimizations \
                --param l1-cache-size=16 \
                --param l1-cache-line-size=32 \
                --param simultaneous-prefetches=6 \
                --param prefetch-latency=400"
	if [ $? != 0 ] ; then
	    exit 1
	fi

	for module in $MODULES_fast
	do
		echo cd $MODULES_DIR/$module
		cd $MODULES_DIR/$module
		make KDIR=$KERNEL_DIR CFLAGS="-Ofast \
                -marm \
                -march=armv6zk \
                -mtune=arm1176jzf-s \
                -mfpu=vfp \
                -mfloat-abi=softfp \
                -floop-interchange \
                -floop-strip-mine \
                -floop-block \
                -funsafe-loop-optimizations \
                -funsafe-math-optimizations \
                --param l1-cache-size=16 \
                --param l1-cache-line-size=32 \
                --param simultaneous-prefetches=6 \
                --param prefetch-latency=400"
		if [ -e ./*.ko ]
		then
		    cp ./*.ko  $KERNEL_DIR/../initramfs/lib/modules
		fi
	done 

}


build_kernel()
{
	if [ ! -f $KERNEL_DIR/.config ] ; then
		if [ ! -f $KERNEL_DIR/scripts/mod/modprobe ] ; then
			prepare_kernel
		fi
	fi

	#echo "make " -C $KERNEL_DIR ARCH=arm $KERNEL_DEF_CONFIG
	make -C $KERNEL_DIR ARCH=arm $KERNEL_DEF_CONFIG CFLAGS="-Ofast \
                -marm \
                -march=armv6zk \
                -mtune=arm1176jzf-s \
                -mfpu=vfp \
                -mfloat-abi=softfp \
                -floop-interchange \
                -floop-strip-mine \
                -floop-block \
                -funsafe-loop-optimizations \
                -funsafe-math-optimizations \
                --param l1-cache-size=16 \
                --param l1-cache-line-size=32 \
                --param simultaneous-prefetches=6 \
                --param prefetch-latency=400"
	if [ $? != 0 ] ; then
	    exit 1
	fi

	build_modules_fast
	build_modules_stable

	echo "*************************************"
	echo "*           build kernel            *"
	echo "*************************************"
	echo
	
	cd $KERNEL_DIR

	#make -j$CPU_JOB_NUM CONFIG_DEBUG_SECTION_MISMATCH=y
	make -j$CPU_JOB_NUM 
	if [ $? != 0 ] ; then
		exit $?
	fi

	$CTNG_BIN_DIR/arm-linux-gnueabi-strip -g $KERNEL_DIR/../initramfs/lib/modules/*
 
	make 
}

case "$OPTION" in
	-k)
		build_kernel
		;;
	*)
		Usage
		;;
esac 

exit 0
