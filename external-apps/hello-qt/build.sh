#!/bin/bash

[ -z "$ZYNQ_QT_VERSION" ] && ZYNQ_QT_VERSION=4.8.7
export PATH=$ROOTPATH/../scripts/qt-everywhere-opensource-src-$ZYNQ_QT_VERSION/bin:$PATH
DIR_INSTALL=$ROOTPATH/../rootfs/usr/bin

[ ! -e "$DIR_INSTALL" ] && mkdir -p $DIR_INSTALL

qmake -project
qmake
make clean && make && cp -a hello-qt $DIR_INSTALL
