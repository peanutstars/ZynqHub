#!/bin/bash

export PATH=$ROOTPATH/../scripts/qt-everywhere-opensource-src-4.8.5/bin:$PATH
DIR_INSTALL=$ROOTPATH/../rootfs/usr/bin

[ ! -e "$DIR_INSTALL" ] && mkdir -p $DIR_INSTALL

qmake -project
qmake
make clean && make && cp -a hello-qt $DIR_INSTALL
