#!/usr/bin/env python3

import codecs
import os
import sys


class StrFile(object):
    def fread(self, file):
        with codecs.open(file, 'r', encoding='utf-8') as fd:
            for line in fd:
                return line.strip()

    def fwrite_str(self, file, value):
        with codecs.open(file, 'w', encoding='utf-8') as fd:
            fd.write(str(value))


class GPIO(StrFile):
    BASE_PORT   = 906
    FEXPORT     = '/sys/class/gpio/export'
    FUNEXPORT   = '/sys/class/gpio/unexport'
    PORT_FOLDER = '/sys/class/gpio/gpio{port}'

    def __init__(self, port):
        super(GPIO, self).__init__()
        self.port = int(port) + self.BASE_PORT
        self.port_folder = self.PORT_FOLDER.format(port=self.port)
        self.fdirection = self.port_folder + '/direction'
        self.fvalue = self.port_folder + '/value'

    def export(self):
        if not os.path.exists(self.port_folder):
            self.fwrite_str(self.FEXPORT, self.port)

    def unexport(self):
        if os.path.exists(self.port_folder):
            self.fwrite_str(self.FUNEXPORT, self.port)

    def output(self, value):
        self.export()
        self.fwrite_str(self.fdirection, 'out')
        self.fwrite_str(self.fvalue, '1' if value == '1' else '0')

    def input(self):
        self.export()
        self.fwrite_str(self.fdirection, 'in')
        return self.fread(self.fvalue)

if __name__ == '__main__':
    def usage():
        '''
    usage: gpio.py <out|in> <port> [<value]
'''
        print(usage.__doc__)
        raise SystemExit

    argc = len(sys.argv)
    if argc < 3:
        usage()
    if sys.argv[1] == 'out' and argc < 4:
        usage()

    mode = sys.argv[1]
    port = sys.argv[2]
    if mode == 'in':
        value = GPIO(port).input()
        print('PortIn.{port} = {value}'.format(port=port, value=value))
    elif mode == 'out':
        value = sys.argv[3]
        print('PortOut.{port} = {value}'.format(port=port, value=value))
        GPIO(port).output(value)
