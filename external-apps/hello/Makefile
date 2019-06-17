
TARGET  := helloworld
OBJECTS := helloworld.o

CROSS   := arm-linux-gnueabihf-
CC      := $(CROSS)gcc
CXX     := $(CROSS)g++
AS      := $(CROSS)as
LD      := $(CROSS)ld
STRIP   := $(CROSS)strip
OBJDUMP := $(CROSS)objdump
CFLAGS  := -Wall
CFLAGS  += -pipe
CFLAGS  += -D_FILE_OFFSET_BITS=64
LDFLAGS :=



all: $(TARGET)

$(TARGET): $(OBJECTS)
	$(CC) -o $@ $^ $(LDFLAGS)

clean:
	rm -rf $(TARGET) $(OBJECTS)


.PHONY: clean
