APP = xplinit fbtest

# Add any other object files to this list below
XPL_OBJS = xplapp.o xplioctl.o vdma.o
FB_OBJS = fbtest.o

CFLAGS  := -Wall -g

all: build

build: $(APP)

xplinit: $(XPL_OBJS)
	$(CC) $(LDFLAGS) -o $@ $(XPL_OBJS) $(LDLIBS)

fbtest: $(FB_OBJS)
	$(CC) $(LDFLAGS) -o $@ $(FB_OBJS) $(LDLIBS)

clean:
	-rm -f $(APP) *.elf *.gdb *.o
