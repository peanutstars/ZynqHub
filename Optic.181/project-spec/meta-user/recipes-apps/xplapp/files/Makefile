APP = xplapp

# Add any other object files to this list below
APP_OBJS = xplapp.o xplioctl.o vdma.o

CFLAGS  := -Wall -g

all: build

build: $(APP)

$(APP): $(APP_OBJS)
	$(CC) $(LDFLAGS) -o $@ $(APP_OBJS) $(LDLIBS)

clean:
	-rm -f $(APP) *.elf *.gdb *.o
