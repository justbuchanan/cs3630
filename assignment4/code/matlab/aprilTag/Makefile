CC = gcc
AR = ar

CFLAGS = -std=gnu99 -Wall -Wno-unused-parameter -Wno-unused-function -pthread -I. -Icommon -O1
LDFLAGS = -lpthread -lm

APRILTAG_OBJS = apriltag.o apriltag_quad_thresh.o  tag36h11.o g2d.o zarray.o zhash.o zmaxheap.o unionfind.o matd.o image_u8.o pnm.o image_f32.o image_u32.o workerpool.o time_util.o svd22.o homography.o string_util.o getopt.o

LIBAPRILTAG := libapriltag.a

all: $(LIBAPRILTAG) apriltag_demo


$(LIBAPRILTAG): $(APRILTAG_OBJS)
	@echo "   [$@]"
	@$(AR) -cq $@ $(APRILTAG_OBJS)

apriltag_demo: apriltag_demo.o
	@echo "   [$@]"
	@$(CC) -o $@ apriltag_demo.o $(APRILTAG_OBJS) $(LDFLAGS)

%.o: %.c
	@echo "   $@"
	@$(CC) -o $@ -c $< $(CFLAGS)

clean:
	@rm -rf *.o *.o $(LIBAPRILTAG) apriltag_demo
