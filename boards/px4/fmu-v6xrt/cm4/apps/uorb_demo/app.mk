# uORB bridge layer (lib/uorb_lite.c) and its fault kinds; topic structs
# from the generated headers in the PX4 build directory (PX4_BUILD), which
# pull in uORB/uORB.h from platforms/common.
APP_SRCS     := $(SRCDIR)lib/uorb_lite.c $(SRCDIR)lib/uorb_fault.c
APP_INCLUDES := -I$(SRCDIR)lib -I$(PX4_BUILD) -I$(PX4_SRC)/src/modules/rpmsg_uorb \
                -I$(PX4_SRC)/platforms/common -D__EXPORT=
