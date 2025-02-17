
allall: all

OTHER_HEADERS=
OTHER_OBJS=
TARGET=dcamCtrl

DCAM_PATH=/opt/hamamatsu/dcam/includes/
# LIB_PATH=/

DCAM_INCLUDES=-I$(DCAM_PATH)
DCAM_LIBS = -ldcamapi

INCLUDES += $(DCAM_INCLUDES)
LDLIBS += $(DCAM_LIBS)

include ../../Make/magAOXApp.mk
