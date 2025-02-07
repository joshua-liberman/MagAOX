
allall: all

OTHER_HEADERS=
OTHER_OBJS=
TARGET=dcamCtrl

DCAM_PATH=/opt/hamamatsu/dcam/includes/

DCAM_INCLUDES=-I$(DCAM_PATH)
DCAM_LIBS = -ldcam

INCLUDES += $(DCAM_INCLUDES)
LDLIBS += $(DCAM_LIBS)

include ../../Make/magAOXApp.mk
