#******************************************************************************
#**  CarMaker - Version 10.0
#**  Vehicle Dynamics Simulation Toolkit
#**
#**  Copyright (C)   IPG Automotive GmbH
#**                  Bannwaldallee 60             Phone  +49.721.98520.0
#**                  76185 Karlsruhe              Fax    +49.721.98520.99
#**                  Germany                      WWW    www.ipg-automotive.com
#******************************************************************************
#**
#**  Application Makefile
#**  Run with 'make V=1 ...' for a verbose build.
#**
#******************************************************************************

include /opt/ipg/carmaker/linux64-10.0.1/include/MakeDefs.linux64

APP_VER =		"Car_Generic with CMRosIF"
APP_NAME =		CarMaker.$(ARCH)$(EXE_EXT)

#OPT_CFLAGS =		-g -O1

LD_LIBS =		$(CAR_LIB) \
			$(CARMAKER_LIB) $(DRIVER_LIB) $(ROAD_LIB) $(TIRE_LIB)
OBJS =			CM_Main.o CM_Vehicle.o User.o rsds-client-camera.o

# CarMaker extension CMRosIF (provided by IPG)
DEF_CFLAGS      += -DWITH_CMROSIF
LD_LIBS_OS      += -lCMRosIF-$(ARCH)
LDFLAGS         += -L./../lib/
LDFLAGS         += -Wl,-rpath,'$$ORIGIN/../lib/'
INC_CFLAGS      += -I../include -I/opt/ros/ros1/include
CFLAGS          += -rdynamic
CXXFLAGS          += -rdynamic

# Prepend local include/library directory to include path:
# PREINC_CFLAGS +=		-I../include -I../lib/$(ARCH) -I../lib

# Append local include/library directory to include path:
# INC_CFLAGS +=		-I../include -I../lib/$(ARCH) -I../lib


### Linking with RTW-built Simulink models

#MATSUPP_MATVER =	R2018a
#LD_LIBS +=		$(MATSUPP_LIB)


# @@PLUGIN-BEGIN-LIBS@@ - Automatically generated code - don't edit!
# @@PLUGIN-END@@

### END (Linking with RTW-built Simulink models)


#default:	$(APP_NAME)
default:	clean $(APP_NAME) install_cp
$(APP_NAME):	$(OBJS_$(ARCH)) $(OBJS) $(LD_LIBS_MK) app_tmp.o
	$(QECHO) " LD     $@"
	$Q $(CXX) $(CXXFLAGS) $(LDFLAGS) -o $@ \
		$(OBJS_$(ARCH)) $(OBJS) $(LD_LIBS) $(LD_LIBS_OS) \
		app_tmp.o
	$(SET_EXE_PERMISSIONS) $@


install: $(APP_NAME)
	$(INSTALL_APP) $(APP_NAME) $(ARCH)

# Warning: Only Workaround for this example!
# - "install" creates copies with timestamp
# - here copy only for development
# - alternatively change CM executable directory
#   -> take care or relative paths for shared libraries!
# - currently no strip, ...
install_cp: $(APP_NAME)
	mkdir -p ./../bin/
	cp -f --parents $(APP_NAME) ./../bin/

clean:
	-rm -f 	*~ *% *.o core

app_tmp.c: Makefile $(OBJS_$(ARCH)) $(OBJS) $(LD_LIBS_MK)
	$(QECHO) " MK     $@"
	$Q $(CREATE_INFO_CMD)

depend .depend: Makefile
	$(QECHO) " MK     $@"
	$Q-$(CC)  $(CFLAGS)   $(DEPCFLAGS)   *.c   >  .depend 2>/dev/null
	$Q-$(CXX) $(CXXFLAGS) $(DEPCXXFLAGS) *.cpp >> .depend 2>/dev/null
include .depend
