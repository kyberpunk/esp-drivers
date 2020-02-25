#
# Main component makefile.
#
# This Makefile can be left empty. By default, it will take the sources in the 
# src/ directory, compile them and link them into lib(subdirectory_name).a 
# in the build directory. This behaviour is entirely configurable,
# please read the ESP-IDF documents if you need to do this.
#
CFLAGS += -Wno-char-subscripts
CFLAGS += -Wno-implicit-function-declaration
CFLAGS += -Wno-maybe-uninitialized
CFLAGS += -Wno-unused-variable
CFLAGS += -Wno-unused-function
CFLAGS += -Wno-type-limits
CFLAGS += -Wno-switch

COMPONENT_SRCDIRS = . drivers/src/ tasks/src/
COMPONENT_ADD_INCLUDEDIRS := include drivers/inc/ tasks/inc/