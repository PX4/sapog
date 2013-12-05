
canas_dir_src := canaerospace/canaerospace/src/

CANAS_SRC := $(canas_dir_src)core.c    \
             $(canas_dir_src)list.c    \
             $(canas_dir_src)marshal.c \
             $(canas_dir_src)service.c \
             $(canas_dir_src)util.c    \
             $(canas_dir_src)services/std_identification.c

CANAS_INC := canaerospace/canaerospace/include
CANAS_DEF := -DCANAEROSPACE_DEBUG=1
