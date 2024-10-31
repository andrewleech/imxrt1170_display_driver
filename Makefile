# Name of module (different to built-in btree so it can coexist)
MOD = imxrt1170_display_$(ARCH)

export USERMOD_DIR = .

include micropython.mk

# Architecture to build for (x86, x64, armv7m, xtensa, xtensawin)
ARCH = armv7m

CFLAGS += $(CFLAGS_USERMOD)
LDFLAGS += $(LDFLAGS_USERMOD)
SRC += $(SRC_USERMOD)


#BTREE_DIR = $(MPY_DIR)/lib/berkeley-db-1.xx
#BERKELEY_DB_CONFIG_FILE ?= \"extmod/berkeley-db/berkeley_db_config_port.h\"
#CFLAGS += -I$(BTREE_DIR)/include
#CFLAGS += -DBERKELEY_DB_CONFIG_FILE=$(BERKELEY_DB_CONFIG_FILE)
#CFLAGS += -Wno-old-style-definition -Wno-sign-compare -Wno-unused-parameter

include $(MPY_DIR)/py/dynruntime.mk

