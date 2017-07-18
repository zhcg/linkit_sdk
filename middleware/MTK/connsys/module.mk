
CONNSYS_DIR = middleware/MTK/connsys

C_FILES  += $(CONNSYS_DIR)/src/connsys_bus.c
C_FILES  += $(CONNSYS_DIR)/src/connsys_driver.c
C_FILES  += $(CONNSYS_DIR)/src/connsys_util.c
C_FILES  += $(CONNSYS_DIR)/src/connsys_log.c
C_FILES  += $(CONNSYS_DIR)/src/uart_slim_version.c

######### N9 FW ###########
ifeq ($(PRODUCT_VERSION),7686)
C_FILES  += $(CONNSYS_DIR)/src/mt7686_ram_release_flash.c
else ifeq ($(PRODUCT_VERSION),7682)
C_FILES  += $(CONNSYS_DIR)/src/mt7686_ram_release_flash.c
else ifeq ($(PRODUCT_VERSION),5932)
C_FILES  += $(CONNSYS_DIR)/src/mt7686_ram_release_flash.c
endif

#################################################################################
#include path
#CFLAGS  += -I$(SOURCE_DIR)/$(CONNSYS_DIR)/inc
CFLAGS 	 += -I$(SOURCE_DIR)/middleware/MTK/wifi_service/combo/inc
CFLAGS 	 += -I$(SOURCE_DIR)/middleware/MTK/wifi_stack/inc

