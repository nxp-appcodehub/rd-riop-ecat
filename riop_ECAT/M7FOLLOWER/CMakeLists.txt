#
# Copyright 2025 NXP
#
# SPDX-License-Identifier: Apache-2.0
#

cmake_minimum_required(VERSION 3.22.0)

include(
    ${SdkRootDirPath}/cmake/extension/mcux.cmake
)

project(riop_ECAT_M7FOLLOWER LANGUAGES C ASM)


include(
    ${SdkRootDirPath}/CMakeLists.txt
)

set(GOAL_ROOT ../../goal)

include(override.cmake OPTIONAL)

if(NOT RIOP_SKIP_BOARD_FILES)

# board soures
mcux_add_source(
    BASE_PATH ${CMAKE_CURRENT_LIST_DIR}
    SOURCES
    board/board.c
    board/board.h
    board/pin_mux.c
    board/pin_mux.h
    board/clock_config.c
    board/clock_config.h
)

mcux_add_include(
    BASE_PATH ${CMAKE_CURRENT_LIST_DIR}
    INCLUDES
    board
)

mcux_add_configuration(
    TARGETS debug release
    AS " -DRIOP=1 "
    CC " -DRIOP=1 "
)

endif() # NOT RIOP_SKIP_BOARD_FILES

# riop sources
mcux_add_source(
    BASE_PATH ${CMAKE_CURRENT_LIST_DIR}
    SOURCES
    source/afe_task/afe_task.c
    source/afe_task/afe_task.h
    source/api_afe.c
    source/api_afe.h
    source/api_riop.c
    source/api_riop.h
    source/api_riop_common.h
    source/api_siggen.c
    source/api_siggen.h
    source/digital_io.c
    source/gl.h
    source/gpio_task/gpio_task.c
    source/gpio_task/gpio_task.h
    source/icc_task/icc_task.c
    source/icc_task/icc_task.h
    source/riop_feature_config.h
    source/riop_ECAT_M7FOLLOWER.c
    source/SIGGEN_task/SIGGEN_task.c
    source/SIGGEN_task/SIGGEN_task.h
    source/NAFE_hw/edma_permission.c
    source/NAFE_hw/edma_permission.h
    source/NAFE_hw/nafe13388.c
    source/NAFE_hw/nafe13388.h
    source/NAFE_hw/nafe_hal.c
    source/NAFE_hw/nafe_hal.h
)

mcux_add_include(
    BASE_PATH ${CMAKE_CURRENT_LIST_DIR}
    INCLUDES
    .
    source
    source/afe_task
    source/gpio_task
    source/icc_task
    source/SIGGEN_task
    source/NAFE_hw
)

if(NOT RIOP_SKIP_GOAL_APPL_FILES)

# goal appl sources
mcux_add_source(
    BASE_PATH ${CMAKE_CURRENT_LIST_DIR}
    SOURCES
    source/appl/goal_ecat/riop/goal_appl.c
    source/appl/goal_ecat/riop/goal_appl.h
    source/appl/goal_ecat/riop/goal_appl_ecat.c
    source/appl/goal_ecat/riop/goal_appl_ecat.h
    source/appl/goal_ecat/riop/goal_appl_ecat_objects.c
    source/appl/goal_ecat/riop/goal_appl_ecat_objects.h
    source/appl/goal_ecat/riop/goal_config.h
)

mcux_add_include(
    BASE_PATH ${CMAKE_CURRENT_LIST_DIR}
    INCLUDES
    source/appl
    source/appl/goal_ecat
    source/appl/goal_ecat/riop
)

endif() # NOT RIOP_SKIP_GOAL_APPL_FILES


# goal sources
mcux_add_source(
    BASE_PATH ${CMAKE_CURRENT_LIST_DIR}
    SOURCES
    ${GOAL_ROOT}/ext/psnprintf/psnprintf.c
    ${GOAL_ROOT}/ext/psnprintf/psnprintf.h
    ${GOAL_ROOT}/ext/uthash/utlist.h
    ${GOAL_ROOT}/goal/cm/goal_cm_cm.c
    ${GOAL_ROOT}/goal/cm/goal_cm_cm.h
    ${GOAL_ROOT}/goal/cm/goal_eth_cm.c
    ${GOAL_ROOT}/goal/cm/goal_eth_cm.h
    ${GOAL_ROOT}/goal/cm/goal_lm_cm.c
    ${GOAL_ROOT}/goal/cm/goal_lm_cm.h
    ${GOAL_ROOT}/goal/cm/goal_queue_cm.c
    ${GOAL_ROOT}/goal/cm/goal_queue_cm.h
    ${GOAL_ROOT}/goal/goal_alloc.c
    ${GOAL_ROOT}/goal/goal_alloc.h
    ${GOAL_ROOT}/goal/goal_bm.c
    ${GOAL_ROOT}/goal/goal_bm.h
    ${GOAL_ROOT}/goal/goal_bus.h
    ${GOAL_ROOT}/goal/goal_cm.c
    ${GOAL_ROOT}/goal/goal_cm.h
    ${GOAL_ROOT}/goal/goal_cm_id.h
    ${GOAL_ROOT}/goal/goal_cm_t.h
    ${GOAL_ROOT}/goal/goal_config_def.h
    ${GOAL_ROOT}/goal/goal_id.h
    ${GOAL_ROOT}/goal/goal_includes.h
    ${GOAL_ROOT}/goal/goal_inst.c
    ${GOAL_ROOT}/goal/goal_inst.h
    ${GOAL_ROOT}/goal/goal_lm.c
    ${GOAL_ROOT}/goal/goal_lm.h
    ${GOAL_ROOT}/goal/goal_lock.c
    ${GOAL_ROOT}/goal/goal_lock.h
    ${GOAL_ROOT}/goal/goal_log.c
    ${GOAL_ROOT}/goal/goal_log.h
    ${GOAL_ROOT}/goal/goal_main.c
    ${GOAL_ROOT}/goal/goal_main.h
    ${GOAL_ROOT}/goal/goal_mbox.c
    ${GOAL_ROOT}/goal/goal_mbox.h
    ${GOAL_ROOT}/goal/goal_queue.c
    ${GOAL_ROOT}/goal/goal_queue.h
    ${GOAL_ROOT}/goal/goal_rb.c
    ${GOAL_ROOT}/goal/goal_rb.h
    ${GOAL_ROOT}/goal/goal_reg.c
    ${GOAL_ROOT}/goal/goal_reg.h
    ${GOAL_ROOT}/goal/goal_stat.c
    ${GOAL_ROOT}/goal/goal_stat.h
    ${GOAL_ROOT}/goal/goal_target_api.h
    ${GOAL_ROOT}/goal/goal_task.c
    ${GOAL_ROOT}/goal/goal_task.h
    ${GOAL_ROOT}/goal/goal_timer.c
    ${GOAL_ROOT}/goal/goal_timer.h
    ${GOAL_ROOT}/goal/goal_types.h
    ${GOAL_ROOT}/goal/goal_util.c
    ${GOAL_ROOT}/goal/goal_util.h
    ${GOAL_ROOT}/goal_media/cm/goal_mi_mctc_cm.c
    ${GOAL_ROOT}/goal_media/cm/goal_mi_mctc_cm.h
    ${GOAL_ROOT}/goal_media/goal_ma_heap.c
    ${GOAL_ROOT}/goal_media/goal_ma_heap.h
    ${GOAL_ROOT}/goal_media/goal_mi_dm.c
    ${GOAL_ROOT}/goal_media/goal_mi_dm.h
    ${GOAL_ROOT}/goal_media/goal_mi_mctc.c
    ${GOAL_ROOT}/goal_media/goal_mi_mctc.h
    ${GOAL_ROOT}/goal_media/goal_mi_mctc_ram.c
    ${GOAL_ROOT}/goal_media/goal_mi_mctc_ram.h
    ${GOAL_ROOT}/plat/arch/common/goal_arch_common.h
    ${GOAL_ROOT}/plat/arch/common/goal_target_default.h
    ${GOAL_ROOT}/plat/arch/freertos/goal_target.c
    ${GOAL_ROOT}/plat/arch/freertos/goal_target_ext.h
    ${GOAL_ROOT}/plat/arch/rt1180/goal_target.h
    ${GOAL_ROOT}/plat/arch/rt1180/goal_target_common.c
    ${GOAL_ROOT}/plat/arch/rt1180/goal_target_common.h
    ${GOAL_ROOT}/plat/arch/rt1180/goal_target_types.h
    ${GOAL_ROOT}/plat/board/nxp/evkmimxrt1180/goal_target_board.c
    ${GOAL_ROOT}/plat/board/nxp/evkmimxrt1180/goal_target_board.h
    ${GOAL_ROOT}/plat/drv/mem/heap/freertos/mem_heap_freertos.c
    ${GOAL_ROOT}/plat/drv/mem/heap/freertos/mem_heap_freertos.h
    ${GOAL_ROOT}/protos/dd/goal_dd.h
    ${GOAL_ROOT}/protos/dd/rpc/goal_dd_rpc.h
    ${GOAL_ROOT}/protos/dd/rpc/goal_dd_rpc_ac.c
    ${GOAL_ROOT}/protos/dd/rpc/goal_dd_rpc_ac.h
    ${GOAL_ROOT}/protos/ethercat/drivers/goal/ctc/ac/goal_ecat.h
    ${GOAL_ROOT}/protos/ethercat/drivers/goal/ctc/ac/goal_ecat_ac.c
    ${GOAL_ROOT}/protos/ethercat/drivers/goal/ctc/goal_ecat_pdolist.c
    ${GOAL_ROOT}/protos/ethercat/drivers/goal/ctc/goal_ecat_pdolist.h
    ${GOAL_ROOT}/protos/ethercat/drivers/goal/ctc/goal_ecat_rpc.h
    ${GOAL_ROOT}/protos/goal_mctc/plat/goal/goal_mctc_plat.h
    ${GOAL_ROOT}/protos/goal_mctc/src/goal_mctc.c
    ${GOAL_ROOT}/protos/goal_mctc/src/goal_mctc.h
    ${GOAL_ROOT}/protos/goal_mctc/src/goal_mctc_rb.c
    ${GOAL_ROOT}/protos/goal_mctc/src/goal_mctc_rb.h
    ${GOAL_ROOT}/protos/lm_emit/goal_lm_emit.c
    ${GOAL_ROOT}/protos/lm_emit/goal_lm_emit.h
    ${GOAL_ROOT}/protos/lm_emit/goal_lm_emit_raw.c
    ${GOAL_ROOT}/protos/lm_emit/goal_lm_emit_raw.h
)

mcux_add_include(
    BASE_PATH ${CMAKE_CURRENT_LIST_DIR}
    INCLUDES
    .
    ${GOAL_ROOT}/
    ${GOAL_ROOT}/ext/psnprintf
    ${GOAL_ROOT}/ext/uthash
    ${GOAL_ROOT}/goal
    ${GOAL_ROOT}/plat
    ${GOAL_ROOT}/plat/arch/freertos
    ${GOAL_ROOT}/plat/arch/rt1180
    ${GOAL_ROOT}/plat/board/nxp/evkmimxrt1180
    ${GOAL_ROOT}/plat/drv/nvs/sflash/fsl/rt1180
    ${GOAL_ROOT}/plat/drv/nvs/sflash/fsl/rt1180/vendor
    ${GOAL_ROOT}/protos/dd
    ${GOAL_ROOT}/protos/dd/rpc
    ${GOAL_ROOT}/protos/ethercat/drivers/goal/ctc/ac
    ${GOAL_ROOT}/protos/goal_mctc/plat/goal
    ${GOAL_ROOT}/protos/goal_mctc/src
)

mcux_add_configuration(
    TARGETS debug
    # naming no TARGETS will add the elements to all build TARGETs
    # Assembler-Compiler flags
    AS " \
    -D__STARTUP_CLEAR_BSS \
    -D__STARTUP_INITIALIZE_NONCACHEDATA \
    -D__STARTUP_INITIALIZE_QADATA \
    -D__STARTUP_INITIALIZE_RAMFUNCTION \
    -D__STDC_FORMAT_MACROS \
    -D__USE_SHMEM \
    -DCPU_MIMXRT1189CVM8C_cm7 \
    -DGOAL=1 \
    -DGOAL_CONFIG_ECAT_AC=1 \
    -DGOAL_CONFIG_FEAT_UTHASH=1 \
    -DGOAL_CONFIG_GEN_CTC=1 \
    -DGOAL_CONFIG_GEN_CTC_AC=1 \
    -DGOAL_CONFIG_GEN_RPC=1 \
    -DGOAL_CONFIG_LM_EMIT=1 \
    -DGOAL_CONFIG_MCTC=1 \
    -DGOAL_CONFIG_MCTC_AC=1 \
    -DGOAL_CONFIG_MCTC_RAM=1 \
    -DGOAL_CONFIG_MCTC_RPC_DATA_LENGTH=2 \
    -DGOAL_CONFIG_MEDIA_MI_DM=1 \
    -DGOAL_CONFIG_MM_EXT=1 \
    -DGOAL_CONFIG_TARGET_FREERTOS_TARGET_MAIN=1 \
    -DGOAL_CONFIG_TARGET_FREERTOS_TIMER_SUPPORT=1 \
    -DGOAL_CONFIG_TARGET_FREERTOS_TIMER_TICK_FROM_HOOK=1 \
    -DGOAL_CONFIG_TASK=1 \
    -DGOAL_MI_DM_CFG_LOCAL=1 \
    -DMCUX_META_BUILD \
    -DMCUXPRESSO_SDK \
    -DMIMXRT1189_cm7_SERIES \
    -DNDEBUG \
    -DPRINTF_ADVANCED_ENABLE \
    -DSDK_OS_FREE_RTOS \
    -DSERIAL_PORT_TYPE_UART=1 \
    -DUSE_RTOS \
    -DXIP_BOOT_HEADER_ENABLE=0 \
    -DXIP_EXTERNAL_FLASH=1 \
    -fdata-sections \
    -ffreestanding \
    -ffunction-sections \
    -fno-builtin \
    -fno-common \
    -mapcs \
    -mcpu=cortex-m33 \
    -mcpu=cortex-m7 \
    -mfloat-abi=hard \
    -mfpu=fpv5-sp-d16 \
    -MMD \
    -MP \
    -mthumb \
    -O2 \
    -Wno-unused-parameter \
    -DDEBUG \
    -g \
    -O0 \
    "
    # C-Compiler flags
    CC " \
    -D__STARTUP_CLEAR_BSS \
    -D__STDC_FORMAT_MACROS \
    -D__USE_SHMEM \
    -DCPU_MIMXRT1189CVM8C_cm7 \
    -DGOAL=1 \
    -DGOAL_CONFIG_ECAT_AC=1 \
    -DGOAL_CONFIG_FEAT_UTHASH=1 \
    -DGOAL_CONFIG_GEN_CTC=1 \
    -DGOAL_CONFIG_GEN_CTC_AC=1 \
    -DGOAL_CONFIG_GEN_RPC=1 \
    -DGOAL_CONFIG_LM_EMIT=1 \
    -DGOAL_CONFIG_MCTC=1 \
    -DGOAL_CONFIG_MCTC_AC=1 \
    -DGOAL_CONFIG_MCTC_RAM=1 \
    -DGOAL_CONFIG_MCTC_RPC_DATA_LENGTH=2 \
    -DGOAL_CONFIG_MEDIA_MI_DM=1 \
    -DGOAL_CONFIG_MM_EXT=1 \
    -DGOAL_CONFIG_TARGET_FREERTOS_TARGET_MAIN=1 \
    -DGOAL_CONFIG_TARGET_FREERTOS_TIMER_SUPPORT=1 \
    -DGOAL_CONFIG_TARGET_FREERTOS_TIMER_TICK_FROM_HOOK=1 \
    -DGOAL_CONFIG_TASK=1 \
    -DGOAL_MI_DM_CFG_LOCAL=1 \
    -DMCUX_META_BUILD \
    -DMCUXPRESSO_SDK \
    -DMIMXRT1189_cm7_SERIES \
    -DNDEBUG \
    -DPRINTF_ADVANCED_ENABLE \
    -DSDK_OS_FREE_RTOS \
    -DSERIAL_PORT_TYPE_UART=1 \
    -DUSE_RTOS \
    -DXIP_BOOT_HEADER_ENABLE=0 \
    -DXIP_EXTERNAL_FLASH=1 \
    -fdata-sections \
    -ffreestanding \
    -ffunction-sections \
    -fno-builtin \
    -fno-common \
    -mapcs \
    -mcpu=cortex-m7 \
    -mfloat-abi=hard \
    -mfpu=fpv5-sp-d16 \
    -MMD \
    -MP \
    -mthumb \
    -O2 \
    -std=c99 \
    -DDEBUG \
    -g \
    -O0 \
    "
)

mcux_add_configuration(
    TARGETS release
    # naming no TARGETS will add the elements to all build TARGETs
    # Assembler-Compiler flags
    AS " \
    -D__STARTUP_CLEAR_BSS \
    -D__STARTUP_INITIALIZE_NONCACHEDATA \
    -D__STARTUP_INITIALIZE_QADATA \
    -D__STARTUP_INITIALIZE_RAMFUNCTION \
    -D__STDC_FORMAT_MACROS \
    -D__USE_SHMEM \
    -DCPU_MIMXRT1189CVM8C_cm7 \
    -DGOAL=1 \
    -DGOAL_CONFIG_ECAT_AC=1 \
    -DGOAL_CONFIG_FEAT_UTHASH=1 \
    -DGOAL_CONFIG_GEN_CTC=1 \
    -DGOAL_CONFIG_GEN_CTC_AC=1 \
    -DGOAL_CONFIG_GEN_RPC=1 \
    -DGOAL_CONFIG_LM_EMIT=1 \
    -DGOAL_CONFIG_MCTC=1 \
    -DGOAL_CONFIG_MCTC_AC=1 \
    -DGOAL_CONFIG_MCTC_RAM=1 \
    -DGOAL_CONFIG_MCTC_RPC_DATA_LENGTH=2 \
    -DGOAL_CONFIG_MEDIA_MI_DM=1 \
    -DGOAL_CONFIG_MM_EXT=1 \
    -DGOAL_CONFIG_TARGET_FREERTOS_TARGET_MAIN=1 \
    -DGOAL_CONFIG_TARGET_FREERTOS_TIMER_SUPPORT=1 \
    -DGOAL_CONFIG_TARGET_FREERTOS_TIMER_TICK_FROM_HOOK=1 \
    -DGOAL_CONFIG_TASK=1 \
    -DGOAL_MI_DM_CFG_LOCAL=1 \
    -DMCUX_META_BUILD \
    -DMCUXPRESSO_SDK \
    -DMIMXRT1189_cm7_SERIES \
    -DNDEBUG \
    -DPRINTF_ADVANCED_ENABLE \
    -DSDK_OS_FREE_RTOS \
    -DSERIAL_PORT_TYPE_UART=1 \
    -DUSE_RTOS \
    -DXIP_BOOT_HEADER_ENABLE=0 \
    -DXIP_EXTERNAL_FLASH=1 \
    -fdata-sections \
    -ffreestanding \
    -ffunction-sections \
    -fno-builtin \
    -fno-common \
    -mapcs \
    -mcpu=cortex-m33 \
    -mcpu=cortex-m7 \
    -mfloat-abi=hard \
    -mfpu=fpv5-sp-d16 \
    -MMD \
    -MP \
    -mthumb \
    -O2 \
    -Wno-unused-parameter \
    "
    # C-Compiler flags
    CC " \
    -D__STARTUP_CLEAR_BSS \
    -D__STDC_FORMAT_MACROS \
    -D__USE_SHMEM \
    -DCPU_MIMXRT1189CVM8C_cm7 \
    -DGOAL=1 \
    -DGOAL_CONFIG_ECAT_AC=1 \
    -DGOAL_CONFIG_FEAT_UTHASH=1 \
    -DGOAL_CONFIG_GEN_CTC=1 \
    -DGOAL_CONFIG_GEN_CTC_AC=1 \
    -DGOAL_CONFIG_GEN_RPC=1 \
    -DGOAL_CONFIG_LM_EMIT=1 \
    -DGOAL_CONFIG_MCTC=1 \
    -DGOAL_CONFIG_MCTC_AC=1 \
    -DGOAL_CONFIG_MCTC_RAM=1 \
    -DGOAL_CONFIG_MCTC_RPC_DATA_LENGTH=2 \
    -DGOAL_CONFIG_MEDIA_MI_DM=1 \
    -DGOAL_CONFIG_MM_EXT=1 \
    -DGOAL_CONFIG_TARGET_FREERTOS_TARGET_MAIN=1 \
    -DGOAL_CONFIG_TARGET_FREERTOS_TIMER_SUPPORT=1 \
    -DGOAL_CONFIG_TARGET_FREERTOS_TIMER_TICK_FROM_HOOK=1 \
    -DGOAL_CONFIG_TASK=1 \
    -DGOAL_MI_DM_CFG_LOCAL=1 \
    -DMCUX_META_BUILD \
    -DMCUXPRESSO_SDK \
    -DMIMXRT1189_cm7_SERIES \
    -DNDEBUG \
    -DPRINTF_ADVANCED_ENABLE \
    -DSDK_OS_FREE_RTOS \
    -DSERIAL_PORT_TYPE_UART=1 \
    -DUSE_RTOS \
    -DXIP_BOOT_HEADER_ENABLE=0 \
    -DXIP_EXTERNAL_FLASH=1 \
    -fdata-sections \
    -ffreestanding \
    -ffunction-sections \
    -fno-builtin \
    -fno-common \
    -mapcs \
    -mcpu=cortex-m7 \
    -mfloat-abi=hard \
    -mfpu=fpv5-sp-d16 \
    -MMD \
    -MP \
    -mthumb \
    -O2 \
    -std=c99 \
    "
)
# Linker flags for shared memory
mcux_add_linker_symbol(
    TARGETS debug release
    SYMBOLS " \
    __use_shmem__=1 \
    __multicore__=1 \
    "
)

mcux_convert_binary(
    TOOLCHAINS armgcc
    BINARY ${APPLICATION_BINARY_DIR}/${CONFIG_TOOLCHAIN}/core1_image.bin
)

# replace linker script for debug configuration
mcux_remove_armgcc_linker_script(
    TARGETS debug release
    BASE_PATH ${SdkRootDirPath}
    LINKER ${device_root}/RT/RT1180/MIMXRT1189/gcc/MIMXRT1189xxxxx_cm7_ram.ld
)

mcux_add_armgcc_linker_script(
    TARGETS debug release
    BASE_PATH ${CMAKE_CURRENT_LIST_DIR}
    LINKER armgcc/MIMXRT1189xxxxx_cm7_ram.ld
)

mcux_convert_binary(BINARY ${APPLICATION_BINARY_DIR}/${MCUX_SDK_PROJECT_NAME}.bin)
