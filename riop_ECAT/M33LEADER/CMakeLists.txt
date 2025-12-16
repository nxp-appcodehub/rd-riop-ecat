#
# Copyright 2025 NXP
#
# SPDX-License-Identifier: Apache-2.0
#

cmake_minimum_required(VERSION 3.22.0)

include(
    ${SdkRootDirPath}/cmake/extension/mcux.cmake
)

project(riop_ECAT_M33LEADER LANGUAGES C ASM)


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
    board/ecat_hw.c
)

mcux_add_include(
    BASE_PATH ${CMAKE_CURRENT_LIST_DIR}
    INCLUDES
    board
)

mcux_add_configuration(
    TARGETS debug release flexspi_nor_debug flexspi_nor_release
    AS " -DRIOP=1 "
    CC " -DRIOP=1 "
)

endif() # NOT RIOP_SKIP_BOARD_FILES


# riop sources
mcux_add_source(
    BASE_PATH ${CMAKE_CURRENT_LIST_DIR}
    SOURCES
    source/riop_ECAT_M33LEADER.c
    source/semihost_hardfault.c
    source/flash/flexspi_flash_cfg.c
    source/flash/flexspi_nor_flash_ops.c
)

mcux_add_include(
    BASE_PATH ${CMAKE_CURRENT_LIST_DIR}
    INCLUDES
    .
    board
    source
    source/flash
)

# goal sources
mcux_add_source(
    BASE_PATH ${CMAKE_CURRENT_LIST_DIR}
    SOURCES
    ${GOAL_ROOT}/appl/goal_ecat/00_rpc_cc/goal_appl.c
    ${GOAL_ROOT}/appl/goal_ecat/00_rpc_cc/goal_config.h
    ${GOAL_ROOT}/ext/base64/include/b64/cdecode.h
    ${GOAL_ROOT}/ext/base64/include/b64/cencode.h
    ${GOAL_ROOT}/ext/base64/src/cdecode.c
    ${GOAL_ROOT}/ext/base64/src/cencode.c
    ${GOAL_ROOT}/ext/lwip/src/core/def.c
    ${GOAL_ROOT}/ext/lwip/src/core/dns.c
    ${GOAL_ROOT}/ext/lwip/src/core/inet_chksum.c
    ${GOAL_ROOT}/ext/lwip/src/core/init.c
    ${GOAL_ROOT}/ext/lwip/src/core/ip.c
    ${GOAL_ROOT}/ext/lwip/src/core/ipv4/etharp.c
    ${GOAL_ROOT}/ext/lwip/src/core/ipv4/icmp.c
    ${GOAL_ROOT}/ext/lwip/src/core/ipv4/igmp.c
    ${GOAL_ROOT}/ext/lwip/src/core/ipv4/ip4.c
    ${GOAL_ROOT}/ext/lwip/src/core/ipv4/ip4_addr.c
    ${GOAL_ROOT}/ext/lwip/src/core/ipv4/ip4_frag.c
    ${GOAL_ROOT}/ext/lwip/src/core/mem.c
    ${GOAL_ROOT}/ext/lwip/src/core/memp.c
    ${GOAL_ROOT}/ext/lwip/src/core/netif.c
    ${GOAL_ROOT}/ext/lwip/src/core/pbuf.c
    ${GOAL_ROOT}/ext/lwip/src/core/tcp.c
    ${GOAL_ROOT}/ext/lwip/src/core/tcp_in.c
    ${GOAL_ROOT}/ext/lwip/src/core/tcp_out.c
    ${GOAL_ROOT}/ext/lwip/src/core/timeouts.c
    ${GOAL_ROOT}/ext/lwip/src/core/udp.c
    ${GOAL_ROOT}/ext/lwip/src/include/lwip/altcp.h
    ${GOAL_ROOT}/ext/lwip/src/include/lwip/api.h
    ${GOAL_ROOT}/ext/lwip/src/include/lwip/arch.h
    ${GOAL_ROOT}/ext/lwip/src/include/lwip/autoip.h
    ${GOAL_ROOT}/ext/lwip/src/include/lwip/debug.h
    ${GOAL_ROOT}/ext/lwip/src/include/lwip/def.h
    ${GOAL_ROOT}/ext/lwip/src/include/lwip/dhcp.h
    ${GOAL_ROOT}/ext/lwip/src/include/lwip/dns.h
    ${GOAL_ROOT}/ext/lwip/src/include/lwip/err.h
    ${GOAL_ROOT}/ext/lwip/src/include/lwip/etharp.h
    ${GOAL_ROOT}/ext/lwip/src/include/lwip/icmp.h
    ${GOAL_ROOT}/ext/lwip/src/include/lwip/icmp6.h
    ${GOAL_ROOT}/ext/lwip/src/include/lwip/igmp.h
    ${GOAL_ROOT}/ext/lwip/src/include/lwip/inet_chksum.h
    ${GOAL_ROOT}/ext/lwip/src/include/lwip/init.h
    ${GOAL_ROOT}/ext/lwip/src/include/lwip/ip.h
    ${GOAL_ROOT}/ext/lwip/src/include/lwip/ip4.h
    ${GOAL_ROOT}/ext/lwip/src/include/lwip/ip4_addr.h
    ${GOAL_ROOT}/ext/lwip/src/include/lwip/ip4_frag.h
    ${GOAL_ROOT}/ext/lwip/src/include/lwip/ip6.h
    ${GOAL_ROOT}/ext/lwip/src/include/lwip/ip6_addr.h
    ${GOAL_ROOT}/ext/lwip/src/include/lwip/ip6_frag.h
    ${GOAL_ROOT}/ext/lwip/src/include/lwip/ip_addr.h
    ${GOAL_ROOT}/ext/lwip/src/include/lwip/mem.h
    ${GOAL_ROOT}/ext/lwip/src/include/lwip/memp.h
    ${GOAL_ROOT}/ext/lwip/src/include/lwip/mld6.h
    ${GOAL_ROOT}/ext/lwip/src/include/lwip/nd6.h
    ${GOAL_ROOT}/ext/lwip/src/include/lwip/netbuf.h
    ${GOAL_ROOT}/ext/lwip/src/include/lwip/netdb.h
    ${GOAL_ROOT}/ext/lwip/src/include/lwip/netif.h
    ${GOAL_ROOT}/ext/lwip/src/include/lwip/netifapi.h
    ${GOAL_ROOT}/ext/lwip/src/include/lwip/opt.h
    ${GOAL_ROOT}/ext/lwip/src/include/lwip/pbuf.h
    ${GOAL_ROOT}/ext/lwip/src/include/lwip/priv/api_msg.h
    ${GOAL_ROOT}/ext/lwip/src/include/lwip/priv/memp_priv.h
    ${GOAL_ROOT}/ext/lwip/src/include/lwip/priv/memp_std.h
    ${GOAL_ROOT}/ext/lwip/src/include/lwip/priv/nd6_priv.h
    ${GOAL_ROOT}/ext/lwip/src/include/lwip/priv/sockets_priv.h
    ${GOAL_ROOT}/ext/lwip/src/include/lwip/priv/tcp_priv.h
    ${GOAL_ROOT}/ext/lwip/src/include/lwip/priv/tcpip_priv.h
    ${GOAL_ROOT}/ext/lwip/src/include/lwip/prot/dhcp.h
    ${GOAL_ROOT}/ext/lwip/src/include/lwip/prot/dns.h
    ${GOAL_ROOT}/ext/lwip/src/include/lwip/prot/etharp.h
    ${GOAL_ROOT}/ext/lwip/src/include/lwip/prot/ethernet.h
    ${GOAL_ROOT}/ext/lwip/src/include/lwip/prot/icmp.h
    ${GOAL_ROOT}/ext/lwip/src/include/lwip/prot/icmp6.h
    ${GOAL_ROOT}/ext/lwip/src/include/lwip/prot/igmp.h
    ${GOAL_ROOT}/ext/lwip/src/include/lwip/prot/ip.h
    ${GOAL_ROOT}/ext/lwip/src/include/lwip/prot/ip4.h
    ${GOAL_ROOT}/ext/lwip/src/include/lwip/prot/tcp.h
    ${GOAL_ROOT}/ext/lwip/src/include/lwip/prot/udp.h
    ${GOAL_ROOT}/ext/lwip/src/include/lwip/raw.h
    ${GOAL_ROOT}/ext/lwip/src/include/lwip/snmp.h
    ${GOAL_ROOT}/ext/lwip/src/include/lwip/sockets.h
    ${GOAL_ROOT}/ext/lwip/src/include/lwip/stats.h
    ${GOAL_ROOT}/ext/lwip/src/include/lwip/sys.h
    ${GOAL_ROOT}/ext/lwip/src/include/lwip/tcp.h
    ${GOAL_ROOT}/ext/lwip/src/include/lwip/tcpbase.h
    ${GOAL_ROOT}/ext/lwip/src/include/lwip/timeouts.h
    ${GOAL_ROOT}/ext/lwip/src/include/lwip/udp.h
    ${GOAL_ROOT}/ext/lwip/src/include/netif/etharp.h
    ${GOAL_ROOT}/ext/lwip/src/include/netif/ethernet.h
    ${GOAL_ROOT}/ext/lwip/src/include/netif/ppp/ppp_impl.h
    ${GOAL_ROOT}/ext/lwip/src/include/netif/ppp/ppp_opts.h
    ${GOAL_ROOT}/ext/lwip/src/netif/ethernet.c
    ${GOAL_ROOT}/ext/psnprintf/psnprintf.h
    ${GOAL_ROOT}/ext/uthash/utlist.h
    ${GOAL_ROOT}/goal/cm/goal_cm_cm.h
    ${GOAL_ROOT}/goal/cm/goal_eth_cm.h
    ${GOAL_ROOT}/goal/cm/goal_lm_cm.h
    ${GOAL_ROOT}/goal/cm/goal_net_cm.h
    ${GOAL_ROOT}/goal/cm/goal_queue_cm.h
    ${GOAL_ROOT}/goal/goal_alloc.h
    ${GOAL_ROOT}/goal/goal_bm.h
    ${GOAL_ROOT}/goal/goal_bus.h
    ${GOAL_ROOT}/goal/goal_cm.h
    ${GOAL_ROOT}/goal/goal_cm_id.h
    ${GOAL_ROOT}/goal/goal_cm_t.h
    ${GOAL_ROOT}/goal/goal_config_def.h
    ${GOAL_ROOT}/goal/goal_eth_names.h
    ${GOAL_ROOT}/goal/goal_id.h
    ${GOAL_ROOT}/goal/goal_includes.h
    ${GOAL_ROOT}/goal/goal_inst.h
    ${GOAL_ROOT}/goal/goal_lm.h
    ${GOAL_ROOT}/goal/goal_lock.h
    ${GOAL_ROOT}/goal/goal_log.h
    ${GOAL_ROOT}/goal/goal_main.h
    ${GOAL_ROOT}/goal/goal_mbox.h
    ${GOAL_ROOT}/goal/goal_net_chan.h
    ${GOAL_ROOT}/goal/goal_queue.h
    ${GOAL_ROOT}/goal/goal_rb.h
    ${GOAL_ROOT}/goal/goal_reg.h
    ${GOAL_ROOT}/goal/goal_stat.h
    ${GOAL_ROOT}/goal/goal_target_api.h
    ${GOAL_ROOT}/goal/goal_task.h
    ${GOAL_ROOT}/goal/goal_timer.h
    ${GOAL_ROOT}/goal/goal_types.h
    ${GOAL_ROOT}/goal/goal_util.h
    ${GOAL_ROOT}/goal_media/goal_ma_eth.h
    ${GOAL_ROOT}/goal_media/goal_ma_heap.h
    ${GOAL_ROOT}/goal_media/goal_ma_net.h
    ${GOAL_ROOT}/goal_media/goal_mi_eth.h
    ${GOAL_ROOT}/goal_media/goal_mi_net.h
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
    ${GOAL_ROOT}/plat/drv/bus/mdio/esc/mdio_esc.c
    ${GOAL_ROOT}/plat/drv/bus/mdio/esc/mdio_esc.h
    ${GOAL_ROOT}/plat/drv/bus/mdio/netc/mdio.c
    ${GOAL_ROOT}/plat/drv/bus/mdio/netc/mdio.h
    ${GOAL_ROOT}/plat/drv/eoe/goal_drv_eoe.c
    ${GOAL_ROOT}/plat/drv/eoe/goal_drv_eoe.h
    ${GOAL_ROOT}/plat/drv/eth/nxp/netc/goal_target_eth.c
    ${GOAL_ROOT}/plat/drv/eth/nxp/netc/goal_target_eth.h
    ${GOAL_ROOT}/plat/drv/mem/heap/freertos/mem_heap_freertos.c
    ${GOAL_ROOT}/plat/drv/mem/heap/freertos/mem_heap_freertos.h
    ${GOAL_ROOT}/plat/drv/net/lwip/goal_target_net.c
    ${GOAL_ROOT}/plat/drv/net/lwip/vendor/arch/cc.h
    ${GOAL_ROOT}/plat/drv/net/lwip/vendor/lwipopts.h
    ${GOAL_ROOT}/plat/drv/nvs/sflash/fsl/rt1180/nvs_sflash_fsl.c
    ${GOAL_ROOT}/plat/drv/phy/generic/phy_generic.c
    ${GOAL_ROOT}/plat/drv/phy/generic/phy_generic.h
    ${GOAL_ROOT}/plat/drv/phy/generic/phy_generic_int.h
    ${GOAL_ROOT}/plat/drv/phy/realtek_rtl8201/phy_rtl8201.c
    ${GOAL_ROOT}/plat/drv/phy/realtek_rtl8201/phy_rtl8201.h
    ${GOAL_ROOT}/plat/drv/phy/realtek_rtl8211/phy_rtl8211.c
    ${GOAL_ROOT}/plat/drv/phy/realtek_rtl8211/phy_rtl8211.h
    ${GOAL_ROOT}/plat/drv/switch/nxp_netc/nxp_netc.c
    ${GOAL_ROOT}/plat/drv/switch/nxp_netc/nxp_netc_bsp.c
    ${GOAL_ROOT}/projects/goal_ecat_rpc_lib/00_cc/edt/nxp/evkmimxrt1180/ecat_conf.h
    ${GOAL_ROOT}/protos/dd/cm/goal_dd_cm.h
    ${GOAL_ROOT}/protos/dd/goal_dd.h
    ${GOAL_ROOT}/protos/dd/goal_dd_config.h
    ${GOAL_ROOT}/protos/dd/goal_dd_crypt.h
    ${GOAL_ROOT}/protos/dd/goal_dd_getconf.h
    ${GOAL_ROOT}/protos/dd/goal_dd_getlist.h
    ${GOAL_ROOT}/protos/dd/goal_dd_hello.h
    ${GOAL_ROOT}/protos/dd/goal_dd_packet.h
    ${GOAL_ROOT}/protos/dd/goal_dd_setconf.h
    ${GOAL_ROOT}/protos/dd/goal_dd_setip.h
    ${GOAL_ROOT}/protos/dd/goal_dd_wink.h
    ${GOAL_ROOT}/protos/dd/rpc/goal_dd_rpc.h
    ${GOAL_ROOT}/protos/dd/rpc/goal_dd_rpc_cc.h
    ${GOAL_ROOT}/protos/ethercat/drivers/board/nxp/evkmimxrt1180/goal_target_esc.c
    ${GOAL_ROOT}/protos/ethercat/drivers/goal/ctc/cc/goal_ecat_cc.h
    ${GOAL_ROOT}/protos/ethercat/drivers/goal/ctc/goal_ecat_rpc.h
    ${GOAL_ROOT}/protos/ethercat/drivers/goal/goal_ecat.h
    ${GOAL_ROOT}/protos/ethercat/drivers/goal/goal_ethercat.h
    ${GOAL_ROOT}/protos/ethercat/drivers/goal/goal_target_esc.h
    ${GOAL_ROOT}/protos/ethercat/drivers/shar_inc/ec_default.h
    ${GOAL_ROOT}/protos/ethercat/ethercat/include/ec_access.h
    ${GOAL_ROOT}/protos/ethercat/ethercat/include/ec_coe.h
    ${GOAL_ROOT}/protos/ethercat/ethercat/include/ec_compat.h
    ${GOAL_ROOT}/protos/ethercat/ethercat/include/ec_dc.h
    ${GOAL_ROOT}/protos/ethercat/ethercat/include/ec_drv.h
    ${GOAL_ROOT}/protos/ethercat/ethercat/include/ec_drvif.h
    ${GOAL_ROOT}/protos/ethercat/ethercat/include/ec_dyncfg.h
    ${GOAL_ROOT}/protos/ethercat/ethercat/include/ec_dynod.h
    ${GOAL_ROOT}/protos/ethercat/ethercat/include/ec_emerg.h
    ${GOAL_ROOT}/protos/ethercat/ethercat/include/ec_eoe.h
    ${GOAL_ROOT}/protos/ethercat/ethercat/include/ec_esc.h
    ${GOAL_ROOT}/protos/ethercat/ethercat/include/ec_esm.h
    ${GOAL_ROOT}/protos/ethercat/ethercat/include/ec_foe.h
    ${GOAL_ROOT}/protos/ethercat/ethercat/include/ec_gpio.h
    ${GOAL_ROOT}/protos/ethercat/ethercat/include/ec_header.h
    ${GOAL_ROOT}/protos/ethercat/ethercat/include/ec_led.h
    ${GOAL_ROOT}/protos/ethercat/ethercat/include/ec_mbox.h
    ${GOAL_ROOT}/protos/ethercat/ethercat/include/ec_miim.h
    ${GOAL_ROOT}/protos/ethercat/ethercat/include/ec_pdo.h
    ${GOAL_ROOT}/protos/ethercat/ethercat/include/ec_sdo.h
    ${GOAL_ROOT}/protos/ethercat/ethercat/include/ec_siiprom.h
    ${GOAL_ROOT}/protos/ethercat/ethercat/include/ec_sm.h
    ${GOAL_ROOT}/protos/ethercat/ethercat/include/ec_type.h
    ${GOAL_ROOT}/protos/ethercat/ethercat/include/ec_usr.h
    ${GOAL_ROOT}/protos/ethercat/ethercat/include/ec_version.h
    ${GOAL_ROOT}/protos/http/cm/goal_http_cm.c
    ${GOAL_ROOT}/protos/http/cm/goal_http_cm.h
    ${GOAL_ROOT}/protos/http/goal_http.c
    ${GOAL_ROOT}/protos/http/goal_http.h
    ${GOAL_ROOT}/protos/http/goal_http_auth.c
    ${GOAL_ROOT}/protos/http/goal_http_auth.h
    ${GOAL_ROOT}/protos/http/goal_http_cc.h
    ${GOAL_ROOT}/protos/http/goal_http_idlist.c
    ${GOAL_ROOT}/protos/http/goal_http_idlist.h
    ${GOAL_ROOT}/protos/http/goal_http_int.h
    ${GOAL_ROOT}/protos/http/goal_http_parser.c
    ${GOAL_ROOT}/protos/http/goal_http_parser.h
    ${GOAL_ROOT}/protos/http/goal_http_resmgr.c
    ${GOAL_ROOT}/protos/http/goal_http_resmgr.h
    ${GOAL_ROOT}/protos/http/goal_http_rpc.h
    ${GOAL_ROOT}/protos/http/goal_http_tmpmgr.c
    ${GOAL_ROOT}/protos/http/goal_http_tmpmgr.h
)

mcux_add_include(
    BASE_PATH ${CMAKE_CURRENT_LIST_DIR}
    INCLUDES
    ${GOAL_ROOT}/
    ${GOAL_ROOT}/appl/goal_ecat/00_rpc_cc
    ${GOAL_ROOT}/ext/base64/include
    ${GOAL_ROOT}/ext/lwip/src/include
    ${GOAL_ROOT}/ext/psnprintf
    ${GOAL_ROOT}/ext/uthash
    ${GOAL_ROOT}/goal
    ${GOAL_ROOT}/plat
    ${GOAL_ROOT}/plat/arch/freertos
    ${GOAL_ROOT}/plat/arch/rt1180
    ${GOAL_ROOT}/plat/board/nxp/evkmimxrt1180
    ${GOAL_ROOT}/plat/drv/bus/mdio/esc
    ${GOAL_ROOT}/plat/drv/bus/mdio/netc
    ${GOAL_ROOT}/plat/drv/eth/nxp/netc
    ${GOAL_ROOT}/plat/drv/net/lwip
    ${GOAL_ROOT}/plat/drv/net/lwip/vendor
    ${GOAL_ROOT}/plat/drv/nvs/sflash/fsl/rt1180
    ${GOAL_ROOT}/plat/drv/nvs/sflash/fsl/rt1180/vendor
    ${GOAL_ROOT}/plat/drv/phy/realtek_rtl8201
    ${GOAL_ROOT}/plat/drv/phy/realtek_rtl8211
    ${GOAL_ROOT}/projects/goal_ecat_rpc_lib/00_cc/edt/nxp/evkmimxrt1180
    ${GOAL_ROOT}/protos/dd
    ${GOAL_ROOT}/protos/dd/rpc
    ${GOAL_ROOT}/protos/ethercat/drivers/goal
    ${GOAL_ROOT}/protos/ethercat/drivers/goal/ctc
    ${GOAL_ROOT}/protos/ethercat/drivers/shar_inc
    ${GOAL_ROOT}/protos/ethercat/ethercat/include
    ${GOAL_ROOT}/protos/goal_mctc/plat/goal
    ${GOAL_ROOT}/protos/http
)

mcux_add_configuration(
    TARGETS debug flexspi_nor_debug
    # naming no TARGETS will add the elements to all build TARGETs
    # Assembler-Compiler flags
    AS " \
    -D__STARTUP_CLEAR_BSS \
    -D__STARTUP_INITIALIZE_NONCACHEDATA \
    -D__STARTUP_INITIALIZE_QADATA \
    -D__STARTUP_INITIALIZE_RAMFUNCTION \
    -D__STDC_FORMAT_MACROS \
    -D__USE_SHMEM \
    -DCPU_MIMXRT1189CVM8C_cm33 \
    -DGOAL=1 \
    -DGOAL_BASE64=1 \
    -DGOAL_CONFIG_DD=1 \
    -DGOAL_CONFIG_ECAT=1 \
    -DGOAL_CONFIG_ECAT_CC=1 \
    -DGOAL_CONFIG_ETHERCAT=1 \
    -DGOAL_CONFIG_ETHERNET=1 \
    -DGOAL_CONFIG_FEAT_UTHASH=1 \
    -DGOAL_CONFIG_GEN_CTC=1 \
    -DGOAL_CONFIG_GEN_CTC_CC=1 \
    -DGOAL_CONFIG_GEN_RPC=1 \
    -DGOAL_CONFIG_HTTP=1 \
    -DGOAL_CONFIG_LIBRARY_APPL=1 \
    -DGOAL_CONFIG_LM_EMIT=1 \
    -DGOAL_CONFIG_MCTC=1 \
    -DGOAL_CONFIG_MCTC_CC=1 \
    -DGOAL_CONFIG_MCTC_RAM=1 \
    -DGOAL_CONFIG_MCTC_RPC_DATA_LENGTH=2 \
    -DGOAL_CONFIG_MEDIA_MA_NVS=1 \
    -DGOAL_CONFIG_MEDIA_MI_DM=1 \
    -DGOAL_CONFIG_MEDIA_MI_ETH=1 \
    -DGOAL_CONFIG_MEDIA_MI_NVS=1 \
    -DGOAL_CONFIG_MM_EXT=1 \
    -DGOAL_CONFIG_TARGET_FREERTOS_TARGET_MAIN=1 \
    -DGOAL_CONFIG_TARGET_FREERTOS_TIMER_SUPPORT=1 \
    -DGOAL_CONFIG_TARGET_FREERTOS_TIMER_TICK_FROM_HOOK=1 \
    -DGOAL_CONFIG_TASK=1 \
    -DGOAL_CONFIG_TCPIP_STACK=1 \
    -DGOAL_CONFIG_TCPIP_TCP=1 \
    -DGOAL_FEAT_LWIP_INIT=1 \
    -DGOAL_MI_DM_CFG_LOCAL=1 \
    -DMCUX_META_BUILD \
    -DMCUXPRESSO_SDK \
    -DMIMXRT1189_cm33_SERIES \
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
    -DCPU_MIMXRT1189CVM8C_cm33 \
    -DGOAL=1 \
    -DGOAL_BASE64=1 \
    -DGOAL_CONFIG_DD=1 \
    -DGOAL_CONFIG_ECAT=1 \
    -DGOAL_CONFIG_ECAT_CC=1 \
    -DGOAL_CONFIG_ETHERCAT=1 \
    -DGOAL_CONFIG_ETHERNET=1 \
    -DGOAL_CONFIG_FEAT_UTHASH=1 \
    -DGOAL_CONFIG_GEN_CTC=1 \
    -DGOAL_CONFIG_GEN_CTC_CC=1 \
    -DGOAL_CONFIG_GEN_RPC=1 \
    -DGOAL_CONFIG_HTTP=1 \
    -DGOAL_CONFIG_LIBRARY_APPL=1 \
    -DGOAL_CONFIG_LM_EMIT=1 \
    -DGOAL_CONFIG_MCTC=1 \
    -DGOAL_CONFIG_MCTC_CC=1 \
    -DGOAL_CONFIG_MCTC_RAM=1 \
    -DGOAL_CONFIG_MCTC_RPC_DATA_LENGTH=2 \
    -DGOAL_CONFIG_MEDIA_MA_NVS=1 \
    -DGOAL_CONFIG_MEDIA_MI_DM=1 \
    -DGOAL_CONFIG_MEDIA_MI_ETH=1 \
    -DGOAL_CONFIG_MEDIA_MI_NVS=1 \
    -DGOAL_CONFIG_MM_EXT=1 \
    -DGOAL_CONFIG_TARGET_FREERTOS_TARGET_MAIN=1 \
    -DGOAL_CONFIG_TARGET_FREERTOS_TIMER_SUPPORT=1 \
    -DGOAL_CONFIG_TARGET_FREERTOS_TIMER_TICK_FROM_HOOK=1 \
    -DGOAL_CONFIG_TASK=1 \
    -DGOAL_CONFIG_TCPIP_STACK=1 \
    -DGOAL_CONFIG_TCPIP_TCP=1 \
    -DGOAL_FEAT_LWIP_INIT=1 \
    -DGOAL_MI_DM_CFG_LOCAL=1 \
    -DMCUX_META_BUILD \
    -DMCUXPRESSO_SDK \
    -DMIMXRT1189_cm33_SERIES \
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
    TARGETS release flexspi_nor_release
    # naming no TARGETS will add the elements to all build TARGETs
    # Assembler-Compiler flags
    AS " \
    -D__STARTUP_CLEAR_BSS \
    -D__STARTUP_INITIALIZE_NONCACHEDATA \
    -D__STARTUP_INITIALIZE_QADATA \
    -D__STARTUP_INITIALIZE_RAMFUNCTION \
    -D__STDC_FORMAT_MACROS \
    -D__USE_SHMEM \
    -DCPU_MIMXRT1189CVM8C_cm33 \
    -DGOAL=1 \
    -DGOAL_BASE64=1 \
    -DGOAL_CONFIG_DD=1 \
    -DGOAL_CONFIG_ECAT=1 \
    -DGOAL_CONFIG_ECAT_CC=1 \
    -DGOAL_CONFIG_ETHERCAT=1 \
    -DGOAL_CONFIG_ETHERNET=1 \
    -DGOAL_CONFIG_FEAT_UTHASH=1 \
    -DGOAL_CONFIG_GEN_CTC=1 \
    -DGOAL_CONFIG_GEN_CTC_CC=1 \
    -DGOAL_CONFIG_GEN_RPC=1 \
    -DGOAL_CONFIG_HTTP=1 \
    -DGOAL_CONFIG_LIBRARY_APPL=1 \
    -DGOAL_CONFIG_LM_EMIT=1 \
    -DGOAL_CONFIG_MCTC=1 \
    -DGOAL_CONFIG_MCTC_CC=1 \
    -DGOAL_CONFIG_MCTC_RAM=1 \
    -DGOAL_CONFIG_MCTC_RPC_DATA_LENGTH=2 \
    -DGOAL_CONFIG_MEDIA_MA_NVS=1 \
    -DGOAL_CONFIG_MEDIA_MI_DM=1 \
    -DGOAL_CONFIG_MEDIA_MI_ETH=1 \
    -DGOAL_CONFIG_MEDIA_MI_NVS=1 \
    -DGOAL_CONFIG_MM_EXT=1 \
    -DGOAL_CONFIG_TARGET_FREERTOS_TARGET_MAIN=1 \
    -DGOAL_CONFIG_TARGET_FREERTOS_TIMER_SUPPORT=1 \
    -DGOAL_CONFIG_TARGET_FREERTOS_TIMER_TICK_FROM_HOOK=1 \
    -DGOAL_CONFIG_TASK=1 \
    -DGOAL_CONFIG_TCPIP_STACK=1 \
    -DGOAL_CONFIG_TCPIP_TCP=1 \
    -DGOAL_FEAT_LWIP_INIT=1 \
    -DGOAL_MI_DM_CFG_LOCAL=1 \
    -DMCUX_META_BUILD \
    -DMCUXPRESSO_SDK \
    -DMIMXRT1189_cm33_SERIES \
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
    -DCPU_MIMXRT1189CVM8C_cm33 \
    -DGOAL=1 \
    -DGOAL_BASE64=1 \
    -DGOAL_CONFIG_DD=1 \
    -DGOAL_CONFIG_ECAT=1 \
    -DGOAL_CONFIG_ECAT_CC=1 \
    -DGOAL_CONFIG_ETHERCAT=1 \
    -DGOAL_CONFIG_ETHERNET=1 \
    -DGOAL_CONFIG_FEAT_UTHASH=1 \
    -DGOAL_CONFIG_GEN_CTC=1 \
    -DGOAL_CONFIG_GEN_CTC_CC=1 \
    -DGOAL_CONFIG_GEN_RPC=1 \
    -DGOAL_CONFIG_HTTP=1 \
    -DGOAL_CONFIG_LIBRARY_APPL=1 \
    -DGOAL_CONFIG_LM_EMIT=1 \
    -DGOAL_CONFIG_MCTC=1 \
    -DGOAL_CONFIG_MCTC_CC=1 \
    -DGOAL_CONFIG_MCTC_RAM=1 \
    -DGOAL_CONFIG_MCTC_RPC_DATA_LENGTH=2 \
    -DGOAL_CONFIG_MEDIA_MA_NVS=1 \
    -DGOAL_CONFIG_MEDIA_MI_DM=1 \
    -DGOAL_CONFIG_MEDIA_MI_ETH=1 \
    -DGOAL_CONFIG_MEDIA_MI_NVS=1 \
    -DGOAL_CONFIG_MM_EXT=1 \
    -DGOAL_CONFIG_TARGET_FREERTOS_TARGET_MAIN=1 \
    -DGOAL_CONFIG_TARGET_FREERTOS_TIMER_SUPPORT=1 \
    -DGOAL_CONFIG_TARGET_FREERTOS_TIMER_TICK_FROM_HOOK=1 \
    -DGOAL_CONFIG_TASK=1 \
    -DGOAL_CONFIG_TCPIP_STACK=1 \
    -DGOAL_CONFIG_TCPIP_TCP=1 \
    -DGOAL_FEAT_LWIP_INIT=1 \
    -DGOAL_MI_DM_CFG_LOCAL=1 \
    -DMCUX_META_BUILD \
    -DMCUXPRESSO_SDK \
    -DMIMXRT1189_cm33_SERIES \
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
    TARGETS debug release flexspi_nor_debug flexspi_nor_release
    SYMBOLS " \
    __use_shmem__=1 \
    __multicore__=1 \
    "
)

# replace linker script for debug configuration
mcux_remove_armgcc_linker_script(
    TARGETS debug release
    BASE_PATH ${SdkRootDirPath}
    LINKER ${device_root}/RT/RT1180/MIMXRT1189/gcc/MIMXRT1189xxxxx_cm33_ram.ld
)

mcux_add_armgcc_linker_script(
    TARGETS debug release
    BASE_PATH ${CMAKE_CURRENT_LIST_DIR}
    LINKER armgcc/MIMXRT1189xxxxx_cm33_ram.ld
)

# replace linker script for flexspi_nor_* configuration
mcux_remove_armgcc_linker_script(
    TARGETS flexspi_nor_debug flexspi_nor_release
    BASE_PATH ${SdkRootDirPath}
    LINKER ${device_root}/RT/RT1180/MIMXRT1189/gcc/MIMXRT1189xxxxx_cm33_flexspi_nor.ld
)

mcux_add_armgcc_linker_script(
    TARGETS flexspi_nor_debug flexspi_nor_release
    BASE_PATH ${CMAKE_CURRENT_LIST_DIR}
    LINKER armgcc/MIMXRT1189xxxxx_cm33_flexspi_nor.ld
)

if(NOT RIOP_SKIP_GOAL_LIB_LINKING)
# adding GOAL library
mcux_add_library(
    BASE_PATH ${CMAKE_CURRENT_LIST_DIR}
    LIBS ${GOAL_ROOT}/projects/goal_ecat_rpc_lib/00_lib_cc/gcc/build/mimxrt1180_evk_cm33_rtos/libgoal_mimxrt1180_evk_cm33_rtos.a
)
endif() # NOT RIOP_SKIP_GOAL_LIB_LINKING

mcux_add_include(
    BASE_PATH ${APPLICATION_BINARY_DIR}
    TOOLCHAINS armgcc
    INCLUDES ../riop_ECAT_M7FOLLOWER/${CONFIG_TOOLCHAIN}/
)

mcux_convert_binary(BINARY ${APPLICATION_BINARY_DIR}/${MCUX_SDK_PROJECT_NAME}.bin)
