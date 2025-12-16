
# Copyright 2025 NXP
#
# SPDX-License-Identifier: Apache-2.0



ExternalZephyrProject_Add(
    APPLICATION riop_ECAT_M7FOLLOWER
    SOURCE_DIR ${APP_DIR}/../M7FOLLOWER
    board ${SB_CONFIG_secondary_board}
    core_id ${SB_CONFIG_secondary_core_id}
    config ${SB_CONFIG_secondary_config}
    toolchain ${SB_CONFIG_secondary_toolchain}
)

add_dependencies(${DEFAULT_IMAGE} riop_ECAT_M7FOLLOWER)
