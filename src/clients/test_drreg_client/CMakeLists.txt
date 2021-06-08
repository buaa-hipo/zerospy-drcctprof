# **********************************************************
# Copyright (c) 2020 Xuhpclab. All rights reserved.
# Licensed under the MIT License.
# See LICENSE file for more information.
#  **********************************************************

cmake_minimum_required(VERSION 2.6)

include(../../make/policies.cmake NO_POLICY_SCOPE)

if (UNIX)
  add_compile_options(-Wno-deprecated)
  add_compile_options(-Wno-unused-result)
  add_compile_options(-Wno-unused-variable)
  add_compile_options(-Wno-unused-local-typedefs)
  add_compile_options(-Wno-unused-function)
  add_compile_options(-Werror=sign-compare)
  add_compile_options(-Werror=narrowing)
  add_compile_options(-std=c++11)

  if (DEBUG)
    add_compile_options(-g3)
  endif (DEBUG)
endif (UNIX)

# add third-party libraries

add_library(drreg_aflag_reserve_test_client SHARED
  drreg_aflag_reserve_test_client.cpp
  )

configure_DynamoRIO_client(drreg_aflag_reserve_test_client)
use_DynamoRIO_extension(drreg_aflag_reserve_test_client drreg) 
use_DynamoRIO_extension(drreg_aflag_reserve_test_client drx) 
place_shared_lib_in_lib_dir(drreg_aflag_reserve_test_client)

add_dependencies(drreg_aflag_reserve_test_client api_headers)

# Provide a hint for how to use the client
if (NOT DynamoRIO_INTERNAL OR NOT "${CMAKE_GENERATOR}" MATCHES "Ninja")
  add_custom_command(TARGET drreg_aflag_reserve_test_client
    POST_BUILD
    COMMAND ${CMAKE_COMMAND}
    ARGS -E echo "Usage: pass to drconfig or drrun: -t drreg_aflag_reserve_test_client"
    VERBATIM)
endif ()

install_target(drreg_aflag_reserve_test_client ${INSTALL_CLIENTS_LIB})

set(INSTALL_CCTTEST_CONFIG ${INSTALL_CLIENTS_BASE})

function (write_config_file dst bindir libdir)
  file(WRITE  ${dst} "# drreg_aflag_reserve_test_client tool config file\n")
  file(APPEND ${dst} "CLIENT_REL=${libdir}/${LIB_PFX}drreg_aflag_reserve_test_client${LIB_EXT}\n")
endfunction ()

if (X64)
  set(CONFIG_INSTALL ${PROJECT_BINARY_DIR}/drreg_aflag_reserve_test_client.drrun64)
  set(CONFIG_BUILD ${PROJECT_BINARY_DIR}/tools/drreg_aflag_reserve_test_client.drrun64)
else (X64)
  set(CONFIG_INSTALL ${PROJECT_BINARY_DIR}/drreg_aflag_reserve_test_client.drrun32)
  set(CONFIG_BUILD ${PROJECT_BINARY_DIR}/tools/drreg_aflag_reserve_test_client.drrun32)
endif (X64)

set(BUILD_CLIENTS_BIN clients/${INSTALL_BIN})
set(BUILD_CLIENTS_LIB clients/${INSTALL_LIB})

write_config_file(${CONFIG_INSTALL} ${INSTALL_CLIENTS_BIN} ${INSTALL_CLIENTS_LIB})
write_config_file(${CONFIG_BUILD} ${BUILD_CLIENTS_BIN} ${BUILD_CLIENTS_LIB})

DR_install(FILES "${CONFIG_INSTALL}" DESTINATION ${INSTALL_CCTTEST_CONFIG})
register_tool_file("drreg_aflag_reserve_test_client")

##################################################
# Documentation

