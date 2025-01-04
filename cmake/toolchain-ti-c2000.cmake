include(options)

if(NOT DEFINED CG_TOOL_ROOT)
  message(FATAL_ERROR "CG_TOOL_ROOT is not defined in options.cmake")
endif()

if(NOT DEFINED DEVICE_SUPPORT_ROOT)
  message(FATAL_ERROR "DEVICE_SUPPORT_ROOT is not defined in options.cmake")
endif()

if(NOT DEFINED TI_PRODUCTS_DIR)
  message(FATAL_ERROR "TI_PRODUCTS_DIR is not defined in options.cmake")
endif()

if(NOT DEFINED C2000_WARE_DIR)
  message(FATAL_ERROR "C2000_WARE_DIR is not defined in options.cmake")
endif()

if(NOT DEFINED MW_INSTALL_DIR)
  message(FATAL_ERROR "MW_INSTALL_DIR is not defined in options.cmake")
endif()

if(NOT IS_DIRECTORY "${CG_TOOL_ROOT}")
  message(
    FATAL_ERROR
      "The specified directory CG_TOOL_ROOT does not exist: ${CG_TOOL_ROOT}")
endif()

if(NOT IS_DIRECTORY "${DEVICE_SUPPORT_ROOT}")
  message(
    FATAL_ERROR
      "The specified directory DEVICE_SUPPORT_ROOT does not exist: ${DEVICE_SUPPORT_ROOT}"
  )
endif()

if(NOT IS_DIRECTORY "${TI_PRODUCTS_DIR}")
  message(
    FATAL_ERROR
      "The specified directory TI_PRODUCTS_DIR does not exist: ${TI_PRODUCTS_DIR}"
  )
endif()

if(NOT IS_DIRECTORY "${C2000_WARE_DIR}")
  message(
    FATAL_ERROR
      "The specified directory C2000_WARE_DIR does not exist: ${C2000_WARE_DIR}"
  )
endif()

if(NOT IS_DIRECTORY "${MW_INSTALL_DIR}")
  message(
    FATAL_ERROR
      "The specified directory MW_INSTALL_DIR does not exist: ${MW_INSTALL_DIR}"
  )
endif()

if(NOT EXISTS "${UNIFLASH_PATH}")
  message(
    WARNING
      "Uniflash not found at \"${UNIFLASH_PATH}\". You will not be able to flash via make."
  )
endif()

if(NOT EXISTS "${TARGET_CONFIG}")
  message(
    WARNING
      "Target config not found at \"${TARGET_CONFIG}\". You will not be able to flash via make."
  )
endif()
# message(STATUS "Found a valid options.cmake")

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# toolchain paths
find_program(
  TI_GCC
  NAMES cl2000
  PATHS ${CG_TOOL_ROOT}/bin
  NO_DEFAULT_PATH)
find_program(
  TI_CXX
  NAMES cl2000
  PATHS ${CG_TOOL_ROOT}/bin
  NO_DEFAULT_PATH)
find_program(
  TI_AS
  NAMES cl2000
  PATHS ${CG_TOOL_ROOT}/bin
  NO_DEFAULT_PATH)
find_program(
  TI_AR
  NAMES ar2000
  PATHS ${CG_TOOL_ROOT}/bin
  NO_DEFAULT_PATH)
find_program(
  TI_OBJCOPY
  NAMES ofd2000
  PATHS ${CG_TOOL_ROOT}/bin
  NO_DEFAULT_PATH)
find_program(
  TI_OBJDUMP
  NAMES hex2000
  PATHS ${CG_TOOL_ROOT}/bin
  NO_DEFAULT_PATH)
find_program(
  TI_SIZE
  NAMES size2000
  PATHS ${CG_TOOL_ROOT}/bin
  NO_DEFAULT_PATH)
find_program(
  TI_LD
  NAMES cl2000
  PATHS ${CG_TOOL_ROOT}/bin
  NO_DEFAULT_PATH)

# set executables settings
set(CMAKE_C_COMPILER ${TI_GCC})
set(CMAKE_CXX_COMPILER ${TI_CXX})
set(AS ${TI_AS})
set(AR ${TI_AR})
set(OBJCOPY ${TI_OBJCOPY})
set(OBJDUMP ${TI_OBJDUMP})
set(SIZE ${TI_SIZE})
set(LD ${TI_LD})

# Add the default include and lib directories for tool chain
include_directories(
  ${CG_TOOL_ROOT}/include ${DEVICE_SUPPORT_ROOT}/f2806x/headers/include
  ${DEVICE_SUPPORT_ROOT}/f2806x/common/include)

# linker search paths
link_directories(${CG_TOOL_ROOT}/lib ${DEVICE_SUPPORT_ROOT}/f2806x/headers/cmd
                 ${DEVICE_SUPPORT_ROOT}/f2806x/common/cmd)

# where is the target environment
set(CMAKE_FIND_ROOT_PATH ${CG_TOOL_ROOT})

# search for programs in the build host directories
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
# for libraries and headers in the target directories
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
