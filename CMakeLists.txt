cmake_minimum_required(VERSION 3.13)

#set(CMAKE_C_COMPILER_WORKS 1)
#set(CMAKE_CXX_COMPILER_WORKS 1)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${CMAKE_CURRENT_SOURCE_DIR}/cmake")
if(NOT DEFINED CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/cmake/toolchain-ti-c2000.cmake")
    message(STATUS "CMAKE_TOOLCHAIN_FILE not set, using default")
    include(${CMAKE_TOOLCHAIN_FILE})
endif()

set(PROJ_NAME "proj_lab09a-flash")
set(BIN_SUFFIX ".out")
set(MAIN_BINARY "${PROJ_NAME}${BIN_SUFFIX}")

project(${PROJ_NAME} C ASM)

set(PROJECT_LOC ${CMAKE_CURRENT_SOURCE_DIR})

file(GLOB_RECURSE MODULE_SOURCES CONFIGURE_DEPENDS "include/*/*.c")
file(GLOB_RECURSE MODULE_HEADERS CONFIGURE_DEPENDS "include/*/*.h")

set(MODULE_INCLUDE_DIRS "")
foreach(_headerFile ${MODULE_HEADERS})
    get_filename_component(_dir ${_headerFile} PATH)
    list(APPEND MODULE_INCLUDE_DIRS ${_dir})
endforeach()
list(REMOVE_DUPLICATES MODULE_INCLUDE_DIRS)

include(mw-src)

add_executable(${PROJ_NAME} ${MODULE_SOURCES} ${MW_SRC} proj_utils.c proj_lab09a.c)
target_include_directories(${PROJ_NAME} PRIVATE ${PROJECT_LOC} 
                           ${C2000_WARE_DIR} ${C2000_WARE_DIR}/libraries/flash_api/f2806x/include 
                           ${CG_TOOL_ROOT}/include ${MW_INSTALL_DIR} ${MODULE_INCLUDE_DIRS})
add_compile_options(-Wall -c99)
target_compile_options(${PROJ_NAME} BEFORE PUBLIC -v28 --float_support=fpu32 -ml -mt --abi=coffabi)
target_compile_definitions(${PROJ_NAME} PUBLIC FAST_ROM_V1p6 EEPROM_EMULATION ECAN FLASH)

target_link_directories(${PROJ_NAME} PRIVATE ${PROJECT_LOC} ${CG_TOOL_ROOT}/lib 
                        ${C2000_WARE_DIR}/libraries/flash_api/f2806x/lib 
                        ${CG_TOOL_ROOT}/include
						${MW_INSTALL_DIR}/sw/modules/fast/lib/32b/f28x/f2806x/ 
                        ${MW_INSTALL_DIR}/sw/modules/iqmath/lib/f28x/float/) 
target_link_options(${PROJ_NAME} BEFORE PUBLIC --heap_size=0x400 --stack_size=0x3b0 --warn_sections 
                    --reread_libs --display_error_number --rom_model --entry_point=code_start 
                    --diag_suppress=10063 -o "${MAIN_BINARY}")
target_link_libraries(${PROJ_NAME} -lFlash2806x_API_wFPU_Library.lib -lrts2800_fpu32.lib 
                      2806xRevB_FastSpinROMSymbols_fpu32.lib IQmath_fpu32.lib -llibc.a 
                      ${PROJECT_LOC}/F28069M.cmd F2806x_Headers_nonBIOS.cmd)


# Add command to flash device
if(EXISTS "${UNIFLASH_PATH}" AND EXISTS "${TARGET_CONFIG}")
    get_filename_component(UNIFLASH_EXT ${UNIFLASH_PATH} EXT)
	string(TOLOWER "${UNIFLASH_EXT}" UNIFLASH_EXT)
	if("${UNIFLASH_EXT}" STREQUAL ".sh")
        set(FLASH_CMD "")
    else()
        set(FLASH_CMD "flash")
    endif()
    add_custom_target(flash
        COMMAND ${UNIFLASH_PATH} ${FLASH_CMD} "--flash" "--verbose" "--verify" "--config=${TARGET_CONFIG}" "${CMAKE_BINARY_DIR}/${MAIN_BINARY}"
        DEPENDS "${CMAKE_BINARY_DIR}/${MAIN_BINARY}"
    )
endif()

