include(options)

# Paths to the drivers/modules folders. Pattern:
# ${MW_DRV_PATH1}/<drvName>${MW_DRV_PATH2}/<drvName>.c
set(MW_DRV_PATH1 "${MW_INSTALL_DIR}/sw/drivers")
set(MW_DRV_PATH2 "/src/32b/f28x/f2806x")
set(MW_MOD_PATH1 "${MW_INSTALL_DIR}/sw/modules")
set(MW_MOD_PATH2 "/src/32b")

# File extensions of the sources can be .c or .asm.
set(MW_SRC_FE ".c" ".asm")

# Names of drivers with standard paths.
set(MW_DRV_NAMES
    "adc"
    "clk"
    "cpu"
    "flash"
    "gpio"
    "osc"
    "pie"
    "pll"
    "pwm"
    "pwr"
    "timer"
    "wdog")

# Names of modules with standarnd paths.
set(MW_MOD_NAMES
    "clarke"
    "fw"
    "ipark"
    "offset"
    "park"
    "pid"
    "svgen"
    "traj")

# Manually enter paths to files that do not follow the standard pattern.
set(MW_SRC
    "${MW_DRV_PATH1}/cpu${MW_DRV_PATH2}/CodeStartBranch.asm"
    "${MW_MOD_PATH1}/filter${MW_MOD_PATH2}/filter_fo.c"
    "${MW_MOD_PATH1}/svgen${MW_MOD_PATH2}/svgen_current.c"
    "${MW_MOD_PATH1}/usDelay${MW_MOD_PATH2}/f28x/usDelay.asm"
    "${MW_MOD_PATH1}/memCopy/src/memCopy.c")

# Generate paths for drivers with standard pattern.
foreach(srcName ${MW_DRV_NAMES})
  set(basePath "${MW_DRV_PATH1}/${srcName}${MW_DRV_PATH2}/${srcName}")
  foreach(fe ${MW_SRC_FE})
    if(EXISTS "${basePath}${fe}")
      list(APPEND MW_SRC "${basePath}${fe}")
    endif()
  endforeach()
endforeach()

# Generate paths for modules with standard pattern.
foreach(srcName ${MW_MOD_NAMES})
  set(basePath "${MW_MOD_PATH1}/${srcName}${MW_MOD_PATH2}/${srcName}")
  foreach(fe ${MW_SRC_FE})
    if(EXISTS "${basePath}${fe}")
      list(APPEND MW_SRC "${basePath}${fe}")
    endif()
  endforeach()
endforeach()
