# Minimal FindINDI: used when the INDI install does not ship INDIConfig.cmake.
# Sets INDI_FOUND, INDI_INCLUDE_DIR, INDI_DRIVER_LIBRARIES, INDI_DATA_DIR
# Override with -DINDI_DRIVER_LIB=/path/to/libindidriver.so if needed.
find_path(INDI_INCLUDE_DIR indidevapi.h PATH_SUFFIXES libindi)

# Prefer the distribution's multiarch directory (e.g. /usr/lib/aarch64-linux-gnu) over a
# stray /usr/lib copy left behind by an older source install.
if(NOT INDI_DRIVER_LIB)
  find_library(INDI_DRIVER_LIB NAMES indidriver
               PATHS /usr/lib/${CMAKE_LIBRARY_ARCHITECTURE} /usr/lib/aarch64-linux-gnu /usr/lib/x86_64-linux-gnu
               NO_DEFAULT_PATH)
  if(NOT INDI_DRIVER_LIB)
    find_library(INDI_DRIVER_LIB NAMES indidriver)
  endif()
endif()

if(INDI_INCLUDE_DIR AND INDI_DRIVER_LIB)
  set(INDI_DRIVER_LIBRARIES ${INDI_DRIVER_LIB})
  if(NOT INDI_DATA_DIR)
    set(INDI_DATA_DIR "/usr/share/indi")
  endif()
  set(INDI_FOUND TRUE)
  message(STATUS "INDI headers: ${INDI_INCLUDE_DIR}")
  message(STATUS "INDI driver library: ${INDI_DRIVER_LIB}")
endif()
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(INDI DEFAULT_MSG INDI_INCLUDE_DIR INDI_DRIVER_LIBRARIES)
