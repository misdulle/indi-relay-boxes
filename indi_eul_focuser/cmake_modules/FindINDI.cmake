# Minimal FindINDI: used when the INDI install does not ship INDIConfig.cmake.
# Sets INDI_FOUND, INDI_INCLUDE_DIR, INDI_DRIVER_LIBRARIES, INDI_DATA_DIR
find_path(INDI_INCLUDE_DIR indidevapi.h PATH_SUFFIXES libindi)
find_library(INDI_DRIVER_LIB NAMES indidriver)
find_library(INDI_MAIN_LIB NAMES indiAlignmentDriver indi)
if(INDI_INCLUDE_DIR AND INDI_DRIVER_LIB)
  set(INDI_DRIVER_LIBRARIES ${INDI_DRIVER_LIB})
  if(NOT INDI_DATA_DIR)
    set(INDI_DATA_DIR "/usr/share/indi")
  endif()
  set(INDI_FOUND TRUE)
endif()
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(INDI DEFAULT_MSG INDI_INCLUDE_DIR INDI_DRIVER_LIBRARIES)
