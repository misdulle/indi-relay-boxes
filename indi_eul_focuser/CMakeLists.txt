cmake_minimum_required(VERSION 3.16)
project(indi-encoder-focuser CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Prefer INDI's own config package (2.x); fall back to our FindINDI.cmake
find_package(INDI QUIET CONFIG)
if(NOT INDI_FOUND)
  list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake_modules)
  find_package(INDI REQUIRED MODULE)
endif()
find_package(Threads REQUIRED)

include_directories(${INDI_INCLUDE_DIR})

add_executable(indi_encoder_focuser encoder_focuser.cpp)
target_link_libraries(indi_encoder_focuser ${INDI_DRIVER_LIBRARIES} Threads::Threads)

install(TARGETS indi_encoder_focuser RUNTIME DESTINATION bin)
install(FILES indi_encoder_focuser.xml DESTINATION ${INDI_DATA_DIR})
