add_executable(indi_relay16 relay16.cpp)
target_link_libraries(indi_relay16 ${INDI_DRIVER_LIBRARIES} Threads::Threads)
install(TARGETS indi_relay16 RUNTIME DESTINATION bin)
install(FILES indi_relay16.xml DESTINATION ${INDI_DATA_DIR})
