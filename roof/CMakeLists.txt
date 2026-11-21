add_executable(indi_roof roof.cpp)
target_link_libraries(indi_roof ${INDI_DRIVER_LIBRARIES} Threads::Threads)
install(TARGETS indi_roof RUNTIME DESTINATION bin)
install(FILES indi_roof.xml DESTINATION ${INDI_DATA_DIR})
