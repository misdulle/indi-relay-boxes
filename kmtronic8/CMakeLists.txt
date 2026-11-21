add_executable(indi_kmtronic8 kmtronic8.cpp)
target_link_libraries(indi_kmtronic8 ${INDI_DRIVER_LIBRARIES} Threads::Threads)
install(TARGETS indi_kmtronic8 RUNTIME DESTINATION bin)
install(FILES indi_kmtronic8.xml DESTINATION ${INDI_DATA_DIR})
