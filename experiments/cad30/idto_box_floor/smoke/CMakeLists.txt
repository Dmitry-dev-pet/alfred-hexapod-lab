include_directories(${PROJECT_SOURCE_DIR})
add_executable(cad30_idto_smoke cad30.cc)
target_link_libraries(cad30_idto_smoke drake::drake example_base find_resource gflags)
configure_file(cad30.yaml ${CMAKE_BINARY_DIR}/idto/examples/cad30/cad30.yaml COPYONLY)
