add_subdirectory(hello)
add_subdirectory(redirect)

configure_file(testdata.txt ${CMAKE_CURRENT_BINARY_DIR}/fsroot/testdata.txt COPYONLY)
