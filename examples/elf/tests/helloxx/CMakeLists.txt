foreach(app hello++1 hello++2 hello++3 hello++4 hello++5)
  # skip some apps based on configs
  if ((${app} STREQUAL "hello++2" AND NOT CONFIG_HAVE_CXXINITIALIZE)
      OR ((${app} STREQUAL "hello++4" OR ${app} STREQUAL "hello++5") AND NOT CONFIG_EXAMPLES_ELF_CXX))
    continue()
  endif()

  nuttx_add_application(
    NAME ${app}
    SRCS ${app}.cxx
    MODULE "m")

  add_custom_command(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/../fsroot/${app}
    COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_CURRENT_BINARY_DIR}/../fsroot
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${app}> ${CMAKE_CURRENT_BINARY_DIR}/../fsroot
    DEPENDS ${app})
endforeach()
