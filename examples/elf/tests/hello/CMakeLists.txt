nuttx_add_application(
  NAME hello
  SRCS hello.c
  MODULE "m")

add_custom_command(
  OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/../fsroot/hello
  COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_CURRENT_BINARY_DIR}/../fsroot
  COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:hello> ${CMAKE_CURRENT_BINARY_DIR}/../fsroot
  DEPENDS hello)

