nuttx_add_application(
  NAME hello
  SRCS hello.c
  MODULE "m")

add_custom_command(TARGET hello POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_CURRENT_BINARY_DIR}/../fsroot
  COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:hello> ${CMAKE_CURRENT_BINARY_DIR}/../fsroot)
