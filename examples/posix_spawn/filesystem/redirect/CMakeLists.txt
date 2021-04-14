nuttx_add_application(
  NAME redirect
  SRCS redirect.c
  MODULE "m")

add_custom_command(TARGET redirect POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_CURRENT_BINARY_DIR}/../fsroot
  COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:redirect> ${CMAKE_CURRENT_BINARY_DIR}/../fsroot)

