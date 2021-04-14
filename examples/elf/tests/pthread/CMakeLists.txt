nuttx_add_application(
  NAME pthread
  SRCS pthread.c
  MODULE "m")

add_custom_command(
  OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/../fsroot/pthread
  COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_CURRENT_BINARY_DIR}/../fsroot
  COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:pthread> ${CMAKE_CURRENT_BINARY_DIR}/../fsroot
  DEPENDS pthread)

