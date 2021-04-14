nuttx_add_application(
  NAME mutex
  SRCS mutex.c
  MODULE "m")

add_custom_command(
  OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/../fsroot/mutex
  COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_CURRENT_BINARY_DIR}/../fsroot
  COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:mutex> ${CMAKE_CURRENT_BINARY_DIR}/../fsroot
  DEPENDS mutex)

