nuttx_add_application(
  NAME task
  SRCS task.c
  MODULE "m")

add_custom_command(
  OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/../fsroot/task
  COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_CURRENT_BINARY_DIR}/../fsroot
  COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:task> ${CMAKE_CURRENT_BINARY_DIR}/../fsroot
  DEPENDS task)
