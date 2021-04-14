nuttx_add_application(
  NAME longjmp
  SRCS longjmp.c
  MODULE "m")

add_custom_command(
  OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/../fsroot/longjmp
  COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_CURRENT_BINARY_DIR}/../fsroot
  COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:longjmp> ${CMAKE_CURRENT_BINARY_DIR}/../fsroot
  DEPENDS longjmp)

