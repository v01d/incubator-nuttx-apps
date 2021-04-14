nuttx_add_application(
  NAME signal
  SRCS signal.c
  MODULE "m")

add_custom_command(
  OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/../fsroot/signal
  COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_CURRENT_BINARY_DIR}/../fsroot
  COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:signal> ${CMAKE_CURRENT_BINARY_DIR}/../fsroot
  DEPENDS signal)

