nuttx_add_application(
  NAME struct
  SRCS struct_main.c struct_dummy.c
  MODULE "m")

add_custom_command(
  OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/../fsroot/struct
  COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_CURRENT_BINARY_DIR}/../fsroot
  COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:struct> ${CMAKE_CURRENT_BINARY_DIR}/../fsroot
  DEPENDS struct)

