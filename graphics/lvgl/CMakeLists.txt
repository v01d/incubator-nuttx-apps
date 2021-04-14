if (CONFIG_GRAPHICS_LVGL)
  FetchContent_Declare(lvgl URL "https://github.com/lvgl/lvgl/archive/v${CONFIG_LVGL_VERSION}.tar.gz")
  FetchContent_Populate(lvgl)
  FetchContent_GetProperties(lvgl SOURCE_DIR LVGL_SOURCE_DIR)

  set(SRCS)
  file(GLOB_RECURSE SRCS ${LVGL_SOURCE_DIR}/src/*.c)
  list(APPEND SRCS lv_tick_interface.c)

  if (NOT CONFIG_USE_LV_FILESYSTEM)
    list(APPEND SRCS lv_fs_interface.c)
  endif()

  # support including as <lvgl/lvgl.h>
  execute_process(
    COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_CURRENT_BINARY_DIR}/include
    COMMAND ${CMAKE_COMMAND} -E create_symlink ${LVGL_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR}/include/lvgl
  )

  nuttx_add_user_library(lvgl)
  target_sources(lvgl PRIVATE ${SRCS})
  target_compile_options(lvgl PRIVATE -Wno-format)
  target_compile_definitions(lvgl PUBLIC -DLV_CONF_INCLUDE_SIMPLE)
  target_include_directories(lvgl PUBLIC ${CMAKE_CURRENT_LIST_DIR})
  target_include_directories(lvgl INTERFACE ${LVGL_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR}/include)
endif()
