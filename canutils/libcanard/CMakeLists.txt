if(CONFIG_CANUTILS_LIBCANARD)
  FetchContent_Declare(libcanard
    URL "${CONFIG_LIBCANARD_URL}/${CONFIG_LIBCANARD_VERSION}.tar.gz")
  FetchContent_Populate(libcanard)
  FetchContent_GetProperties(libcanard)

  nuttx_add_user_library(libcanard)
  target_sources(libcanard PRIVATE ${libcanard_SOURCE_DIR}/canard.c
    ${libcanard_SOURCE_DIR}/drivers/nuttx/canard_nuttx.c)
  target_include_directories(libcanard PUBLIC
    ${libcanard_SOURCE_DIR} ${libcanard_SOURCE_DIR}/drivers/nuttx)
endif()
