if (CONFIG_SYSTEM_ARGTABLE3)
  FetchContent_Declare(argtable3
    URL https://github.com/argtable/argtable3/archive/v$(ARGTABLE3_VERSION).tar.gz)
  FetchContent_Populate(argtable3)
  FetchContent_GetProperties(argtable3 SORUCE_DIR ARGTABLE3_SOURCE_DIR)

  nuttx_add_user_library(argtable3)
  target_compile_options(argtable3 PRIVATE -DARG_REPLACE_GETOPT=0)

  file(GLOB SRCS MATCH_DIRECTORIES false ${ARGTABLE3_SOURCE_DIR}/*.c)
  target_sources(argtable3 PRIVATE ${SRCS})
endif()
