if (CONFIG_INTERPRETERS_FICL)
  nuttx_add_user_library(ficl src/nuttx.c)

  # TODO: build depends on undeclared external sources?
endif()
