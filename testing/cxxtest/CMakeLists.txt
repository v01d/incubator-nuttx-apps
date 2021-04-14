if (CONFIG_TESTING_CXXTEST)
  nuttx_add_application(
    SRCS cxxtest_main.cxx
    NAME cxxtest)
endif()
