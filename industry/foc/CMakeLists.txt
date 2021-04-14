if (CONFIG_INDUSTRY_FOC)

  # FOC Library
  set(SRCS foc_utils.c)

  # float support
  if (CONFIG_INDUSTRY_FOC_FLOAT)
    list(APPEND SRCS
      float/foc_ramp.c float/foc_handler.c float/foc_angle.c
      float/foc_velocity.c)

    if (CONFIG_INDUSTRY_FOC_ANGLE_OPENLOOP)
      list(APPEND SRCS float/foc_openloop.c)
    endif()
    if (CONFIG_INDUSTRY_FOC_CORDIC)
      list(APPEND SRCS float/foc_cordic.c)
    endif()
    if (CONFIG_INDUSTRY_FOC_CONTROL_PI)
      list(APPEND SRCS float/foc_picontrol.c)
    endif()
    if (CONFIG_INDUSTRY_FOC_MODULATION_SVM3)
      list(APPEND SRCS float/foc_svm3.c)
    endif()
    if (CONFIG_INDUSTRY_FOC_HAVE_MODEL)
    list(APPEND SRCS float/foc_model.c)
      endif()
    if (CONFIG_INDUSTRY_FOC_MODEL_PMSM)
      list(APPEND SRCS float/foc_model_pmsm.c)
    endif()
  endif()

  # fixed16 support
  if (CONFIG_INDUSTRY_FOC_FIXED16)
    list(APPEND SRCS
      fixed16/foc_ramp.c fixed16/foc_handler.c fixed16/foc_angle.c
      fixed16/foc_velocity.c)

    if (CONFIG_INDUSTRY_FOC_ANGLE_OPENLOOP)
      list(APPEND SRCS fixed16/foc_openloop.c)
    endif()
    if (CONFIG_INDUSTRY_FOC_CORDIC)
      list(APPEND SRCS fixed16/foc_cordic.c)
    endif()
    if (CONFIG_INDUSTRY_FOC_CONTROL_PI)
      list(APPEND SRCS fixed16/foc_picontrol.c)
    endif()
    if (CONFIG_INDUSTRY_FOC_MODULATION_SVM3)
      list(APPEND SRCS fixed16/foc_svm3.c)
    endif()
    if (CONFIG_INDUSTRY_FOC_HAVE_MODEL)
      list(APPEND SRCS fixed16/foc_model.c)
    endif()
    if (CONFIG_INDUSTRY_FOC_MODEL_PMSM)
      list(APPEND SRCS fixed16/foc_model_pmsm.c)
    endif()
  endif()

  nuttx_add_user_library(foc ${SRCS})
endif()
