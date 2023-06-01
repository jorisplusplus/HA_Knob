set(srcs "motor.c"
        "motorindent.c")


idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS "include"
                       REQUIRES esp_timer driver)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wdouble-promotion)