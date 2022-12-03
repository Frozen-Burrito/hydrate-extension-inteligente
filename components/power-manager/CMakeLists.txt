set(SOURCES "power-manager.c")

idf_component_register(
    SRCS ${SOURCES}
    INCLUDE_DIRS "include"
    REQUIRES soc ulp driver)

set(ulp_app_name ulp_main)
set(ulp_s_sources ulp/motion.S)
set(ulp_exp_dep_srcs ${SOURCES})

ulp_embed_binary(${ulp_app_name} "${ulp_s_sources}" "${ulp_exp_dep_srcs}")
