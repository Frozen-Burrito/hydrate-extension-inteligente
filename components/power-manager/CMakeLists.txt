set(SOURCES "power-manager.c")

idf_component_register(SRCS ${SOURCES}
                    INCLUDE_DIRS "include")

set(ulp_app_name ulp_${COMPONENT_NAME})
set(ulp_s_sources ulp/movement.S)
set(ulp_exp_dep_srcs ${SOURCES})

ulp_embed_binary(${ulp_app_name} "${ulp_s_sources}" "${ulp_exp_dep_srcs}")
