# add_library(${PROJECT_NAME}.ch4
#     gins_pre_integ.cc
#     imu_preintegration.cc
#     g2o_types.cc)

# add_executable(run_gins_pre_integ run_gins_pre_integ.cc)
# target_link_libraries(run_gins_pre_integ
#             ${PROJECT_NAME}.ch3
#             ${PROJECT_NAME}.ch4
#             ${g2o_libs}
# )