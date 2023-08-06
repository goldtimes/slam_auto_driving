add_executable(run_imu_integration run_imu_integration.cc)
target_link_libraries(run_imu_integration glog gflags ${PROJECT_NAME}.common)