add_executable(run_imu_integration
    run_imu_integration.cc)

target_link_libraries(run_imu_integration
    glog
    gflags
    ${PROJECT_NAME}.common
    )

add_executable(process_gnss

process_gnss.cc)

target_link_libraries(process_gnss
    glog
    gflags
    ${PROJECT_NAME}.common 
    ${PROJECT_NAME}.imu_gnss
    )


add_library(
    ${PROJECT_NAME}.imu_gnss

    ${PROJECT_SOURCE_DIR}/thirdparty/utm_convert/utm.cc
    ${PROJECT_SOURCE_DIR}/thirdparty/utm_convert/tranmerc.cc
    utm_convert.cc
)