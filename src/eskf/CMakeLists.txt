add_library(
    ${PROJECT_NAME}.eskf

    static_imu_init.cc
    ${PROJECT_SOURCE_DIR}/src/imu_gnss/utm_convert.cc
    ${PROJECT_SOURCE_DIR}/thirdparty/utm_convert/utm.cc
    ${PROJECT_SOURCE_DIR}/thirdparty/utm_convert/tranmerc.cc
)

add_executable(run_eskf_gins run_eskf_gins.cc)
target_link_libraries(run_eskf_gins
        glog gflags ${PROJECT_NAME}.common ${PROJECT_NAME}.eskf
        )