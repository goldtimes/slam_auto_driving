find_package(Ceres REQUIRED)

add_executable(test_ceres_icp
        test_ceres_icp.cc
        ceres_icp.cc
        lidarFactor.hpp
        lidarOptimization/lidarCeres.cpp
        )

target_link_libraries(test_ceres_icp
        ${PROJECT_NAME}.ch5
        ${PROJECT_NAME}.ch7
        ${PROJECT_NAME}.common
        ${third_party_libs}
        ${CERES_LIBRARIES}
        )