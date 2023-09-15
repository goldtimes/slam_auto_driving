add_executable(test_icp_gicp
        test_icp_gicp.cc
        )

target_link_libraries(test_icp_gicp
        ${PROJECT_NAME}.ch5
        ${PROJECT_NAME}.ch7
        ${PROJECT_NAME}.common
        ${third_party_libs}
        )