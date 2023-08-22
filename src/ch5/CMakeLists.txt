add_executable(point_cloud_load_vis point_cloud_load_vis.cc)
target_link_libraries(point_cloud_load_vis
        ${PCL_LIBRARIES}
        ${GLOG_LIBRARIES}
        gflags
        )

add_library(${PROJECT_NAME}.ch5
        bfnn.cc
        kdtree.cc
)
target_link_libraries(${PROJECT_NAME}.ch5
        tbb
)

add_executable(linear_fitting linear_fitting.cc)
target_link_libraries(linear_fitting
        ${PCL_LIBRARIES}
        ${GLOG_LIBRARIES}
        ${OpenCV_LIBS}
        gflags
        )

ADD_EXECUTABLE(test_nn test_nn.cc)
ADD_TEST(test_nn test_bfnn)
target_link_libraries(test_nn
        gtest pthread glog gflags  ${PROJECT_NAME}.ch5 ${PROJECT_NAME}.common ${PCL_LIBRARIES} tbb
        )