add_executable(point_cloud_load_vis point_cloud_load_vis.cc)
target_link_libraries(point_cloud_load_vis
        ${PCL_LIBRARIES}
        ${GLOG_LIBRARIES}
        gflags
        )

add_library(${PROJECT_NAME}.ch5
        bfnn.cc
)
target_link_libraries(${PROJECT_NAME}.ch5
        tbb
)