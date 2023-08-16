add_executable(point_cloud_load_vis point_cloud_load_vis.cc)
target_link_libraries(point_cloud_load_vis
        ${PCL_LIBRARIES}
        ${GLOG_LIBRARIES}
        gflags
        )