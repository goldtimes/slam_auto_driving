add_library(${PROJECT_NAME}.common
        io_utils.cc
        # timer/timer.cc
        global_flags.cc
        # g2o_types.cc
        # point_cloud_utils.cc
        )

target_link_libraries(${PROJECT_NAME}.common
        ${PROJECT_NAME}.tools
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        )