add_library(${PROJECT_NAME}.tools
            # 窗口类
            ui/pangolin_window.cc
            ui/pangolin_window_impl.cc
            )
target_link_libraries(${PROJECT_NAME}.tools
                    ${third_party_libs}
                    )
add_executable(ui_test ui/ui_test.cc)   
target_link_libraries(ui_test
                    ${PROJECT_NAME}.tools
                    ${third_party_libs}
                    )    