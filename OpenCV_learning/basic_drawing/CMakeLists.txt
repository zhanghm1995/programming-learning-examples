add_executable(geometry_drawing geometry_drawing.cpp)
target_link_libraries(geometry_drawing ${OpenCV_LIBS})

add_executable(mouse_callback mouse_callback.cpp)
target_link_libraries(mouse_callback ${OpenCV_LIBS})