cmake_minimum_required(VERSION 3.8)
project(event_recording)

# C++コンパイラオプションの設定
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(MetavisionSDK COMPONENTS core driver ui REQUIRED)

find_package(Boost REQUIRED)

add_executable(event_recorder src/event_recorder.cpp)
target_include_directories(event_recorder PRIVATE
   ${OpenCV_INCLUDE_DIRS}
   ${Boost_INCLUDE_DIRS}
   # Metavisionインクルードディレクトリ
)
target_link_libraries(event_recorder
   ${OpenCV_LIBS}
   ${Boost_LIBRARIES}
   MetavisionSDK::core 
   MetavisionSDK::driver 
   MetavisionSDK::ui
)
ament_target_dependencies(event_recorder rclcpp cv_bridge std_msgs MetavisionSDK)

install(TARGETS event_recorder
   DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
   launch
   config
   DESTINATION share/${PROJECT_NAME})  
 

ament_package()