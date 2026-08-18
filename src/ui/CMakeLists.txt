cmake_minimum_required(VERSION 3.17)
project(ui LANGUAGES CXX)

# 收集所有源文件
file(GLOB_RECURSE UI_SOURCES 
    "*.cpp"
    "*.h"
)

# 创建静态库
add_library(${PROJECT_NAME} STATIC ${UI_SOURCES})

# 查找所需的包
find_package(Qt6 REQUIRED COMPONENTS
    Widgets
    OpenGLWidgets
)
find_package(glm CONFIG REQUIRED)

# 设置包含目录（确保Qt6 Widgets头文件路径被包含）
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/src/glframework
    ${CMAKE_SOURCE_DIR}/src/application
    ${CMAKE_SOURCE_DIR}/src/dataStructure
    ${CMAKE_SOURCE_DIR}/src/wrapper
    "D:/Qt/6.7.3/msvc2019_64/include"
    "D:/Qt/6.7.3/msvc2019_64/include/QtOpenGLWidgets"
    
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    Qt6::Widgets
    Qt6::OpenGLWidgets
    glm::glm
)

# 设置编译特性
set_target_properties(${PROJECT_NAME} PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    AUTOUIC ON
    AUTOMOC ON
    AUTORCC ON
)
