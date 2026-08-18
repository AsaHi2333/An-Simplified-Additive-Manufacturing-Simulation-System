# 收集所有源文件
file(GLOB_RECURSE APPLICATION_SOURCES 
    "*.cpp"
    
)

# 创建静态库
add_library(application STATIC ${APPLICATION_SOURCES})

# 设置包含目录
target_include_directories(application 
    PUBLIC 
        ${CMAKE_CURRENT_SOURCE_DIR}
)

# 查找所需的包
find_package(GLEW CONFIG REQUIRED)
find_package(glfw3 CONFIG REQUIRED)
find_package(glad CONFIG REQUIRED)
find_package(glm CONFIG REQUIRED)

# 链接必要的依赖
target_link_libraries(application
    PUBLIC
        glfw
        glm::glm
        wrapper
        glframework
)

# 设置编译特性
target_compile_features(application PRIVATE cxx_std_11)
set_target_properties(application PROPERTIES CXX_EXTENSIONS OFF)

# # 设置输出目录
# set_target_properties(glframework PROPERTIES 
#     ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
#     LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
# )
