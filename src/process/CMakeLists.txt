# 收集所有源文件
file(GLOB_RECURSE PROCESS_SOURCES CONFIGURE_DEPENDS
    "*.cpp"
    
)

# 创建静态库
add_library(process STATIC ${PROCESS_SOURCES})

# 设置包含目录
target_include_directories(process 
    PUBLIC 
        ${CMAKE_CURRENT_SOURCE_DIR}
)

# 查找所需的包
find_package(GLEW CONFIG REQUIRED)
find_package(glfw3 CONFIG REQUIRED)
find_package(glad CONFIG REQUIRED)
find_package(glm CONFIG REQUIRED)

# 链接必要的依赖
target_link_libraries(process
    PUBLIC
        glm::glm
        dataStructure
)

# 设置编译特性
target_compile_features(process PRIVATE cxx_std_11)
set_target_properties(process PROPERTIES CXX_EXTENSIONS OFF)

