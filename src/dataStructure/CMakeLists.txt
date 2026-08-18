# 收集所有源文件
file(GLOB_RECURSE DATASTRUCTURE_SOURCES 
    "*.cpp"
    
)

# 创建静态库
add_library(dataStructure STATIC ${DATASTRUCTURE_SOURCES})

# 设置包含目录
target_include_directories(dataStructure 
    PUBLIC 
        ${CMAKE_CURRENT_SOURCE_DIR}
)

# 查找所需的包
find_package(GLEW CONFIG REQUIRED)
find_package(glfw3 CONFIG REQUIRED)
find_package(glad CONFIG REQUIRED)
find_package(glm CONFIG REQUIRED)

# 链接必要的依赖
target_link_libraries(dataStructure
    PUBLIC
        glm::glm
)

# 设置编译特性
target_compile_features(dataStructure PRIVATE cxx_std_11)
set_target_properties(dataStructure PROPERTIES CXX_EXTENSIONS OFF)

