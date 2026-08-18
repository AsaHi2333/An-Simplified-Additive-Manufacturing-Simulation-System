# 收集所有源文件
file(GLOB_RECURSE FORTEMPTEST_SOURCES 
    "*.cpp"
    
)

# 创建静态库
add_library(forTempTest STATIC ${FORTEMPTEST_SOURCES})

# 设置包含目录
target_include_directories(forTempTest 
    PUBLIC 
        ${CMAKE_CURRENT_SOURCE_DIR}
)

# 查找所需的包
find_package(GLEW CONFIG REQUIRED)
find_package(glfw3 CONFIG REQUIRED)
find_package(glad CONFIG REQUIRED)
find_package(glm CONFIG REQUIRED)

# 链接必要的依赖
target_link_libraries(forTempTest
    PUBLIC
        glad::glad
        glfw
        GLEW::GLEW
        glm::glm
)

# 设置编译特性
target_compile_features(forTempTest PRIVATE cxx_std_11)
set_target_properties(forTempTest PROPERTIES CXX_EXTENSIONS OFF)

# # 设置输出目录
# set_target_properties(glframework PROPERTIES 
#     ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
#     LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
# )
