cmake_minimum_required(VERSION 3.17)

set(CMAKE_PREFIX_PATH D:\\vcpkg\\installed\\x64-windows\\share)

project(vcpkgTest)

# 添加assets目录 - 保持目录结构
file(COPY ${CMAKE_SOURCE_DIR}/src/assets DESTINATION ${CMAKE_BINARY_DIR})

add_executable(vcpkgTest src/main.cpp src/tool.cpp)

target_compile_features(vcpkgTest PRIVATE cxx_std_11)

set_target_properties(vcpkgTest PROPERTIES CXX_EXTENSIONS OFF)

# 添加fmt库
find_package(fmt CONFIG REQUIRED)
target_link_libraries(vcpkgTest PRIVATE fmt::fmt)

# 添加GLEW库
find_package(GLEW CONFIG REQUIRED)
target_link_libraries(vcpkgTest PRIVATE GLEW::GLEW)

# 添加FreeGLUT库
find_package(FreeGLUT CONFIG REQUIRED)
target_link_libraries(vcpkgTest PRIVATE $<IF:$<TARGET_EXISTS:FreeGLUT::freeglut>,FreeGLUT::freeglut,FreeGLUT::freeglut_static>)

# 添加glfw库
find_package(glfw3 CONFIG REQUIRED)
target_link_libraries(vcpkgTest PRIVATE glfw)

# 添加glad库
find_package(glad CONFIG REQUIRED)
target_link_libraries(vcpkgTest PRIVATE glad::glad)

# 添加glm库
find_package(glm CONFIG REQUIRED)
target_link_libraries(vcpkgTest PRIVATE glm::glm)

# 添加assimp库
find_package(assimp CONFIG REQUIRED)
if(assimp_FOUND)
    message("assimp found: ${assimp_DIR}")
else()
    message("assimp not found")
endif()
target_link_libraries(vcpkgTest PRIVATE assimp::assimp)

# 添加Qt6库
find_package(Qt6 COMPONENTS Widgets OpenGLWidgets REQUIRED)
if(Qt6_FOUND)
    message("Qt6 found: ${Qt6_DIR}")
else()
    message("Qt6 not found")
endif()

target_link_libraries(vcpkgTest PRIVATE Qt6::Widgets Qt6::OpenGLWidgets)






# 添加glframework子目录
add_subdirectory(src/glframework)
target_link_libraries(vcpkgTest PRIVATE glframework)

# 添加wrapper子目录
add_subdirectory(src/wrapper)
target_link_libraries(vcpkgTest PRIVATE wrapper)

# 添加application子目录
add_subdirectory(src/application)
target_link_libraries(vcpkgTest PRIVATE application)

# 添加dataStructure子目录
add_subdirectory(src/dataStructure)
target_link_libraries(vcpkgTest PRIVATE dataStructure)

# 添加process子目录（切片/几何处理）
add_subdirectory(src/process)
target_link_libraries(vcpkgTest PRIVATE process)

# 添加assimp头文件搜索路径
target_include_directories(vcpkgTest PRIVATE
    ${CMAKE_SOURCE_DIR}/src/third_party/include
)

# 添加ui子目录（用于Qt UI相关文件）
add_subdirectory(src/ui)
target_link_libraries(vcpkgTest PRIVATE ui)



# 链接assimp静态库和依赖库
target_link_libraries(vcpkgTest PRIVATE
    ${CMAKE_SOURCE_DIR}/src/third_party/libs/assimp-vc143-mtd.lib
    ${CMAKE_SOURCE_DIR}/src/third_party/libs/draco.lib   # 如果有其它依赖库也一并加上
    ${CMAKE_SOURCE_DIR}/src/third_party/libs/kubazip.lib
    ${CMAKE_SOURCE_DIR}/src/third_party/libs/minizip.lib
    ${CMAKE_SOURCE_DIR}/src/third_party/libs/poly2tri.lib
    ${CMAKE_SOURCE_DIR}/src/third_party/libs/polyclipping.lib
    ${CMAKE_SOURCE_DIR}/src/third_party/libs/pugixml.lib
    ${CMAKE_SOURCE_DIR}/src/third_party/libs/zlibd.lib
)



set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)






