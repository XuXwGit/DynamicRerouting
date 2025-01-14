cmake_minimum_required(VERSION 3.10)
project(YourProjectName)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_GENERATOR_PLATFORM x64)

# 添加头文件路径
include_directories(C:/gurobi1200/win64/include)

# 添加库文件路径
link_directories(C:/gurobi1200/win64/lib)

# 添加源文件
set(SOURCES
    Graph.cpp
    Model.cpp
    ArcBasedModel.cpp
    PathBasedModel.cpp
    LagrangePathBasedModel.cpp
    RoutingAlgorithms.cpp
    RouteManager.cpp
    sample_solve.cpp
    grader.cpp
)

# 生成可执行文件
add_executable(grader ${SOURCES})

# 链接库
# 多线程支持：gurobi_c++mt2017
# 动态链接：gurobi_c++md2017
# 调试支持：gurobi_c++mdd2017 或 gurobi_c++mtd2017
# 调试 + 多线程：gurobi_c++mtd2017
target_link_libraries(grader gurobi120 gurobi_c++mdd2017)
