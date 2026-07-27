# PlaneGCS (FreeCAD Sketcher solver) - vendored for OneCAD
add_library(planegcs STATIC
    GCS.cpp
    Constraints.cpp
    SubSystem.cpp
    Geo.cpp
    qp_eq.cpp
)

find_package(Boost REQUIRED CONFIG)

target_include_directories(planegcs
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${Boost_INCLUDE_DIRS}
)

target_link_libraries(planegcs
    PUBLIC
    Eigen3::Eigen
)

target_compile_definitions(planegcs PRIVATE EIGEN_NO_DEBUG)
set_target_properties(planegcs PROPERTIES POSITION_INDEPENDENT_CODE ON)
