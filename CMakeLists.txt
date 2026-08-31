cmake_minimum_required(VERSION 3.21)
project(zrdds_uav_ugv_scheduler LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")

set(ZRDDS_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." CACHE PATH "ZRDDS SDK root")
set(GENERATED_DIR "${CMAKE_CURRENT_LIST_DIR}/generated")

set(GENERATED_SOURCES
    "${GENERATED_DIR}/mission.cpp"
    "${GENERATED_DIR}/missionTypeSupport.cpp")

foreach(source_file IN LISTS GENERATED_SOURCES)
    if(NOT EXISTS "${source_file}")
        message(FATAL_ERROR "Generated type source is missing: ${source_file}. Run scripts/generate_types.ps1 first.")
    endif()
endforeach()

add_executable(scheduler_demo src/main.cpp src/task_plan.cpp ${GENERATED_SOURCES})
target_include_directories(scheduler_demo PRIVATE
    "${ZRDDS_ROOT}/include/ZRDDSCoreInterface"
    "${ZRDDS_ROOT}/include/CPlusPlusInterface"
    "${GENERATED_DIR}")
target_compile_definitions(scheduler_demo PRIVATE _ZRDDSCPPINTERFACE _CRT_SECURE_NO_WARNINGS)
target_compile_options(scheduler_demo PRIVATE /W4 /permissive-)
target_link_directories(scheduler_demo PRIVATE "${ZRDDS_ROOT}/lib")
target_link_libraries(scheduler_demo PRIVATE
    debug ZRDDSCppzd_VS2015
    optimized ZRDDSCppz_VS2015)

add_custom_command(TARGET scheduler_demo POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${ZRDDS_ROOT}/lib/ZRDDSCppd_VS2015.dll"
        "$<TARGET_FILE_DIR:scheduler_demo>/ZRDDSCppd_VS2015.dll"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${ZRDDS_ROOT}/lib/ZRDDSCpp_VS2015.dll"
        "$<TARGET_FILE_DIR:scheduler_demo>/ZRDDSCpp_VS2015.dll")
