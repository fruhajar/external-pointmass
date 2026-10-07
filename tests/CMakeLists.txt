add_library(pm_test_support INTERFACE)
target_include_directories(pm_test_support INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

function(add_pm_test name)
  add_executable(${name} ${name}.cpp)
  target_link_libraries(${name} PRIVATE ballistics-pointmass pm_test_support)
  add_test(NAME ${name} COMMAND ${name} ${ARGN})
endfunction()

add_pm_test(test_atmosphere)
add_pm_test(test_drag)
add_pm_test(test_wind)
add_pm_test(test_geo)
add_pm_test(test_predict)
add_pm_test(test_solve)
add_pm_test(test_catalogue)
add_pm_test(test_drift)
add_test(NAME self_contained
  COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/check_self_contained.sh ${CMAKE_CURRENT_SOURCE_DIR}/..)


# Locks the committed catalogue to the generator, so every derived number in it stays traceable.
find_package(Python3 COMPONENTS Interpreter QUIET)
if(Python3_Interpreter_FOUND)
  add_test(NAME pm_catalogue_generated
    COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_CURRENT_SOURCE_DIR}/../tools/generate_catalogue.py --check)
endif()
