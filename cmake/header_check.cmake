# anatomy_header_check(NAME <name> ROOT <include-dir> LIBS <targets>...)
#
# Every header under ROOT must compile on its own, against LIBS and nothing
# else (anatomy docs/plan/split-libraries). For each header the function
# generates a translation unit that includes only that header, and compiles
# them all as one object library, <name>, in `all`. A header that leans on an
# include it does not make, or that includes a header of a library LIBS do not
# reach (a core header including a mobility one), fails the build.

include_guard(GLOBAL)
include(${CMAKE_CURRENT_LIST_DIR}/warnings.cmake)

function(anatomy_header_check)
    cmake_parse_arguments(HC "" "NAME;ROOT" "LIBS" ${ARGN})
    if(NOT HC_NAME OR NOT HC_ROOT OR NOT HC_LIBS)
        message(FATAL_ERROR "anatomy_header_check: NAME, ROOT and LIBS are required")
    endif()
    file(GLOB_RECURSE headers CONFIGURE_DEPENDS RELATIVE ${HC_ROOT} ${HC_ROOT}/*.hpp)
    if(NOT headers)
        message(FATAL_ERROR "anatomy_header_check(${HC_NAME}): no headers under ${HC_ROOT}")
    endif()
    set(units)
    foreach(header IN LISTS headers)
        string(MAKE_C_IDENTIFIER ${header} id)
        set(unit ${CMAKE_CURRENT_BINARY_DIR}/${HC_NAME}/${id}.cpp)
        file(CONFIGURE OUTPUT ${unit} CONTENT "#include <${header}>\n")
        list(APPEND units ${unit})
    endforeach()
    add_library(${HC_NAME} OBJECT ${units})
    target_link_libraries(${HC_NAME} PRIVATE ${HC_LIBS})
    target_compile_features(${HC_NAME} PRIVATE cxx_std_20)
    anatomy_set_warnings(${HC_NAME})
    set_target_properties(${HC_NAME} PROPERTIES FOLDER "Tests/headers")
endfunction()
