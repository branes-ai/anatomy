# cmake/flow.cmake — a mission's operator registry, and a baseline's graph
# (anatomy docs/adr/0004, step 2; docs/plan/flow-graph, 2b).
#
# A mission registers the operators, bodies and research graphs a baseline
# file may name. For each baseline file, flow_add_baseline() reads its
# `class:` and `graph:` and configures the header that names what fills the
# graph: for a scoreboard baseline, a body type per composite; for a research
# baseline, its graph's type. The graph itself is C++ (flow/builder.hpp): the
# mission's stack builds it at startup, checks the file against it
# (flow::check_baseline) and computes its hash, so nothing here hashes.
include_guard(GLOBAL)

# flow_register_operator(<registry> <Name> HEADER <header> TYPE <type> BINDING <binding>)
#   An operator: its C++ class template and its binding's, templated on the
#   scalar type. A body names it as flow::registered<"Name", type<T>, binding<T>>.
function(flow_register_operator registry name)
    cmake_parse_arguments(RO "" "HEADER;TYPE;BINDING" "" ${ARGN})
    if(NOT RO_HEADER OR NOT RO_TYPE OR NOT RO_BINDING)
        message(FATAL_ERROR "flow_register_operator(${registry} ${name}): HEADER, TYPE and BINDING are required")
    endif()
    _flow_register(${registry} ${name} operator ${RO_HEADER}
        "branes::anatomy::flow::registered<\"${name}\", ${RO_TYPE}<T>, ${RO_BINDING}<T>>")
endfunction()

# flow_register_body(<registry> <Name> HEADER <header> TYPE <type>)
#   A body of several operators: its C++ class template, on the scalar type.
function(flow_register_body registry name)
    cmake_parse_arguments(RB "" "HEADER;TYPE" "" ${ARGN})
    if(NOT RB_HEADER OR NOT RB_TYPE)
        message(FATAL_ERROR "flow_register_body(${registry} ${name}): HEADER and TYPE are required")
    endif()
    _flow_register(${registry} ${name} operator ${RB_HEADER} "${RB_TYPE}<T>")
endfunction()

# flow_register_graph(<registry> <Name> HEADER <header> TYPE <type>)
#   A research graph: its C++ class template, on the scalar type.
function(flow_register_graph registry name)
    cmake_parse_arguments(RG "" "HEADER;TYPE" "" ${ARGN})
    if(NOT RG_HEADER OR NOT RG_TYPE)
        message(FATAL_ERROR "flow_register_graph(${registry} ${name}): HEADER and TYPE are required")
    endif()
    _flow_register(${registry} ${name} graph ${RG_HEADER} "${RG_TYPE}<T>")
endfunction()

function(_flow_register registry name kind header type)
    if(NOT name MATCHES "^[A-Za-z0-9_]+$")
        message(FATAL_ERROR "flow registry ${registry}: '${name}' must match [A-Za-z0-9_]+ (G1)")
    endif()
    get_property(known GLOBAL PROPERTY FLOW_${registry}_${name}_KIND)
    if(known)
        message(FATAL_ERROR "flow registry ${registry}: '${name}' is registered twice")
    endif()
    set_property(GLOBAL PROPERTY FLOW_${registry}_${name}_KIND ${kind})
    set_property(GLOBAL PROPERTY FLOW_${registry}_${name}_HEADER ${header})
    set_property(GLOBAL PROPERTY FLOW_${registry}_${name}_TYPE "${type}")
endfunction()

# flow_add_baseline(<registry> <name> FILE <baseline.yaml> TEMPLATE <header.in> OUTPUT <header>)
#
#   0. registers FILE as a configure dependency (editing it reruns CMake),
#   1. reads its `class:` (scoreboard or research) and its `graph:`: for a
#      scoreboard baseline, two-space-indented `composite: Body` lines; for a
#      research baseline, `graph: Name`,
#   2. checks that each name is registered, as an operator or a body
#      (scoreboard) or as a graph (research),
#   3. configures TEMPLATE into OUTPUT with @ONLY substitutions:
#        @FLOW_NAME@      the baseline's name
#        @FLOW_IDENT@     the name as a C++ identifier (`-` becomes `_`)
#        @FLOW_CLASS@     scoreboard or research
#        @FLOW_INCLUDES@  one #include line per header named
#        @FLOW_BODIES@    `using <composite> = <type>;` per composite (scoreboard)
#        @FLOW_GRAPH@     the research graph's type (research), or void
#   The mission's checks against the graph (G8) run at startup, in C++.
function(flow_add_baseline registry name)
    cmake_parse_arguments(PARSE_ARGV 2 arg "" "FILE;TEMPLATE;OUTPUT" "")
    if(arg_UNPARSED_ARGUMENTS OR NOT arg_FILE OR NOT arg_TEMPLATE OR NOT arg_OUTPUT)
        message(FATAL_ERROR "flow_add_baseline(${registry} ${name}): FILE, TEMPLATE and OUTPUT are required")
    endif()
    set(file ${arg_FILE})
    if(NOT EXISTS ${file})
        message(FATAL_ERROR "flow_add_baseline(${registry} ${name}): ${file} not found")
    endif()
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${file})

    # ── 1. read the class and the graph ─────────────────────────────────
    file(STRINGS ${file} lines)
    set(class "")
    set(graph "")
    set(composites "")
    set(in_block FALSE)
    foreach(line IN LISTS lines)
        if(in_block)
            if(line MATCHES "^[ ]*(#.*)?$")
                continue()
            elseif(line MATCHES "^  ([a-z_][a-z0-9_]*):[ ]*([A-Za-z0-9_]+)[ ]*(#.*)?$")
                if(CMAKE_MATCH_1 IN_LIST composites)
                    message(FATAL_ERROR "${file}: composite '${CMAKE_MATCH_1}' named twice")
                endif()
                list(APPEND composites ${CMAKE_MATCH_1})
                set(_body_${CMAKE_MATCH_1} ${CMAKE_MATCH_2})
                continue()
            elseif(line MATCHES "^[^ ]")
                set(in_block FALSE)
            else()
                message(FATAL_ERROR "${file}: cannot parse graph line '${line}'")
            endif()
        endif()
        if(line MATCHES "^class:[ ]*([a-z]+)[ ]*(#.*)?$")
            set(class ${CMAKE_MATCH_1})
        elseif(line MATCHES "^graph:[ ]*(#.*)?$")
            set(in_block TRUE)
        elseif(line MATCHES "^graph:[ ]*([A-Za-z0-9_]+)[ ]*(#.*)?$")
            set(graph ${CMAKE_MATCH_1})
        endif()
    endforeach()

    # ── 2. check the names ──────────────────────────────────────────────
    set(includes "")
    set(bodies "")
    set(graph_type "")
    if(class STREQUAL "scoreboard")
        if(NOT composites OR graph)
            message(FATAL_ERROR "${file}: a scoreboard graph maps each composite to its body (G8)")
        endif()
        foreach(c IN LISTS composites)
            set(body ${_body_${c}})
            get_property(kind GLOBAL PROPERTY FLOW_${registry}_${body}_KIND)
            if(NOT kind STREQUAL "operator")
                message(FATAL_ERROR "${file}: '${body}' is not a registered operator or body (G2)")
            endif()
            get_property(header GLOBAL PROPERTY FLOW_${registry}_${body}_HEADER)
            get_property(type GLOBAL PROPERTY FLOW_${registry}_${body}_TYPE)
            list(APPEND includes "#include <${header}>")
            string(APPEND bodies "    using ${c} = ${type};\n")
        endforeach()
    elseif(class STREQUAL "research")
        if(NOT graph OR composites)
            message(FATAL_ERROR "${file}: a research graph is the name of a registered graph (G8)")
        endif()
        get_property(kind GLOBAL PROPERTY FLOW_${registry}_${graph}_KIND)
        if(NOT kind STREQUAL "graph")
            message(FATAL_ERROR "${file}: '${graph}' is not a registered graph (G8)")
        endif()
        get_property(header GLOBAL PROPERTY FLOW_${registry}_${graph}_HEADER)
        get_property(graph_type GLOBAL PROPERTY FLOW_${registry}_${graph}_TYPE)
        list(APPEND includes "#include <${header}>")
    else()
        message(FATAL_ERROR "${file}: a graph baseline declares its class, scoreboard or research (G8)")
    endif()
    list(REMOVE_DUPLICATES includes)

    # ── 3. the header ───────────────────────────────────────────────────
    list(JOIN includes "\n" FLOW_INCLUDES)
    string(REPLACE "-" "_" FLOW_IDENT ${name})
    set(FLOW_NAME ${name})
    set(FLOW_CLASS ${class})
    set(FLOW_BODIES "${bodies}")
    if(graph_type)
        set(FLOW_GRAPH "${graph_type}")
    else()
        set(FLOW_GRAPH void)
    endif()
    configure_file(${arg_TEMPLATE} ${arg_OUTPUT} @ONLY)
endfunction()
