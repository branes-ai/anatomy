# cmake/composition.cmake — a mission's slot registry, and a baseline's composition.
#
# A mission names its
# slots and registers the implementations each slot can take; for each
# baseline file, composition_generate() checks its `composition:` block against
# that registry, hashes it, and generates the header that defines the
# baseline's stack (racing-drone docs/adr/0001-baseline-composition-framework.md). The
# mission supplies the slot list, the registry name, and the header template
# (racing-drone: cmake/slot_registry.cmake, cmake/baselines.cmake).
include_guard(GLOBAL)

# composition_define_slots(<registry> <slot>...)
#   The registry's slots, in the order the generated stack takes them.
function(composition_define_slots registry)
    if(NOT ARGN)
        message(FATAL_ERROR "composition_define_slots(${registry}): no slots")
    endif()
    set_property(GLOBAL PROPERTY COMPOSITION_${registry}_SLOTS ${ARGN})
endfunction()

# composition_register_impl(<registry> <impl> SLOT <slot> HEADER <header> TYPE <type>)
#   Records, for the implementation name <impl>, the slot it fills, the header
#   that declares it, and its C++ class template (templated on the scalar type).
function(composition_register_impl registry impl)
    cmake_parse_arguments(RI "" "SLOT;HEADER;TYPE" "" ${ARGN})
    if(NOT RI_SLOT OR NOT RI_HEADER OR NOT RI_TYPE)
        message(FATAL_ERROR "composition_register_impl(${registry} ${impl}): SLOT, HEADER and TYPE are required")
    endif()
    get_property(slots GLOBAL PROPERTY COMPOSITION_${registry}_SLOTS)
    if(NOT RI_SLOT IN_LIST slots)
        message(FATAL_ERROR "composition_register_impl(${registry} ${impl}): unknown slot '${RI_SLOT}'")
    endif()
    set_property(GLOBAL PROPERTY COMPOSITION_${registry}_IMPL_${impl}_SLOT   ${RI_SLOT})
    set_property(GLOBAL PROPERTY COMPOSITION_${registry}_IMPL_${impl}_HEADER ${RI_HEADER})
    set_property(GLOBAL PROPERTY COMPOSITION_${registry}_IMPL_${impl}_TYPE   ${RI_TYPE})
endfunction()

# composition_generate(<registry> <name> FILE <baseline.yaml> TEMPLATE <header.in> OUTPUT <header>)
#
#   0. registers FILE as a configure dependency (editing it reruns CMake),
#   1. parses the `composition:` block of FILE (two-space-indented `slot: Impl` lines),
#   2. checks that every slot of the registry is named exactly once, with an
#      implementation registered for that slot,
#   3. hashes the canonical form "slot=Impl\n" (slots sorted) with SHA-256 —
#      the same canonical form config::composition_hash() computes at startup,
#   4. configures TEMPLATE into OUTPUT with @ONLY substitutions:
#        @BASELINE_NAME@      the baseline's name
#        @BASELINE_HASH@      the composition hash
#        @BASELINE_INCLUDES@  one #include line per implementation header
#        @BASELINE_TYPES@     the implementations' `Type<T>`, in slot order, comma-separated
function(composition_generate registry name)
    cmake_parse_arguments(PARSE_ARGV 2 arg "" "FILE;TEMPLATE;OUTPUT" "")
    if(arg_UNPARSED_ARGUMENTS OR NOT arg_FILE OR NOT arg_TEMPLATE OR NOT arg_OUTPUT)
        message(FATAL_ERROR "composition_generate(${registry} ${name}): FILE, TEMPLATE and OUTPUT are required")
    endif()
    set(file ${arg_FILE})
    if(NOT EXISTS ${file})
        message(FATAL_ERROR "composition_generate(${registry} ${name}): ${file} not found")
    endif()
    # Editing the baseline file reruns CMake, so the generated header (and its
    # composition hash) never goes stale. configure_file() tracks the template.
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${file})
    get_property(slots GLOBAL PROPERTY COMPOSITION_${registry}_SLOTS)

    # ── 1. parse the composition block ──────────────────────────────────
    file(STRINGS ${file} lines)
    set(in_block FALSE)
    foreach(line IN LISTS lines)
        if(line MATCHES "^composition:[ ]*$")
            set(in_block TRUE)
            continue()
        endif()
        if(in_block)
            if(line MATCHES "^[ ]*(#.*)?$")
                continue()  # blank or comment line inside the block
            elseif(line MATCHES "^  ([a-z_]+):[ ]*([A-Za-z0-9_]+)[ ]*(#.*)?$")
                set(slot ${CMAKE_MATCH_1})
                set(impl ${CMAKE_MATCH_2})
                if(DEFINED _impl_${slot})
                    message(FATAL_ERROR "${file}: slot '${slot}' named twice")
                endif()
                set(_impl_${slot} ${impl})
            elseif(line MATCHES "^[^ ]")
                set(in_block FALSE)  # next top-level key
            else()
                message(FATAL_ERROR "${file}: cannot parse composition line '${line}'")
            endif()
        endif()
    endforeach()

    # ── 2. validate against the registry ────────────────────────────────
    set(includes "")
    set(types "")
    foreach(slot IN LISTS slots)
        if(NOT DEFINED _impl_${slot})
            message(FATAL_ERROR "${file}: composition is missing slot '${slot}'")
        endif()
        set(impl ${_impl_${slot}})
        get_property(impl_slot GLOBAL PROPERTY COMPOSITION_${registry}_IMPL_${impl}_SLOT)
        if(NOT impl_slot)
            message(FATAL_ERROR "${file}: '${impl}' is not a registered implementation")
        endif()
        if(NOT impl_slot STREQUAL slot)
            message(FATAL_ERROR "${file}: '${impl}' fills slot '${impl_slot}', not '${slot}'")
        endif()
        get_property(header GLOBAL PROPERTY COMPOSITION_${registry}_IMPL_${impl}_HEADER)
        get_property(type   GLOBAL PROPERTY COMPOSITION_${registry}_IMPL_${impl}_TYPE)
        list(APPEND includes "#include <${header}>")
        list(APPEND types "${type}<T>")
    endforeach()
    list(REMOVE_DUPLICATES includes)

    # ── 3. canonical hash ───────────────────────────────────────────────
    set(sorted_slots ${slots})
    list(SORT sorted_slots)
    set(canonical "")
    foreach(slot IN LISTS sorted_slots)
        string(APPEND canonical "${slot}=${_impl_${slot}}\n")
    endforeach()
    string(SHA256 hash "${canonical}")

    # ── 4. generated header ─────────────────────────────────────────────
    list(JOIN includes "\n" BASELINE_INCLUDES)
    list(JOIN types ",\n                                 " BASELINE_TYPES)
    set(BASELINE_NAME ${name})
    set(BASELINE_HASH ${hash})
    configure_file(${arg_TEMPLATE} ${arg_OUTPUT} @ONLY)
endfunction()
