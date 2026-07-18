cmake_minimum_required(VERSION 3.16)

set(DOGBOT_TARGET_ARCH "arm64")
set(DOGBOT_TARGET_TRIPLET "aarch64-linux-gnu")
set(DOGBOT_SYSROOT "/opt/sysroots/arm64")

if(NOT EXISTS "${DOGBOT_SYSROOT}")
  message(FATAL_ERROR "DOGBOT_SYSROOT does not exist: ${DOGBOT_SYSROOT}")
endif()

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)
set(CMAKE_SYSROOT "${DOGBOT_SYSROOT}")
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CMAKE_LIBRARY_ARCHITECTURE "${DOGBOT_TARGET_TRIPLET}")
set(CMAKE_C_LIBRARY_ARCHITECTURE "${DOGBOT_TARGET_TRIPLET}")
set(CMAKE_CXX_LIBRARY_ARCHITECTURE "${DOGBOT_TARGET_TRIPLET}")

set(_dogbot_find_root_path "${DOGBOT_SYSROOT}")
foreach(_dogbot_prefix_env IN ITEMS AMENT_PREFIX_PATH CMAKE_PREFIX_PATH)
  if(DEFINED ENV{${_dogbot_prefix_env}} AND NOT "$ENV{${_dogbot_prefix_env}}" STREQUAL "")
    string(REPLACE ":" ";" _dogbot_env_prefixes "$ENV{${_dogbot_prefix_env}}")
    list(APPEND _dogbot_find_root_path ${_dogbot_env_prefixes})
  endif()
endforeach()
list(REMOVE_DUPLICATES _dogbot_find_root_path)
set(CMAKE_FIND_ROOT_PATH
  "${_dogbot_find_root_path}"
)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

function(dogbot_find_host_program output_var)
  set(options)
  set(oneValueArgs ENV_VAR)
  set(multiValueArgs NAMES)
  cmake_parse_arguments(DOGBOT_FIND_HOST_PROGRAM "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  if(NOT DOGBOT_FIND_HOST_PROGRAM_NAMES)
    message(FATAL_ERROR "dogbot_find_host_program requires at least one candidate in NAMES.")
  endif()

  unset(_dogbot_resolved_program CACHE)
  unset(_dogbot_resolved_program)
  string(REPLACE ":" ";" _dogbot_host_path_entries "$ENV{PATH}")
  if(DOGBOT_FIND_HOST_PROGRAM_ENV_VAR
     AND DEFINED ENV{${DOGBOT_FIND_HOST_PROGRAM_ENV_VAR}}
     AND NOT "$ENV{${DOGBOT_FIND_HOST_PROGRAM_ENV_VAR}}" STREQUAL "")
    set(_dogbot_program_candidates "$ENV{${DOGBOT_FIND_HOST_PROGRAM_ENV_VAR}}")
    if(IS_ABSOLUTE "${_dogbot_program_candidates}")
      if(NOT EXISTS "${_dogbot_program_candidates}" OR IS_DIRECTORY "${_dogbot_program_candidates}")
        message(FATAL_ERROR
          "${DOGBOT_FIND_HOST_PROGRAM_ENV_VAR} points to a missing executable: ${_dogbot_program_candidates}"
        )
      endif()
      set(_dogbot_resolved_program "${_dogbot_program_candidates}")
    else()
      find_program(_dogbot_resolved_program
        NAMES "${_dogbot_program_candidates}"
        PATHS ${_dogbot_host_path_entries}
        NO_DEFAULT_PATH
      )
    endif()
  else()
    set(_dogbot_program_candidates ${DOGBOT_FIND_HOST_PROGRAM_NAMES})
    find_program(_dogbot_resolved_program
      NAMES ${_dogbot_program_candidates}
      PATHS ${_dogbot_host_path_entries}
      NO_DEFAULT_PATH
    )
  endif()

  if(NOT _dogbot_resolved_program)
    list(JOIN _dogbot_program_candidates ", " _dogbot_program_candidates_str)
    message(FATAL_ERROR "Cannot resolve host program. Tried: ${_dogbot_program_candidates_str}")
  endif()

  set(${output_var} "${_dogbot_resolved_program}" PARENT_SCOPE)
endfunction()

dogbot_find_host_program(DOGBOT_C_COMPILER ENV_VAR CC NAMES "${DOGBOT_TARGET_TRIPLET}-gcc")
dogbot_find_host_program(DOGBOT_CXX_COMPILER ENV_VAR CXX NAMES "${DOGBOT_TARGET_TRIPLET}-g++")
dogbot_find_host_program(DOGBOT_PYTHON_EXECUTABLE NAMES python3)
dogbot_find_host_program(DOGBOT_PKG_CONFIG_EXECUTABLE NAMES pkg-config pkgconf)

if(NOT DOGBOT_C_COMPILER)
  message(FATAL_ERROR "Cannot find C compiler: ${DOGBOT_TARGET_TRIPLET}-gcc")
endif()
if(NOT DOGBOT_CXX_COMPILER)
  message(FATAL_ERROR "Cannot find CXX compiler: ${DOGBOT_TARGET_TRIPLET}-g++")
endif()

set(CMAKE_C_COMPILER "${DOGBOT_C_COMPILER}")
set(CMAKE_CXX_COMPILER "${DOGBOT_CXX_COMPILER}")
set(Python3_EXECUTABLE "${DOGBOT_PYTHON_EXECUTABLE}" CACHE FILEPATH "Host Python interpreter" FORCE)
set(AMENT_PYTHON_EXECUTABLE "${DOGBOT_PYTHON_EXECUTABLE}" CACHE FILEPATH "Host Python interpreter for ament" FORCE)
set(PKG_CONFIG_EXECUTABLE "${DOGBOT_PKG_CONFIG_EXECUTABLE}" CACHE FILEPATH "Host pkg-config executable" FORCE)

# NOTE: use the config-dir path (not usr/lib/<triplet>) so CMake keeps the full
# sysroot path: usr/lib/<triplet> is an implicit linker dir and CMake would strip
# the sysroot prefix, generating a Makefile dependency on a non-existent host path.
set(_dogbot_python_library "${DOGBOT_SYSROOT}/usr/lib/python3.10/config-3.10-${DOGBOT_TARGET_TRIPLET}/libpython3.10.so")
set(_dogbot_python_include_dir "${DOGBOT_SYSROOT}/usr/include/python3.10")
if(NOT EXISTS "${_dogbot_python_library}" OR NOT EXISTS "${_dogbot_python_include_dir}")
  message(FATAL_ERROR "Target Python not found in sysroot: ${_dogbot_python_library} / ${_dogbot_python_include_dir}")
endif()
set(PYTHON_LIBRARY "${_dogbot_python_library}" CACHE FILEPATH "Target Python library" FORCE)
set(PYTHON_LIBRARIES "${_dogbot_python_library}" CACHE FILEPATH "Target Python libraries" FORCE)
set(PYTHON_INCLUDE_DIR "${_dogbot_python_include_dir}" CACHE PATH "Target Python include dir" FORCE)
set(PYTHON_INCLUDE_DIRS "${_dogbot_python_include_dir}" CACHE PATH "Target Python include dirs" FORCE)
set(Python3_LIBRARY "${_dogbot_python_library}" CACHE FILEPATH "Target Python3 library" FORCE)
set(Python3_LIBRARIES "${_dogbot_python_library}" CACHE FILEPATH "Target Python3 libraries" FORCE)
set(Python3_INCLUDE_DIR "${_dogbot_python_include_dir}" CACHE PATH "Target Python3 include dir" FORCE)
set(Python3_INCLUDE_DIRS "${_dogbot_python_include_dir}" CACHE PATH "Target Python3 include dirs" FORCE)
set(PYTHON_SOABI "cpython-310-${DOGBOT_TARGET_TRIPLET}" CACHE STRING "Target Python SOABI" FORCE)

if(CMAKE_GENERATOR MATCHES "Makefiles")
  dogbot_find_host_program(DOGBOT_MAKE_PROGRAM NAMES gmake make)
  set(CMAKE_MAKE_PROGRAM "${DOGBOT_MAKE_PROGRAM}" CACHE FILEPATH "Host make program" FORCE)
endif()

set(CMAKE_PREFIX_PATH
  "/opt/ros/humble"
  "/usr/local"
  "/usr"
  CACHE STRING "Target-side prefix path for cross-compilation"
)

set(ENV{PKG_CONFIG_SYSROOT_DIR} "${DOGBOT_SYSROOT}")
set(_dogbot_pkg_config_libdirs
  "${DOGBOT_SYSROOT}/usr/local/lib/${DOGBOT_TARGET_TRIPLET}/pkgconfig"
  "${DOGBOT_SYSROOT}/usr/local/lib/pkgconfig"
  "${DOGBOT_SYSROOT}/usr/local/share/pkgconfig"
  "${DOGBOT_SYSROOT}/usr/lib/${DOGBOT_TARGET_TRIPLET}/pkgconfig"
  "${DOGBOT_SYSROOT}/usr/lib/pkgconfig"
  "${DOGBOT_SYSROOT}/usr/share/pkgconfig"
  "${DOGBOT_SYSROOT}/lib/${DOGBOT_TARGET_TRIPLET}/pkgconfig"
  "${DOGBOT_SYSROOT}/lib/pkgconfig"
  "${DOGBOT_SYSROOT}/opt/ros/humble/lib/${DOGBOT_TARGET_TRIPLET}/pkgconfig"
  "${DOGBOT_SYSROOT}/opt/ros/humble/lib/pkgconfig"
  "${DOGBOT_SYSROOT}/opt/ros/humble/share/pkgconfig"
)
list(JOIN _dogbot_pkg_config_libdirs ":" _dogbot_pkg_config_libdir)
set(ENV{PKG_CONFIG_LIBDIR} "${_dogbot_pkg_config_libdir}")
