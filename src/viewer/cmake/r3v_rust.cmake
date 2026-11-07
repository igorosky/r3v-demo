# Build the Rust submodules with Cargo and expose their C++ bindings to CMake.
include(ExternalProject)

find_program(CARGO_EXECUTABLE cargo
  HINTS "$ENV{CARGO_HOME}/bin" "$ENV{HOME}/.cargo/bin"
  REQUIRED)

set(_r3v_cargo_target_dir "${CMAKE_BINARY_DIR}/cargo")
if(NOT "$ENV{CARGO_TARGET_DIR}" STREQUAL "")
  set(_r3v_cargo_target_dir "$ENV{CARGO_TARGET_DIR}")
endif()
set(R3V_CARGO_TARGET_DIR "${_r3v_cargo_target_dir}" CACHE PATH
  "Cargo target directory for the r3v libraries")

function(r3v_add_rust_library name crate_dir lib_name features)
  if(NOT EXISTS "${crate_dir}/Cargo.toml")
    message(FATAL_ERROR
      "${crate_dir}/Cargo.toml is missing. Run: git submodule update --init --recursive")
  endif()

  set(lib_path
    "${R3V_CARGO_TARGET_DIR}/release/${CMAKE_SHARED_LIBRARY_PREFIX}${lib_name}${CMAKE_SHARED_LIBRARY_SUFFIX}")

  ExternalProject_Add(${name}_cargo
    SOURCE_DIR "${crate_dir}"
    CONFIGURE_COMMAND ""
    # Cargo cannot use the Make jobserver inherited from the parent build.
    BUILD_COMMAND
      "${CMAKE_COMMAND}" -E env --unset=MAKEFLAGS --unset=MFLAGS
      "${CARGO_EXECUTABLE}" build --release --locked --lib
      --features "${features}"
      --manifest-path "${crate_dir}/Cargo.toml"
      --target-dir "${R3V_CARGO_TARGET_DIR}"
    BUILD_ALWAYS ON
    BUILD_BYPRODUCTS "${lib_path}"
    INSTALL_COMMAND ""
    USES_TERMINAL_BUILD ON)

  add_library(r3v::${name} SHARED IMPORTED GLOBAL)
  set_target_properties(r3v::${name} PROPERTIES
    IMPORTED_LOCATION "${lib_path}"
    # Rust cdylibs have no SONAME. Link by name, so the installed executable
    # does not retain an absolute dependency on the Cargo build directory.
    IMPORTED_NO_SONAME TRUE
    INTERFACE_INCLUDE_DIRECTORIES "${crate_dir}/bindings")
  add_dependencies(r3v::${name} ${name}_cargo)

  set(${name}_LIBRARY "${lib_path}" PARENT_SCOPE)
endfunction()
