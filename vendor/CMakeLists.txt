add_subdirectory(mem)
add_subdirectory(fmt)

add_library(mmaplib INTERFACE)

target_include_directories(mmaplib INTERFACE
    cpp-mmaplib)
