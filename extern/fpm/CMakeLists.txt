set(FPM_HEADER_FILES
    fixed.hpp
    ios.hpp
    math.hpp)

add_library(fpm INTERFACE ${FPM_HEADER_FILES})
target_include_directories(fpm INTERFACE ..)

set_target_properties(fpm PROPERTIES
    CXX_STANDARD 11
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
)

INSTALL(EXPORT fpm DESTINATION include)

source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${FPM_HEADER_FILES})
