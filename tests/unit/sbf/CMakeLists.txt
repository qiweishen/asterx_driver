#
# Unit tests for the SBF frame synchronizer (Phase 3 — thin foundation slice).
#
# Uses the asterx_add_test() helper defined in tests/CMakeLists.txt.
# The helper:
#   * links asterx::core + GTest
#   * adds ${CMAKE_SOURCE_DIR}/src to the include path
#     (so tests can #include "sbf/frame_sync.hpp" etc.)
#   * stamps LABELS "unit" on every discovered test case
#

asterx_add_test(asterx_test_frame_sync
    SRCS test_frame_sync.cpp)
