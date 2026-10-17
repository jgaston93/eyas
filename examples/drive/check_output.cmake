# Checks the driver's output CSV: LINES lines (header included), and no
# non-finite value in any row ("nan" or "inf" as written by iostreams).
# Usage: cmake -DFILE=<csv> -DLINES=<n> -P check_output.cmake
file(STRINGS "${FILE}" rows)
list(LENGTH rows n)
if(NOT n EQUAL LINES)
    message(FATAL_ERROR "${FILE}: ${n} lines, expected ${LINES}")
endif()
foreach(row IN LISTS rows)
    if(row MATCHES "(^|,)-?(nan|inf)(,|$)")
        message(FATAL_ERROR "${FILE}: non-finite value in: ${row}")
    endif()
endforeach()
