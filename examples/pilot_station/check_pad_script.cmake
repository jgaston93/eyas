# ExampleStation.ScriptRunsFromTheStartPoint (code review): a script flown
# from the pad start (whose snapshot carries 40 s of settling) runs from the
# start point: the recording has the doublet's 1-s row, not one at 40 s.
#
#   cmake -DSTATION=... -DCONFIG=... -DCONTROLS=... -DWORK=<dir> -P check_pad_script.cmake

file(MAKE_DIRECTORY ${WORK})
set(rec ${WORK}/pad_recording.csv)
file(REMOVE ${rec})
execute_process(
    COMMAND ${STATION} --config ${CONFIG} --headless --no-pace --input script
            --controls ${CONTROLS} --start pad --duration 3 --record ${rec}
    RESULT_VARIABLE rc)
if(NOT rc EQUAL 0)
    message(FATAL_ERROR "the station failed (exit ${rc})")
endif()
file(STRINGS ${rec} rows REGEX "^[0-9]")
list(GET rows 0 first)
if(NOT first MATCHES "^0,")
    message(FATAL_ERROR "the first row isn't at t = 0: ${first}")
endif()
if(NOT "${rows}" MATCHES ";1,0,0,0\\.0(3|29999)[0-9]*,0")
    message(FATAL_ERROR "no doublet row at t = 1 s in: ${rows}")
endif()
message(STATUS "script runs from the start point: ${rows}")
