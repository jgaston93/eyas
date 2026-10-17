# ExampleStation.RecordingReplaysExactly (D-044): the pilot station flies a
# control file headless and records it; eyas_drive replays the recording with
# the command in its header; the two state histories must be byte-identical.
#
#   cmake -DSTATION=... -DDRIVE=... -DCONFIG=... -DCONTROLS=... -DWORK=<dir>
#         -P check_replay.cmake

file(MAKE_DIRECTORY ${WORK})
set(rec ${WORK}/recording.csv)
set(station_out ${WORK}/station_state.csv)
set(drive_out ${WORK}/drive_state.csv)
file(REMOVE ${rec} ${station_out} ${drive_out})

# An origin other than the config's, so the override is exercised too.
execute_process(
    COMMAND ${STATION} --config ${CONFIG} --headless --no-pace --input script
            --controls ${CONTROLS} --start 60 --origin 51.47,-0.45,25 --duration 10
            --record ${rec} --out ${station_out}
    RESULT_VARIABLE rc)
if(NOT rc EQUAL 0)
    message(FATAL_ERROR "the station failed (exit ${rc})")
endif()

file(STRINGS ${rec} header REGEX "^# Replay: eyas_drive ")
if(NOT header)
    message(FATAL_ERROR "${rec} has no replay command")
endif()
string(REGEX REPLACE "^# Replay: eyas_drive " "" args "${header}")
string(REPLACE "<out.csv>" "${drive_out}" args "${args}")
string(REPLACE "<s>" "10" args "${args}")
separate_arguments(args UNIX_COMMAND "${args}")
execute_process(COMMAND ${DRIVE} ${args} RESULT_VARIABLE rc)
if(NOT rc EQUAL 0)
    message(FATAL_ERROR "eyas_drive failed (exit ${rc}) on: ${args}")
endif()

execute_process(COMMAND ${CMAKE_COMMAND} -E compare_files ${station_out} ${drive_out}
                RESULT_VARIABLE rc)
if(NOT rc EQUAL 0)
    message(FATAL_ERROR "the replay differs: ${station_out} vs ${drive_out}")
endif()
file(STRINGS ${station_out} lines)
list(LENGTH lines n)
if(NOT n EQUAL 602)
    message(FATAL_ERROR "expected 602 lines (header, t = 0, 600 frames), got ${n}")
endif()
message(STATUS "replay identical: ${n} lines")
