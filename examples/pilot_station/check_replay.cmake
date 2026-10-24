# ExampleStation.RecordingReplaysExactly (D-044): the pilot station flies a
# control file headless and records it; eyas_drive replays the recording with
# the command in its header; the two state histories must be byte-identical.
#
#   cmake -DSTATION=... -DDRIVE=... -DCONFIG=... -DCONTROLS=... -DWORK=<dir>
#         [-DAFCS=<switches>] [-DSWITCH_SETS=<n>] -P check_replay.cmake
#
# AFCS, if given, is the station's --afcs (its start switches); a control
# file with an afcs column switches them in flight (M8c3). SWITCH_SETS, if
# given, is how many different sets of switches the state history must show
# (its afcs column), so that a replay that dropped the switches can't pass by
# agreeing with a station that dropped them too (code review).

file(MAKE_DIRECTORY ${WORK})
set(rec ${WORK}/recording.csv)
set(station_out ${WORK}/station_state.csv)
set(drive_out ${WORK}/drive_state.csv)
file(REMOVE ${rec} ${station_out} ${drive_out})

# An origin other than the config's, so the override is exercised too.
set(afcs_args)
if(AFCS)
    set(afcs_args --afcs ${AFCS})
endif()
execute_process(
    COMMAND ${STATION} --config ${CONFIG} --headless --no-pace --input script
            --controls ${CONTROLS} --start 60 --origin 51.47,-0.45,25 --duration 10
            --record ${rec} --out ${station_out} ${afcs_args}
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
if(SWITCH_SETS)
    # The afcs column, found by name (the engine controls follow it).
    list(GET lines 0 header)
    string(REPLACE "," ";" header "${header}")
    list(FIND header afcs afcs_column)
    if(afcs_column LESS 0)
        message(FATAL_ERROR "no afcs column in ${station_out}")
    endif()
    list(REMOVE_AT lines 0)
    set(sets)
    foreach(line IN LISTS lines)
        string(REPLACE "," ";" fields "${line}")
        list(GET fields ${afcs_column} switches)
        list(APPEND sets "${switches}")
    endforeach()
    list(REMOVE_DUPLICATES sets)
    list(LENGTH sets n_sets)
    if(NOT n_sets EQUAL SWITCH_SETS)
        message(FATAL_ERROR "expected ${SWITCH_SETS} sets of AFCS switches, got ${n_sets}: ${sets}")
    endif()
endif()
message(STATUS "replay identical: ${n} lines")
