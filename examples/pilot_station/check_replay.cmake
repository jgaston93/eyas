# ExampleStation.RecordingReplaysExactly (D-044): the pilot station flies a
# control file headless and records it; eyas_drive replays the recording with
# the command in its header; the two state histories must be byte-identical.
#
#   cmake -DSTATION=... -DDRIVE=... -DCONFIG=... -DCONTROLS=... -DWORK=<dir>
#         [-DAFCS=<switches>] [-DSWITCH_SETS=<n>] [-DCOLUMN_VALUES=<col>=<n>;...]
#         -P check_replay.cmake
#
# AFCS, if given, is the station's --afcs (its start switches); a control
# file with an afcs column switches them in flight (M8c3). SWITCH_SETS, if
# given, is how many different sets of switches the state history must show
# (its afcs column), so that a replay that dropped the switches can't pass by
# agreeing with a station that dropped them too (code review). COLUMN_VALUES
# does the same for any state-history columns, by name (the engine controls,
# D-060); SWITCH_SETS is afcs=<n>.

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
    list(APPEND COLUMN_VALUES "afcs=${SWITCH_SETS}")
endif()
# Each column=n: the column, found by name, takes n different values.
list(GET lines 0 header)
string(REPLACE "," ";" header "${header}")
set(rows ${lines})
list(REMOVE_AT rows 0)
foreach(pair IN LISTS COLUMN_VALUES)
    string(REPLACE "=" ";" pair "${pair}")
    list(GET pair 0 column)
    list(GET pair 1 want)
    list(FIND header ${column} index)
    if(index LESS 0)
        message(FATAL_ERROR "no ${column} column in ${station_out}")
    endif()
    set(values)
    foreach(line IN LISTS rows)
        string(REPLACE "," ";" fields "${line}")
        list(GET fields ${index} value)
        list(APPEND values "${value}")
    endforeach()
    list(REMOVE_DUPLICATES values)
    list(LENGTH values n_values)
    if(NOT n_values EQUAL want)
        message(FATAL_ERROR "expected ${want} values of ${column}, got ${n_values}: ${values}")
    endif()
endforeach()
message(STATUS "replay identical: ${n} lines")
