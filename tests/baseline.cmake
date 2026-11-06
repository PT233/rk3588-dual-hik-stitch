string(RANDOM LENGTH 12 suffix)
set(report "${WORK}/基线-${suffix}")
execute_process(COMMAND "${PYTHON}" "${SOURCE}/tools/board_baseline.py" capture
    --app "${APP}" --calibration "${SOURCE}/config/synthetic.calib" --out "${report}"
    --frames 6 --warmup 2 --timeout 5 -- --synthetic
    RESULT_VARIABLE result ERROR_VARIABLE log)
if(NOT result EQUAL 0)
    message(FATAL_ERROR "baseline capture failed: ${log}")
endif()
file(READ "${report}/report.json" evidence)
if(NOT evidence MATCHES "\"status\": \"passed\"" OR NOT evidence MATCHES "\"source\": \"synthetic\"" OR
   NOT evidence MATCHES "\"warmup_excluded\": 2" OR NOT evidence MATCHES "\"pairs\": 6")
    message(FATAL_ERROR "unexpected report: ${evidence}")
endif()
file(STRINGS "${report}/trace/frames.jsonl" rows)
list(LENGTH rows count)
list(GET rows 5 last)
if(NOT count EQUAL 6 OR NOT last MATCHES "\"pair_index\":6" OR NOT last MATCHES "\"write_us\":null")
    message(FATAL_ERROR "invalid or incomplete self-check trace")
endif()

execute_process(COMMAND "${APP}" --calibration "${SOURCE}/config/synthetic.calib" --synthetic
    --fps 2 --check-pairs 6 --run-timeout-ms 900 --trace-dir "${report}/deadline"
    RESULT_VARIABLE result OUTPUT_VARIABLE stdout ERROR_VARIABLE log)
if(result EQUAL 0 OR NOT log MATCHES "capture run deadline exceeded" OR stdout MATCHES "passed")
    message(FATAL_ERROR "slow but advancing capture bypassed the deadline: ${log}")
endif()
