function(stitch output threads)
    execute_process(COMMAND "${APP}" --synthetic --calibration "${SOURCE}/config/synthetic.calib"
        --frames 4 --stitch-threads "${threads}" --output "${output}"
        RESULT_VARIABLE result ERROR_VARIABLE log)
    if(NOT result EQUAL 0 OR NOT log MATCHES "paired=4")
        message(FATAL_ERROR "synthetic pipeline failed: ${log}")
    endif()
    file(SIZE "${output}" size)
    if(NOT size EQUAL 4866048)
        message(FATAL_ERROR "unexpected BGR stream size: ${size}")
    endif()
endfunction()

set(reference "${WORK}/smoke.bgr")
stitch("${reference}" 1)
if(PARALLEL)
    set(parallel "${WORK}/smoke-parallel.bgr")
    stitch("${parallel}" 4)
    file(SHA256 "${reference}" reference_sha)
    file(SHA256 "${parallel}" parallel_sha)
    if(NOT reference_sha STREQUAL parallel_sha)
        message(FATAL_ERROR "parallel stitching changed the output")
    endif()
endif()

execute_process(COMMAND "${APP}" --synthetic --calibration "${SOURCE}/config/synthetic.calib"
    --frames 1 --require-bound-calibration --output "${WORK}/should-not-exist.bgr"
    RESULT_VARIABLE result ERROR_VARIABLE log)
if(result EQUAL 0 OR NOT log MATCHES "must include the camera serials")
    message(FATAL_ERROR "unbound calibration accepted: ${log}")
endif()
