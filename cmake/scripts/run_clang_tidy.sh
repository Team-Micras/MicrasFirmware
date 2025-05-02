#!/bin/bash

FIX_FLAG=""
EXIT_CODE=0

if [[ "$1" == "--fix" ]]; then
    FIX_FLAG="--fix"
    shift
fi

run_clang_tidy() {
    local output=$(clang-tidy $FIX_FLAG --quiet -p build $1 2>&1)

    if [[ $? -ne 0 ]]; then
        echo "clang-tidy failed on $1" >&2
        echo "$output" >&2
        EXIT_CODE=1
    fi
}

export FIX_FLAG EXIT_CODE
export -f run_clang_tidy

declare -a PIDS
echo "Running clang-tidy on all source files..."

for file in "$@"; do
    run_clang_tidy "$file" &
    PIDS+=("$!")
done

for pid in "${PIDS[@]}"; do
    wait "$pid" || EXIT_CODE=1
done

exit $EXIT_CODE
