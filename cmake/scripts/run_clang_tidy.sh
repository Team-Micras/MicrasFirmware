#!/bin/bash

FIX_FLAG=""
EXIT_CODE=0

if [[ "$1" == "--fix" ]]; then
    FIX_FLAG="--fix"
    shift
fi

run_clang_tidy() {
    local output
    output=$(clang-tidy $FIX_FLAG --quiet -p build $1 2>&1)

    if [[ $? -ne 0 ]]; then
        echo "$output"
        return 1
    fi
}

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
