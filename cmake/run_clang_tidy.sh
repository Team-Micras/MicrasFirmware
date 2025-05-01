#!/bin/bash

# Usage: ./run_clang_tidy.sh [-j NUM_JOBS] [--fix] file1.cpp file2.cpp ...

# Default to using all available logical cores
NUM_JOBS=$(nproc)
FIX_FLAG=""
EXIT_CODE=0

# Parse the arguments
while [[ $# -gt 0 ]]; do
    case "$1" in
        -j)
            if [[ -n "$2" && "$2" =~ ^[0-9]+$ ]]; then
                NUM_JOBS="$2"
                shift
            else
                echo "Error: Invalid number of jobs specified with -j."
                exit 1
            fi
            ;;
        --fix)
            FIX_FLAG="--fix"
            ;;
        -*)
            echo "Error: Unknown option $1"
            exit 1
            ;;
        *)
            FILES+=("$1")
            ;;
    esac
    shift
done

# Ensure at least one file is provided
if [[ "${#FILES[@]}" -lt 1 ]]; then
    echo "Usage: $0 [-j NUM_JOBS] [--fix] file1.cpp file2.cpp ..."
    exit 1
fi

# Initialize a semaphore to limit the number of parallel jobs
SEMAPHORE_FILE="/tmp/run_clang_tidy_semaphore_$$"
mkfifo "$SEMAPHORE_FILE"
exec 3<>"$SEMAPHORE_FILE"
rm "$SEMAPHORE_FILE"

# Add tokens to the semaphore for job limiting
for ((i = 0; i < NUM_JOBS; i++)); do
    echo "token" >&3
done

# Function to run clang-tidy on a single file
run_clang_tidy() {
    local file="$1"

    # Wait for a token (block if none are available)
    read -r -u 3

    # Run clang-tidy
    echo "Running clang-tidy on $file..."
    clang-tidy $FIX_FLAG --quiet -p build "$file"
    if [[ $? -ne 0 ]]; then
        echo "clang-tidy failed on $file" >&2
        EXIT_CODE=1
    fi

    # Return the token to the semaphore
    echo "token" >&3
}

# Export variables and functions for subshells
export FIX_FLAG EXIT_CODE
export -f run_clang_tidy

# Track background jobs
declare -a PIDS

# Process each file
for file in "${FILES[@]}"; do
    run_clang_tidy "$file" &
    PIDS+=("$!")
done

# Wait for all background jobs to finish
for pid in "${PIDS[@]}"; do
    wait "$pid" || EXIT_CODE=1
done

# Close the semaphore
exec 3>&-

# Return the aggregated exit code
exit $EXIT_CODE
