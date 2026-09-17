#!/bin/bash

set -euo pipefail

mapfile -t FILES < <(find . \( -name "*.c" -o -name "*.cpp" -o -name "*.h" -o -name "*.hpp" \) \
    -not \( -path "*/cube/*" -o -path "*/build/*" -o -path "*/libs/*" \))

if ! clang-format --style=file --dry-run --Werror "${FILES[@]}"; then
    echo -e "\e[31mCode not properly formatted. Please run make format.\e[0m"
    exit 1
fi

echo -e "\e[32mCode properly formatted.\e[0m"
