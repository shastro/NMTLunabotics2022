#!/bin/sh

# This script applies the default clang-format rules to every C++ file
# in the current directory.

for FILE in $(find -type f | grep '.cpp$'); do
    clang-format -i "$FILE"
done
