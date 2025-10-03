#!/bin/bash
# Apply patches to OpenEXR submodule for modern C++ compiler compatibility

set -e

OPENEXR_DIR="src/deps/openexr/src/lib/OpenEXR"

echo "Applying OpenEXR patches..."

# Patch 1: ImfTiledMisc.h
FILE1="${OPENEXR_DIR}/ImfTiledMisc.h"
if ! grep -q "include <cstdint>" "$FILE1"; then
    echo "Patching $FILE1"
    sed -i '/^#include <ImathBox.h>/a #include <cstdint>' "$FILE1"
else
    echo "$FILE1 already patched"
fi

# Patch 2: ImfDeepTiledInputPart.h
FILE2="${OPENEXR_DIR}/ImfDeepTiledInputPart.h"
if ! grep -q "include <cstdint>" "$FILE2"; then
    echo "Patching $FILE2"
    sed -i '/^#include <ImathBox.h>/a #include <cstdint>' "$FILE2"
else
    echo "$FILE2 already patched"
fi

# Patch 3: ImfDeepTiledInputFile.h
FILE3="${OPENEXR_DIR}/ImfDeepTiledInputFile.h"
if ! grep -q "include <cstdint>" "$FILE3"; then
    echo "Patching $FILE3"
    sed -i '/^#include <ImathBox.h>/a #include <cstdint>' "$FILE3"
else
    echo "$FILE3 already patched"
fi

echo "OpenEXR patches applied successfully"
