# OpenEXR Submodule Patches

## Overview

The OpenEXR submodule (`src/deps/openexr`) requires patches to compile with modern C++ compilers due to missing `<cstdint>` includes.

## Required Changes

Add `#include <cstdint>` to the following files in the OpenEXR submodule:

### File 1: `src/deps/openexr/openexr-2.3.0/IlmImf/ImfTiledMisc.h`

```cpp
// After existing includes (around line 19):
#include <ImathBox.h>

#include <cstdint>  // ADD THIS LINE
#include <stdio.h>
#include <vector>
```

### File 2: `src/deps/openexr/openexr-2.3.0/IlmImf/ImfDeepTiledInputPart.h`

```cpp
// After existing includes (around line 13):
#include <ImathBox.h>
#include <cstdint>  // ADD THIS LINE

OPENEXR_IMF_INTERNAL_NAMESPACE_HEADER_ENTER
```

### File 3: `src/deps/openexr/openexr-2.3.0/IlmImf/ImfDeepTiledInputFile.h`

```cpp
// After existing includes (around line 22):
#include <ImathBox.h>
#include <cstdint>  // ADD THIS LINE

OPENEXR_IMF_INTERNAL_NAMESPACE_HEADER_ENTER
```

## Why This Is Needed

Modern C++ compilers (GCC 11+, Clang 13+) require explicit inclusion of `<cstdint>` for types like `int64_t` and `uint64_t`. The OpenEXR 2.3.0 headers used in this submodule predate this requirement.

## Application

These patches must be applied manually after initializing submodules:

```bash
git submodule update --init --recursive
cd src/deps/openexr
# Apply patches to the three files listed above
cd ../../..
```

Alternatively, set `HSIM_NO_SUBMODULE_UPDATE=1` when building if you've already applied the patches:

```bash
HSIM_NO_SUBMODULE_UPDATE=1 python -m build --wheel
```

## Upstream Status

These changes are local patches. The OpenEXR submodule points to an older version (2.3.0) that is no longer maintained. Newer versions of OpenEXR (3.x+) have fixed these issues but would require significant integration work.

## Related

- Part of the UV/WSL2 support effort
- See `UV_PROJECT.md` in the tbp.monty repository for full build instructions
