# Eye Asset Tool

This tool converts custom eye material PNG files into a C header that can be used by:

- [dual_eye_render.cc](/d:/esp32/xiaozhi-esp32-main/main/display/dual_eye_render.cc)

## What you need to prepare

Two PNG files:

- `sclera.png`
  Size must be `375x375`
  RGB image for the eyeball base, sclera, shading, veins, etc.

- `iris.png`
  Size must be `80x512`
  RGB image for the iris material used by the polar mapping table

The script does not resize automatically on purpose. It checks the size and fails fast if the source image is not prepared correctly.

## Install dependency

```bash
pip install pillow
```

## Generate a header

```bash
python scripts/eye_asset_tools/convert_eye_assets.py ^
  --sclera my_sclera.png ^
  --iris my_iris.png ^
  --name my_eye ^
  --output main/display/my_eyes_data.h
```

This generates:

- `my_eye_sclera`
- `my_eye_iris`

## Use in code

Include your generated header and switch the material macros in:

- [dual_eye_render.cc](/d:/esp32/xiaozhi-esp32-main/main/display/dual_eye_render.cc)

Example:

```cpp
#include "my_eyes_data.h"

#define EYE_SCLERA_MATERIAL my_eye_sclera
#define EYE_IRIS_MATERIAL my_eye_iris
```

## Notes

- `upper_default`
- `lower_default`
- `polar_default`

These are still reused from the existing project. For most custom styles you only need to replace `sclera` and `iris`.
