# ESP32-S3 board profiles

Board profiles are small ESP-IDF `sdkconfig.defaults` fragments layered on top
of the project-level `sdkconfig.defaults`.

The default profile is `n16r8`, selected in `CMakeLists.txt`:

```cmake
set(NS2_ESP32S3_BOARD "n16r8" CACHE STRING "ESP32-S3 board profile")
```

To add a new board, create:

```text
boards/<profile>/sdkconfig.defaults
```

Then build with:

```powershell
idf.py -B build-<profile> -DNS2_ESP32S3_BOARD=<profile> build
```

Use one build directory per profile. The project keeps generated `sdkconfig`
files inside the build directory, so profiles can be built side by side without
reusing stale flash or PSRAM settings.

Current profiles:

```text
n4     4 MB flash, no required PSRAM; safest generic build.
n8     8 MB flash, no required PSRAM.
n8r2   8 MB flash, 2 MB Quad SPI PSRAM.
n8r8   8 MB flash, 8 MB Octal SPI PSRAM.
n16r8  16 MB flash, 8 MB Octal SPI PSRAM; current tested default.
```

For distribution, prefer building at least `n4`, `n8r2`, and `n16r8`. The `n4`
build keeps PSRAM disabled and fits within the default single-app partition, so
it is the best fallback for unknown ESP32-S3 boards with native USB. The `n8r2`
profile is the lowest-memory PSRAM board currently available for local testing.
