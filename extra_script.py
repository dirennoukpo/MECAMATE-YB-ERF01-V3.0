"""
PlatformIO build script — YB-ERF01 port (Keil/ARMCC -> arm-none-eabi-gcc).

Two jobs:

1. Place the flags that MUST land in CFLAGS / LINKFLAGS rather than in CCFLAGS.
   Put in `build_flags` of platformio.ini, SCons would file `--specs=...` and
   `-u _printf_float` under CCFLAGS, where they have no effect at link time:
   `_printf_float` would not be pulled in and EVERY %f would come out broken.

2. Produce a .hex: PlatformIO only emits .elf and .bin by default, whereas
   mcuisp.exe expects a .hex.
"""

Import("env", "projenv")

#
# 1a. -std=gnu99, on the .c files only (Keil: <uC99>1</uC99>).
#     In CCFLAGS this flag would make the assembler warn on the .S files.
#
#     /!\ This needs projenv, NOT env. `env` is the environment of the framework
#     and of the libraries; the project sources (src/) are compiled in `projenv`.
#     An env.Append(CFLAGS=...) here NEVER reaches our .c files: verified with
#     `pio run -v`, where -std= appeared on none of the 88 compile lines. The
#     build was therefore running in gnu11 (GCC 7.2's default) instead of gnu99.
#     No effect on the binary (gnu11 is a superset), but the .uvprojx declares
#     <uC99>1</uC99>, so we reproduce the configuration faithfully.
#
projenv.Append(CFLAGS=["-std=gnu99"])

#
# 1b. %f support in printf/sprintf.
#
#     newlib-nano (--specs=nano.specs, added by the cmsis builder) disables float
#     conversion by default: without -u _printf_float, printf("%.2f") prints a
#     bare "f". MicroLIB handled %f natively.
#
#     This is not merely a debug-trace concern: app_oled.c:89/97/99/101 calls
#     sprintf(text, "Voltage:%.1fV", ...) and displays the battery voltage and
#     the IMU angles on the screen. sprintf shares printf's conversion engine.
#
env.Append(LINKFLAGS=["-u", "_printf_float"])

#
# 2. firmware.hex, alongside the .elf and the .bin.
#
env.AddPostAction(
    "$BUILD_DIR/${PROGNAME}.elf",
    env.VerboseAction(
        " ".join([
            "$OBJCOPY", "-O", "ihex",
            "$BUILD_DIR/${PROGNAME}.elf",
            "$BUILD_DIR/${PROGNAME}.hex",
        ]),
        "Building $BUILD_DIR/${PROGNAME}.hex",
    ),
)
