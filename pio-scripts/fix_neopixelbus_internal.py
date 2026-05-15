"""
PlatformIO extra script: add NeoPixelBus src/internal/ to the include path.

NeoPixelBus uses nested relative includes (e.g. NeoColors.h includes
"colors/NeoGammaEquationMethod.h").  When PlatformIO adds the library via
-isystem, GCC no longer resolves those relative includes from the file's own
directory, so the nested subfolder is not found.  Adding src/internal/ as a
regular -I path fixes this without touching the library source.
"""
import os
import glob

Import("env")  # noqa: F821 (SCons built-in)

project_dir = env.subst("$PROJECT_DIR")
pioenv      = env["PIOENV"]

# Search for NeoPixelBus in this environment's libdeps folder
pattern = os.path.join(project_dir, ".pio", "libdeps", pioenv, "NeoPixelBus*", "src", "internal")
matches = glob.glob(pattern)

for internal_path in matches:
    if os.path.isdir(internal_path):
        env.Prepend(CPPPATH=[internal_path])
        print(f"fix_neopixelbus_internal: added {internal_path}")
        break
