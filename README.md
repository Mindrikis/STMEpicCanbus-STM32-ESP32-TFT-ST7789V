# Epic CAN bus monorepo

Two separate **PlatformIO** projects live here. Each has its own `platformio.ini`.

## Build in VS Code / Cursor

1. **Recommended:** open **`Epic canbus.code-workspace`** (File → Open Workspace from File…). Use the PlatformIO sidebar: pick the project (`ESP32displaystandard` or `STM32 canbus`) before **Build**.

2. **Alternative:** open **`ESP32displaystandard`** or **`STM32 canbus`** as the single workspace folder (not the parent `Epic canbus` folder alone — there is no `platformio.ini` at the repo root, so PlatformIO will not treat the parent as a project).

## After moving or cloning this repo

- Regenerate **`compile_commands.json`** once per project (clangd / IDE). In a normal terminal, `pio` is often **not** on your `PATH` even when the PlatformIO extension can build from the GUI.
  - **Windows:** from `ESP32displaystandard` or `STM32 canbus`, run **`compiledb.cmd`** (in this repo), or:

    `"%USERPROFILE%\.platformio\penv\Scripts\platformio.exe" run -t compiledb`

  - **If you use the “PlatformIO CLI” terminal** from the extension, `pio` / `platformio` may already work there.
- Those `compile_commands.json` files are gitignored (machine-specific paths).
- Do not commit auto-generated `.vscode/c_cpp_properties.json` or `launch.json` from an old disk path; PlatformIO can recreate them from the correct project location.
