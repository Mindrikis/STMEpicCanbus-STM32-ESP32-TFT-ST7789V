@echo off
REM Regenerate compile_commands.json for clangd (pio is not on PATH in plain PowerShell).
cd /d "%~dp0"
"%USERPROFILE%\.platformio\penv\Scripts\platformio.exe" run -t compiledb %*
