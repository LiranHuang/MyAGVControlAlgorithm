Development Environment Setup Guide (Windows + VS Code)

This project requires C++17, CMake and several third-party libraries.
Below is the recommended setup (Windows 10/11 + VS Code).

1. Install Required Tools
VS Code
CMake
MSVC Build Tools
vcpkg Package Manager

2. Install vcpkg
powershell
"git clone https://github.com/microsoft/vcpkg C:\vcpkg
C:\vcpkg\bootstrap-vcpkg.bat
"

3. Install Dependencies via vcpkg
powershell
"C:\vcpkg\vcpkg.exe install eigen3 nlohmann-json paho-mqttpp3 paho-mqtt --triplet x64-windows
"

4. Configure CMake Toolchain in VS Code
Ctrl + Shift + P
"CMake: Edit User-Local CMake Kits
"

open/add file "cmake.configureSettings.json"
json
"cmake.configureSettings": {
    "CMAKE_TOOLCHAIN_FILE": "C:/vcpkg/scripts/buildsystems/vcpkg.cmake"
}

Ctrl + Shift + P
"CMake: Configure
"

5. VS Code IntelliSense Configuration
json
{
    "C_Cpp.default.configurationProvider": "ms-vscode.cmake-tools"
}
