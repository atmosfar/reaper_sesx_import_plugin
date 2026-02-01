# REAPER Import Plugin for Adobe Audition Sessions

A native C++ plugin to handle the importing of Audition .sesx files as Reaper projects.

This project builds on the previous Python script, [Audition session to Reaper project converter](https://github.com/atmosfar/audition_session_to_reaper_project_converter) and includes all of the same features, plus a few more.

## Supported features
Project:
- Markers / Regions

Track:
- Name
- Colour
- Mute / Volume / Pan
- Solo / Record / Monitor
- Volume envelope
- Pan envelope
- Mute envelope

Clip:
- Position and length
- Crossfading
- Volume / Mute / Pan
- Volume envelope
- Pan envelope

Routing:
- Bus output routing and sends 
- Send Volume envelope
- Send Pan envelope
- Send Mute envelope

## Installation

Precompiled libraries are available for Apple Silicon and Windows x64.

### Mac:
- Copy `reaper_sesx_import-arm64.dylib` to `~/Library/Application Support/REAPER/UserPlugins`

### Windows:
- Copy `reaper_sesx_import-x64.dll` to `%APPDATA%\REAPER\UserPlugins`


## Building

If you would prefer to build from source, follow these steps. The compiled library binary should then be automatically copied to the right plugin path as defined in `CMakeLists.txt`.

### Mac:
```
mkdir build
cmake --build build
```

### Windows:

You will need Visual Studio 2022 or similar installed for MSVC compilation, as MINGW64 is not supported by the Reaper SDK.
```
mkdir build
cd build
cmake -G "Visual Studio 17 2022" -A x64 ..
cmake --build . --config Release
```

## Acknowledgements

This plugin builds on top of the "Hello World" example from @nukethebees. Read their blogpost [here](https://nukethebees.github.io/programming/cpp/reaper/2025/08/02/reaper-hello-world.html).

The C++ library [pugixml](https://github.com/zeux/pugixml) is used for parsing the XML in the Audition session files.
