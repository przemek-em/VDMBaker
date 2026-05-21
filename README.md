# VDMBaker

Standalone native Win32 Vector Displacement Map baker for OBJ meshes. It writes uncompressed RGBA32F `.exr` files and has no third-party runtime dependencies.

## Features

- Single-mesh exact UV XYZ bake.
- Two-mesh rest/base to target/detail bake.
- Fast matched vertex-order mode for true `target - rest` displacement.
- Built-in BVH closest-surface projection when target topology differs.
- Built-in Wavefront OBJ parser for vertex positions, UVs, and triangle/ngon faces.
- Built-in RGBA32F OpenEXR writer.
- Small x64 Windows executable.

## Build

Requirements:

- Windows
- Visual Studio 2022 or Microsoft C++ Build Tools

Run:

```bat
build.bat
```

The script uses the x64 MSVC environment and writes:

```text
build\VDMBaker.exe
```

Manual build from an x64 Native Tools Command Prompt:

```bat
cl /EHsc /O2 /MT /std:c++17 /DUNICODE /D_UNICODE VDMBaker.cpp /link /SUBSYSTEM:WINDOWS User32.lib Gdi32.lib Comdlg32.lib Comctl32.lib
```

## Usage

Launch `VDMBaker.exe`, choose input OBJ files, choose an output `.exr`, then press `Run`.

The output is RGBA32F:

- `R`: X displacement or X object position
- `G`: Y displacement or Y object position
- `B`: Z displacement or Z object position
- `A`: coverage mask, usually `1.0` for baked pixels

## Bake Modes

### Single Mesh: Exact UV XYZ

Use one sculpted OBJ that still contains the original unmodified plane/grid UVs. This mode requires face UV indices like:

```text
f v/vt[/vn] v/vt[/vn] v/vt[/vn]
```

Do not regenerate, unwrap, normalize, or destroy UVs after sculpting. The UVs define the 2D EXR layout, and the mesh vertex positions define the displaced surface.

### Two Meshes: Rest/Base -> Target/Detail

Use an unmodified rest/base mesh plus a sculpted target/detail mesh.

Best quality comes from matching vertex order. In that case the baker computes exact mesh displacement from rest position to target position. If topology differs, the baker projects each rest sample to the nearest target surface using the built-in BVH.

Only the rest/base UVs define the 2D EXR layout. Target/detail UVs are ignored.

## Important Pipeline Notes

For vector displacement, UVs are part of the data. If X/Y channels are nearly zero and only Z changes, the source is usually heightfield-like relative to the selected base plane, or the original UV domain was lost.

Recommended workflow:

1. Create a plane/grid with clean UVs.
2. Sculpt or move vertices in X, Y, and Z.
3. Preserve the original UVs through the whole pipeline.
4. Export OBJ with face UV references.
5. Bake in `Single Mesh: Exact UV XYZ` or export rest+target and bake in `Two Meshes`.

## Settings

- `Domain plane`: defines the base plane used for displacement mode.
- `Vector mode`: choose displacement from rest/base or object-space position.
- `Axis transform`: use Blender OBJ default if exporting from Blender with its common OBJ axis conversion.
- `Edge padding`: extrapolates values into empty texels near UV island edges.
- `Output scale`: multiplies final RGB values.

## License

Licensed under the MIT License. See the project license file for details.
