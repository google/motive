Building Motive from Loose Cpp Files   {#motive_guide_loose_cpps}
====================================

If you prefer, you can ignore the included build scripts, and manually add
[Motive][] to your build system by simply,
   * compliling the files under `motive/src`
   * adding `motive/include` to your list of include directories

Note that, when compiling `motive/src`, you'll also need these include
directories: `mathfu/include`, `mathfu/benchmarks`, and
`fplutil/libfplutil/include`.

Note also that compilation of `motive/src/io/flatbuffers.cpp` is optional
(see [below](@ref motive_guide_flatbuffers)).

# Enabling SIMD   {#motive_guide_simd}

[Motive][] runs best with `mathfu`'s [SIMD][] support enabled. Support is
enabled by default, but you may have to add a compiler flags to enable [SIMD][]
instructions on your compiler. Please see the `enable_simd` section of
[MathFu][]'s `CMakeLists.txt` file for details.

# Compile Defines   {#motive_guide_compile_defines}

You can add defines to enable the options below. For example, use
/D`define_name` with the Visual Studio (or use `Configuration Properties\
C,C++ \ Preprocessor`), or -D`define_name` on g++.

   * **BENCHMARK_MOTIVE** -- Analyse the runtime of each `MotiveProcessor` and
     periodically output a histogram of these runtimes.
   * **MOTIVE_ASSEMBLY_TEST** -- Define as `Neon` to run both the [NEON][] and C++
     versions of the code and compare the results. Useful for testing assembly
     language functions. Currently only [NEON][] functions exist.
   * **MOTIVE_NEON** -- Optimize with [NEON][] [SIMD][] instructions. To assemble
     bulk_spline_evaluator_neon.s, you'll need ensure that the [NEON][]
     instruction set is enabled on your assembler, for example with the
     `-mfpu=neon` option on gcc.
   * **FPL_CURVE_GRAPH_FUNCTIONS** -- Compile functions that draw ASCII art
     representations of mathematical curves. These are useful for debugging,
     but they also consume code space.

# Building Motive's FlatBuffer Support from Loose Cpps   {#motive_guide_flatbuffers}

You can optionally use [FlatBuffers][] to quickly load your [Motive][] data in
binary format, and to easily edit your data in text format ([json][]).

[Motive][]'s FlatBuffer support is accessed with
`motive/include/io/flatbuffers.h`. To add support for this header, you'll need
to compile `motive/src/io/flatbuffers.cpp`.

Before you compile `motive/src/io/flatbuffers.cpp`, you need to call the
FlatBuffer compiler `flatc` to generate a header file for motive's
FlatBuffer schemas.

~~~{.sh}
flatc --gen-includes -o generated_includes -c motive/schemas/motive.fbs
~~~

This will create `generated_includes/motive_generated.h`. You should add
`generated_includes` to your include path when compiling
`motive/src/io/flatbuffers.cpp`.

The `flatc` compiler can be downloaded from the latest [FlatBuffer release][],
or you can build it yourself (see [FlatBuffers build instructions][]).



<br>

  [FlatBuffers]: https://github.com/google/flatbuffers
  [FlatBuffers build instructions]: https://google.github.io/flatbuffers/md__building.html
  [FlatBuffer release]: https://github.com/google/flatbuffers/releases
  [json]: http://json.org
  [MathFu]: https://github.com/google/mathfu
  [Motive]: @ref motive_overview
  [NEON]: http://www.arm.com/products/processors/technologies/neon.php
  [SIMD]: http://en.wikipedia.org/wiki/SIMD
  [Visual Studio]: http://www.visualstudio.com/
