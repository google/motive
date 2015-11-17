Building anim_pipeline    {#motive_guide_building_anim_pipeline}
======================

[anim_pipeline] converts animations made with modeling tools such as [Maya] or
[Blender] into Motive's spline format.

[Motive] includes [prebuilt anim_pipeline binaries] for [Windows], [OS X], and
[Linux].

If you'd like to make modifications to [anim_pipeline] you can also
build it from source by following the instructions below.

# Prerequisites

Prior to building, please install the prerequisites below.

   * Install the Motive [prerequisites for Windows], [prerequisites for OSX],
     or [prerequisites for Linux].
   * Then install the [FBX SDK] version 2015.1 or newer, at any location.
   * Set the `FBX_SDK` environment variable to the root directory of
     the FBX SDK.

# Building

   * Open a command line window.
   * Navigate to the `motive/src/anim_pipeline` directory.
   * Generate [Makefiles] from the [CMake] project.
   * Build the binary to the `bin` directory by:
     * executing `make`, on Linux (or OS X command line)
     * opening the Visual Studio solution and building, on Windows
     * opening the Xcode project and building, on OS X

For example, on Linux:

~~~{.sh}
    cd motive/src/anim_pipeline
    cmake -G'Unix Makefiles'
    make
~~~

<br>


  [anim_pipeline]: @ref motive_guide_anim_pipeline
  [Maya]: http://www.autodesk.com/products/maya/overview
  [Blender]: https://www.blender.org/
  [prebuilt anim_pipeline binaries]: @ref motive_guide_anim_pipeline_prebuilts
  [Windows]: http://windows.microsoft.com/
  [OS X]: http://www.apple.com/osx/
  [Linux]: http://en.m.wikipedia.org/wiki/Linux
  [prerequisites for Windows]: @ref motive_guide_windows
  [prerequisites for OSX]: @ref motive_guide_osx
  [prerequisites for Linux]: @ref motive_guide_linux
  [FBX SDK]: http://usa.autodesk.com/adsk/servlet/pc/item?siteID=123112&id=10775847
  [Makefiles]: http://www.gnu.org/software/make/
  [CMake]: http://www.cmake.org/
  [motive]: @ref motive_guide_introduction