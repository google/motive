Build Targets    {#motive_guide_build_targets}
=============

[Motive][] compiles to a simple C++ library. It includes [CMake][] and
[Android][] build scripts.

The CMake build script can be used to generate standard
makefiles on [Linux][] and [OS X][], [Xcode][] projects on [OS X][],
and [Visual Studio][] solutions on [Windows][]. See,

   * [Building for Linux][]
   * [Building for OS X][]
   * [Building for Windows][]

The Android build scripts will compile for Android on Linux, OSX, and Windows.
See,

   * [Building for Android][]

Or, since [Motive][] is a simple library with simple dependencies, it's easy
to add it to your own build by compiling the loose .cpp files.

   * [Building from loose cpp files][]

<br>

  [Android]: http://www.android.com
  [Building for Android]: @ref motive_guide_android
  [Building for Linux]: @ref motive_guide_linux
  [Building for OS X]: @ref motive_guide_osx
  [Building for Windows]: @ref motive_guide_windows
  [Building from loose cpp files]: @ref motive_guide_loose_cpps
  [CMake]: http://www.cmake.org/
  [Linux]: http://en.m.wikipedia.org/wiki/Linux
  [Motive]: @ref motive_overview
  [OS X]: http://www.apple.com/osx/
  [Visual Studio]: http://www.visualstudio.com/
  [Windows]: http://windows.microsoft.com/
  [Xcode]: http://developer.apple.com/xcode/
