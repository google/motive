Building for OS X    {#motive_guide_osx}
=================

# Version Requirements    {#motive_guide_osx_version}

Following are the minimum required versions of tools and libraries you
need to build [Motive][] for [OS X]:

   * [OS X][]: Mavericks 10.9.1.
   * [Xcode][]: 5.0.1
   * [CMake][]: 2.8.12.1

# Prerequisites    {#motive_guide_osx_prerequisites}

   * Install [Xcode][].
   * Install [CMake][].

# Building with Xcode    {#motive_guide_osx_building}

Firstly, the [Xcode][] project needs to be generated using [CMake][]:

   * Open a command line window.
   * Go to the [Motive][] project directory.
   * Use [CMake][] to generate the [Xcode][] project.

~~~{.sh}
    cd motive
    cmake -G Xcode .
~~~

Then the project can be opened in [Xcode][] and built:

   * Double-click on `motive/Motive.xcodeproj` to open the project in
     [Xcode][].
   * Select "Product-->Build" from the menu.

The unit tests can be run from within [Xcode][]:

   * Select an application `Scheme`, for example
     "angle_test-->My Mac 64-bit", from the combo box to the right of the
     "Run" button.
   * Click the "Run" button.


# Building from the Command Line {#motive_guide_osx_cmdline}

To build:

   * Open a command line window.
   * Go to the [Motive][] project directory.
   * Use [CMake][] to generate the makefiles.
   * Run make.

For example:

~~~{.sh}
    cd motive
    cmake -G "Unix Makefiles" .
    make -j10
~~~

The unit tests can be run from the command line:

~~~{.sh}
    cd motive
    ./tests/angle_test
    ./tests/curve_test
    ./tests/motive_test
    ./tests/range_test
    ./tests/spline_test
~~~

<br>

  [CMake]: http://www.cmake.org
  [Motive]: @ref motive_overview
  [OS X]: http://www.apple.com/osx/
  [Xcode]: http://developer.apple.com/xcode/
