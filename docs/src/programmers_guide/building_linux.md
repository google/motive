Building for Linux    {#motive_guide_linux}
==================

# Version Requirements    {#motive_guide_linux_version}

Following are the minimum required versions of tools and libraries you
need to build [Motive][] for Android:

   * [CMake][]: 2.8.12.1

# Prerequisites    {#motive_guide_linux_prerequisites}

Prior to building, install the following components using the [Linux][]
distribution's package manager:

   * [CMake][].  You can also manually install packages from
     [cmake.org](http://cmake.org).

For example, on [Ubuntu][]:

~~~{.sh}
    sudo apt-get install cmake
~~~

# Building    {#motive_guide_linux_building}

   * Open a command line window.
   * Go to the [Motive][] project directory.
   * Generate [Makefiles][] from the [CMake][] project. <br/>
   * Execute `make` to build the library and unit tests.

For example:

~~~{.sh}
    cd motive
    cmake -G'Unix Makefiles' .
    make
~~~

To perform a debug build:

~~~{.sh}
    cd motive
    cmake -G'Unix Makefiles' -DCMAKE_BUILD_TYPE=Debug .
    make
~~~

Build targets can be configured using options exposed in
`motive/CMakeLists.txt` by using [CMake]'s `-D` option.
Build configuration set using the `-D` option is sticky across subsequent
builds.

For example, if a build is performed using:

~~~{.sh}
    cd motive
    cmake -G"Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug .
    make
~~~

to switch to a release build CMAKE_BUILD_TYPE must be explicitly specified:

~~~{.sh}
    cd motive
    cmake -G"Unix Makefiles" -DCMAKE_BUILD_TYPE=Release .
    make
~~~

# Benchmarker Application  {#motive_guide_linux_benchmarker}

The `benchmarker` appliction is in the `benchmarker` directory.
This application creates 40000 [Motivators][] of various sorts, measures
their runtime, and reports these runtimes periodically as histograms.

To build the `benchmarker` application into `motive/bin/benchmark`,

~~~{.sh}
    cd motive/benchmarker
    cmake -G"Unix Makefiles" .
    make -j10
~~~

Run the benchmark application with,

~~~{.sh}
    cd motive
    ./bin/benchmark
~~~

# Unit Tests  {#motive_guide_linux_unit_tests}

The unit tests are in the `tests` directory. They are
built with the Motive library (See "Building" above).
Run tests from the `tests` directory.

~~~{.sh}
    cd motive
    ./tests/angle_test
    ./tests/curve_test
    ./tests/motive_test
    ./tests/range_test
    ./tests/spline_test
~~~


<br>

  [CMake]: http://www.cmake.org/
  [Linux]: http://en.wikipedia.org/wiki/Linux
  [Makefiles]: http://www.gnu.org/software/make/
  [Motive]: @ref motive_overview
  [Ubuntu]: http://www.ubuntu.com
