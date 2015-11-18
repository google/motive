Building for Windows    {#motive_guide_windows}
====================

# Version Requirements    {#motive_guide_windows_version}

Following are the minimum required versions of tools and libraries you
need to build [Motive][] for Windows:

   * [Windows][]: 7
   * [Visual Studio][]: 2010
   * [CMake][]: 2.8.12.1

# Prerequisites    {#motive_guide_windows_prerequisites}

Prior to building, install the following:

   * [Visual Studio][]
   * [CMake][] from [cmake.org](http://cmake.org).

# Building with Visual Studio    {#motive_guide_windows_building}

Generate the [Visual Studio][] project using [CMake][]:

   * Open a command line window.
   * Go to the [Motive][] project directory.
   * Use [CMake][] to generate the [Visual Studio][] project.

~~~{.sh}
    cd motive
    cmake -G "Visual Studio 11 2012" .
~~~

Open the [Motive][] solution in [Visual Studio][].

   * Double-click on `motive/Motive.sln` to open the solution.
   * Select "Build-->Build Solution" from the menu.

To run the unit tests from [Visual Studio][],

   * Right click on a project, for example `angle_test`, select
     "Set as StartUp Project" from the menu.
   * Select "Debug-->Start Debugging", or press F5.


<br>

  [CMake]: http://www.cmake.org
  [Motive]: @ref motive_overview
  [Visual Studio]: http://www.visualstudio.com/
  [Windows]: http://windows.microsoft.com/

