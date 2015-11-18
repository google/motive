Introduction    {#motive_guide_introduction}
============

# About Motive    {#motive_guide_about_motive}

[Motive][] is an animation system written in cross-platform C++.
It's designed to be,
  * **performant** -- data is stored centrally and processed in bulk so that we
    can take advantage of [SIMD] and multi-threading opportunities.
  * **compact** -- spline data is preferred over sampled data, so animations
    requires less memory.
  * **expandable** -- new animation algorithms can be registered and utilized
    without touching central code.
Motive has been tested on [Android], [Windows], [OS X], and [Linux], but
should be portable to any platform that supports C++.

You can see [Motive][] animation in use in the [Zooshi][] and [Pie Noon][]
sample games.

# Prerequisites    {#motive_guide_prerequisites}

[Motive][] is written in C++. You should be familiar with C++ coding
environments and tools.

# Concepts    {#motive_guide_concepts}

In [Motive][], a [Motivator][] encapsulates an animating variable. There are
currently two kinds of [Motivator][]s: One to animate a float, and one to
animate a 4x4 [matrix][].

Each [Motivator][] is driven by a [MotiveProcessor][]. Each MotiveProcessor
implements a different algorithm for animating a variable. There are several
MotiveProcessors and a Motivator may be switched between them.

A [MotiveProcessor][] holds the animation data for **all** `Motivators` that
it drives. This animation data is updated in bulk during the [MotiveEngine][]
`AdvanceFrame()` call. The high performance of the Motive System comes from
the optimization oportunities provided by this bulk update.

All [Motivator][]s are updated at the same time in one call to
`MotiveEngine::AdvanceFrame()`. You can have multiple [MotiveEngine][]s,
if required, but you will get the best performance if you have just one.

# Optimization    {#motive_guide_optimization}

[Motive][] has [SIMD][] versions of essential functions for [NEON][] processors.

[Motive][] uses the [MathFu][] [vector][] math library. For optimial performance,
you should use the [SIMD][] implementation of [MathFu][] by specifying
<code>MATHFU_COMPILE_WITH_SIMD</code>.


  [matrix]: http://en.wikipedia.org/wiki/Matrix_(mathematics)
  [vector]: http://en.wikipedia.org/wiki/Euclidean_vector
  [MathFu]: https://github.com/google/mathfu
  [Motivator]: @ref motive_guide_motivators
  [Motive]: @ref motive_overview
  [MotiveProcessor]: @ref motive_guide_processors
  [MotiveEngine]: @ref motive_guide_engine
  [Pie Noon]: https://github.com/google/pienoon
  [Zooshi]:  https://github.com/google/zooshi
  [SIMD]: http://en.wikipedia.org/wiki/SIMD
  [NEON]: http://www.arm.com/products/processors/technologies/neon.php
  [Android]: http://www.android.com
  [Linux]: http://en.m.wikipedia.org/wiki/Linux
  [OS X]: http://www.apple.com/osx/
  [Windows]: http://windows.microsoft.com/
