Introduction    {#motive_guide_introduction}
============

# About Motive    {#motive_guide_about_motive}

[Motive][] is an animation system written for speed and low memory usage.

The current version is feature light. It's suitable for procedural animation
of props, cameras, and UI elements. It currently lacks the data pipeline
required for full character animation, so it is not suitable for that.

You can see [Motive][] animation in use in the [Pie Noon][]
sample game.

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
  [SIMD]: http://en.wikipedia.org/wiki/SIMD
  [NEON]: http://www.arm.com/products/processors/technologies/neon.php
