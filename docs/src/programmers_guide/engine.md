MotiveEngine  {#motive_guide_engine}
============

The `MotiveEngine` is the gateway to the [MotiveProcessor][]s. When a
[Motivator][] is initialized, you must provide a reference to a `MotiveEngine`.
The [Motivator][] queries the `MotiveEngine` to initialize itself with the
appropriate [MotiveProcessor][].

All [Motivator][]s under a `MotiveEngine` are updated at the same time, by
calling `MotiveEngine::AdvanceFrame()`. This eliminates the overhead of updating
every [Motivator][] individually, and indeed, it provides many other
opportunities for optimizations.

You can have several `MotiveEngines` in your program, if you like, but you
will have the best performance by sticking to just one, if possible.

  [Motivator]: @ref motive_guide_motivators
  [MotiveProcessor]: @ref motive_guide_processors
