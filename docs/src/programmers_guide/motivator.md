Motivators
==========

# Overview   {#motive_guide_motivators}

A `Motivator` is an animated variable. There are currently two kinds:
`Motivator1f`--which animates a single `float`, and `MotivatorMatrix4f`--which
animates a 4x4 matrix of `floats`.

One `Motivator` can drive another. For example, a series of `Motivator1fs` can
drive the rotations, translations, and scales that compose a
`MotivatorMatrix4f`.

Internally, a `Motivator` is a simple reference into a `MotiveProcessor`. The
`MotiveProcessor` holds all the data and does all the animating. At any time,
the `Motivator` can be reinitialized to use a different animation algorithm
from a different `MotiveProcessor`. The external API remains the same,
no matter what `MotiveProcessor` is backing a `Motivator`, although not all
`MotiveProcessors` implement all API features.


# Example Usage   {#motive_guide_motivator_example}

The following example illustrates how to animate the face-angle of a character.
The comments describe, in brief, what is going on internally.

\snippet docs/samples/smooth1f.cpp Motivator Example

This program generates the output below. Notice that the face angle is
animated smoothly from 120 degrees to -120 degrees.

Notice also that we take the shortest path by passing through 180 degrees,
instead of 0. We get this behavior by setting `modular_arithmetic` to `true`
in `SmoothInit`.

    y = 178.020294
    |
    |                                        ***
    |                                    ****
    |                                ****
    |                          ******
    |              ************
    ***************
    |
    |
    |
    |
    |
    |
    |
    |
    *--------------------------------------------------------------------------------
    |
    |
    |
    |
    |
    |
    |
    |
    |
    |                                                                     **********
    |                                                            **********
    |                                                      ******
    |                                                  *****
    |                                             *****
    |                                          ***
    y = -179.489014

If you'd like to experiment with this program, it is compiled for you in
`docs/samples/smooth1f.cpp`.