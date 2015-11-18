Motivators   {#motive_guide_motivators}
==========

# Overview

A `Motivator` is an animated variable. You can use,

   * `Motivator1fs` to animate `floats`,
   * `Motivator2fs`, `Motivator3fs`, and `Motivator4fs` to animate
      2, 3, and 4 dimensional vectors,
   * `MatrixMotivator4fs` to animate 4x4 matrices, and
   * `RigMotivators` to animate entire skeletons.

# Simple example {#motive_guide_motivator_example}

Suppose you have a duck that follows screen touches.
You could use a `Motivator2f` to represent the duck's position on screen.
Whenever a touch occurs, the duck's `Motivator2f` is updated with a new
target position, and the duck smoothly animates to it.

The code would look something like this:

\snippet src/samples/duck2f/duck2f.cpp Duck Example

Please see `src/samples/duck2f/duck2f.cpp` for the complete source of this
example.

# Behind-the-scenes example

The next example illustrates how to animate the face-angle of a character.
The comments describe, in brief, what is going on internally.

\snippet src/samples/smooth1f/smooth1f.cpp Motivator Example

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
`src/samples/smooth1f/smooth1f.cpp`.

# Motivators driving other Motivators

A `MatrixMotivator4f` is driven by a series of `Motivator1fs` that represent
rotation, translation, and scale operations. These operations are composed
to produce the final 4x4 matrix.

A `RigMotivator` is driven by an array of `MatrixMotivator4fs`.
Each 4x4 matrix represents the transform of a bone in the skeleton.

In general, more complicated `Motivators` are driven by simpler `Motivators`.

# Animation algorithms

Internally, a `Motivator` is a handle into a `MotiveProcessor`.
The `MotiveProcessor` holds all the data and does all the animating.
Each `MotiveProcessor` follows its own animation algorithm.
See [Motive Processors] for more details.

At any time, a `Motivator` can be reinitialized to use a different
animation algorithm from a different `MotiveProcessor`.

The external API remains the same, no matter what `MotiveProcessor`
is backing a `Motivator`. This means you can always call `Value()` and
`Velocity()` on a `Motivator1f` that's been initialized.


  [Motive Processors]: @ref motive_guide_processors