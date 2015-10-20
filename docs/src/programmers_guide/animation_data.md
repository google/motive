Animation Data   {#motive_guide_animation_data}
==============

# Overview

[Motive] animations are stored as a series of one-dimensional cubic [splines].
This is a distinguishing characteristic of Motive, since traditional animation
systems store animation data sampled at a constant frequency (typically 30Hz).

Splines provides two important advantages over sampled data:
1. Splines are much more **memory efficient**, since most animation data is
   intrinsically low-frequency and smooth.
2. Splines are **less prone to jitter**, since they have fewer values and
   continuous derivatives.

# Dual-Cubic Splines

One-dimensional polynomial [splines] can be unwieldy. Low degree splines tend
to overshoot the target. High degree splines tend to have extra wobbles
instead of a smooth curve (see [Runge's phenomenon]).

Most animation *authoring* solutions use two-dimensional [Bezier splines].
They simulate a one-dimensional function by labeling the axes 'time' vs.
'value', and then ensuring the curve never folds back on itself.

This works great for *authoring* curves; animators love creating animations
with Bezier splines. But *evaluating* curves is extremely slow--we must
iteratively search for a 'time' to find its 'value'--so Bezier splines are
not suitable as an in-game format for animations.

Dual cubic splines were developed for Motive as a way to maintain the speed
of polynomial splines, but temper their unwieldiness. Their premise is
simple: Whenever a cubic spline gets unwieldy, it is divided into two cubic
splines that are better behaved.

Motive's [anim_pipeline] converts Bezier splines and sampled data into
dual cubic splines.

At runtime, the dual cubic splines are quickly evaluated by processing them
in bulk, and by using the [SIMD] capabilities of the processor.

For more information on dual cubic splines, please read the
[dual cubic whitepaper] or watch the five minute [dual cubic lightning talk].

# Animation Data Classes

Animation data is built up from one-dimensional curves in `CompactSpline`.
`CompactSplines` drive matrix operations to create 4x4 matrices in
`MatrixAnim`. `MatrixAnims`, in turn, drive the individual bone animations
of a `RigAnim`.

## CompactSpline

One-dimensional spline data is held in `CompactSpline`. These splines are
a series of nodes. Each node has a time (`x`), value (`y`), and derivative
(`angle`).

When evaluating a `CompactSpline` we use the two neighboring nodes to
create a polynomial curve (in Motive, we use `CubicCurves`),
and then evaluate the curve to find its intermediate values.
The evaluation is handled efficiently by the `BulkSplineEvaluator`.

Each node element is quantized to 16-bits, so one node requires only
6-bytes. The quantizing assumes that individual animation curves will
vary over a limited enough range such that 16-bits (65536 possible values)
will still give enough precision.

We have not seen precision problems with quantization, though theoretically
it is possible. For example, if an animation translates extremely far at
one point, and then extremely subtly at another point.
Such animations may have to be divided into two animations.
If you encounter precision problems, we would be interested in seeing it
so please do contact us.

You can make a one-dimensional [Motivator] follow a CompactSpline by
initializing it with a `SmoothInit` and then calling `SetSpline()`.
See the [Motivator example] for details.

## MatrixAnim

A `MatrixAnim` holds the data to drive a `MotivatorMatrix4f`, the Motivator
that animates a 4x4 matrix.

A 4x4 matrix can move an entire model, without animating any of its component
parts. We call this "popsicle stick animation".
A 4x4 matrix can also animate a single bone of a model's skeleton.
That is how RigAnim uses it (see below).

MatrixAnim data is stored as a series of one-dimensional translate, rotate,
and scale operations. Each operation is driven by a `CompactSpline`
or a constant value. The series of operations are held in the
`MatrixOpArray` class.

There are no restrictions on the order or number of translates, rotates,
and scales. You can translate by x, rotate about y, and then translate by x
again. This is useful for when your rotation or scale pivot is non-zero,
for instance.

You can provide a constant transform from which to start applying the
series of operations. For example, `RigAnims` use this feature to represent
the constant transformation from a bone to its parent.

### Rotation Format

Rotations are output as [Euler angles] instead of [quaternions].
This has a few advantages,
1. It is much faster to apply an Euler angle to a transform matrix than
   a quaternion. Even applying three Euler angles is faster than one
   quaternion.
2. Most animations are authored in Eulers, so the runtime format will more
   closely match the authored value.
3. Euler angles easy to compact as 16-bit fixed point values.
   Quaternions are more difficult to represent as three 16-bit values
   (though theoretically that should be possible).
4. Euler angles are conceptually easier than quaternions.

[Gimbal lock] is often sited as the shortcoming of Euler angles.
If you experience Gimbal lock problems, we would be interested in hearing
from you. The runtime speedup alone is compelling enough reason to
stick with Eulers, but of course consequent problems (though we haven't
experienced any yet) will require solutions.


## RigAnim

A `RigAnim` holds a series of `MatrixAnims` and a bone hierarchy.
The `MatrixAnims` animate each bone relative to the bone's origin,
and the hierarchy lets us convert each bone's transform from bone-space
to model-space.

Each `RigAnim` has one "defining animation" that holds the union of all
operations for each bone. The defining animation is calculated from a set
of animations in `CreateDefiningAnim()`. It's important to initialize the
underlying `MotivatorMatrix4fs` with the union of all operations so that
we can smoothly blend between animations. For example, even if the target
animation has a matrix operation that the current animation does not have,
we can still blend to that operation because the defining animation
initialized it.


  [Motive]: @ref motive_overview
  [splines]: https://en.wikipedia.org/wiki/Spline_(mathematics)
  [Runge's phenomenon]: https://en.wikipedia.org/wiki/Runge%27s_phenomenon
  [Bezier splines]: https://en.wikipedia.org/wiki/B%C3%A9zier_curve
  [SIMD]: https://en.wikipedia.org/wiki/SIMD
  [dual cubic whitepaper]: https://github.com/google/motive/blob/master/docs/dual_cubics.pdf
  [dual cubic lightning talk]: https://www.youtube.com/watch?v=6ON6fYO4FbY
  [anim_pipeline]: @ref motive_guide_anim_pipeline
  [Euler angles]: https://en.wikipedia.org/wiki/Euler
  [quaternions]: https://en.wikipedia.org/wiki/Quaternion
  [Gimbal lock]: https://en.wikipedia.org/wiki/Gimbal_lock
  [Motivator]: @ref motive_guide_motivators
  [Motivator example]: @ref motive_guide_motivator_example


