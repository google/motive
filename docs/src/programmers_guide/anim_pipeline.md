Animation Pipeline   {#motive_guide_anim_pipeline}
==================

Motive comes with an animation pipeline that converts [FBX files]
to `RigAnims`. The `RigAnims` are stored as [FlatBuffer] files with the
`.fplanim` extension.

    Usage: anim_pipeline [-v|-d|-i] [-o OUTPUT_FILE]
               [-s SCALE_TOLERANCE] [-r ROTATE_TOLERANCE]
               [-t TRANSLATE_TOLERANCE] [-a DERIVATIVE_TOLERANCE]
               [--repeat|--norepeat] FBX_FILE

    Pipeline to convert FBX animations into FlatBuffer animations.
    Outputs a .fplanim file with the same base name as FBX_FILE.

    Options:
      -v, --verbose         output all informative messages
      -d, --details         output important informative messages
      -i, --info            output more than details, less than verbose.
      -o, --out OUTPUT_FILE file to write .fplanim file to;
                            can be an absolute or relative path;
                            when unspecified, uses base FBX name
                            + .fplanim
      -s, --scale SCALE_TOLERANCE
                            max deviation of output scale curves
                            from input scale curves, unitless
      -r, --rotate ROTATE_TOLERANCE
                            max deviation of output rotate curves
                            from intput rotate curves, in degrees
      -t, --translate TRANSLATE_TOLERANCE
                            max deviation of output translate curves
                            from input translate curves, in scene's
                            units of distance
      -a, --angle DERIVATIVE_TOLERANCE
                            max deviation of curve derivatives,
                            considered as an angle in the x/y plane
                            (e.g. derivative 1 ==> 45 degrees),
                            in degrees.
      --repeat, --norepeat  mark the animation as repeating or not
                            repeating. A repeating animation cycles
                            over and over. If neither option is
                            specified, the animation is marked as
                            repeating when it starts and ends
                            with the same pose and derivatives.

# Bone Assignment

The `anim_pipeline` traverses the FBX's scene graph in [depth-first order].
A node is output if it is the ancestor of a mesh node--that is, if it has
vertex data somewhere under it. Every node that is output is assigned a bone.
Each bone is animated with its own `MatrixAnim`.
Together, the series of `MatrixAnims` creates a `RigAnim`.

Each `MatrixAnim` is composed of the minimum set of matrix operations that
will generate the animation. For example, if a node is animated by rotating
about its x-axis, then the `MatrixAnim` will contain a single operation for
`kRotateAboutX`. This creates a much smaller data set than outputting all
scale, rotation, and translation values for every bone.

The bone heirarchy output by `anim_pipeline` matches the bone hierarchy
output by `mesh_pipeline` with the `-h` (hierarchy).


  [FBX files]: https://en.wikipedia.org/wiki/Spline_(mathematics)
  [FlatBuffer]: http://google.github.io/flatbuffers/
  [depth-first order]: https://en.wikipedia.org/wiki/Depth-first_search


