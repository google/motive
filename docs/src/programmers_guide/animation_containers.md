Animation Containers   {#motive_guide_animation_containers}
====================

# AnimTable

`AnimTable` is a basic container for `RigAnims`. It encapsulates an
array of arrays: the top level array is indexed by an `object` index, and
the sub-array is indexed by an `anim_idx`. Both `object` and `anim` indices
are defined by the calling application (usually with enums).

The same `RigAnim` can be referenced multiple times in an `AnimTable`,
but only one copy of that `RigAnim` will exist.

For simple games, a single `AnimTable` will be enough to hold all the
animations.

Future releases of Motive will have containers with richer selection logic.
For the moment, however, Motive is very much a low-level animation solution;
it provides animation playback and blending, but animation selection is
left to the calling code.

# Deserializing Animation Data

Motive defines [FlatBuffer] schemas for `MatrixAnims`, `RigAnims`, and
`AnimTables`. The schemas are in `motive/schemas`.

`RigAnim` FlatBuffer files have the `.fplanim` extension.
`AnimTables` FlatBuffer files have the `.fplanimtab` extension.

The FlatBuffer files can be deserialized by calling
`MatrixAnimFromFlatBuffers()`, `RigAnimFromFlatBuffers()`, and
`AnimTable::InitFromFlatBuffers()`.


  [FlatBuffer]: http://google.github.io/flatbuffers/
