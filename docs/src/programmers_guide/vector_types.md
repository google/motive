Using Your Own Vector Types    {#motive_guide_vector_types}
===========================

# Overview  {#motive_guide_vector_types_overview}
Internally, Motive uses `mathfu` types. For example, all the data in a
`MotiveProcessor` is transmitted using `mathfu::vec3`, `mathfu::mat4`, etc.

The external API can be wrapped in a conversion layer. You can supply your own
vector types, together with conversion functions to and from `mathfu` types.

# Example  {#motive_guide_vector_types_example}
For example, suppose you have these vector types in your code base:

\snippet src/samples/own_vector_types.cpp Own Vector Types

Then, in your own header file, you can define a conversion class like this:

\snippet src/samples/own_vector_types.cpp Own Vector Converter

And then define a `MatrixMotivator` that uses your types in its external API:

\snippet src/samples/own_vector_types.cpp Own MatrixMotivator

# Strict Aliasing  {#motive_guide_vector_types_strict_aliasing}

Strict aliasing is not a worry in this situation.

Strict aliasing bugs can occur when you cast between types. The C++ compiler is
allowed to rearrange pointer operations (reads and writes) of different types.
Therefore, writing a value, casting a pointer to the value to a different type,
then reading from at pointer can get you into trouble: the read can be put
before the write.

In Motive, for all conversions, the read and write are separated by a virtual
function call. The compiler cannot reorder across a virtual function call
boundary (because they cannot be inlined). Therefore, strict aliasing is
probably not a worry.

So casting a `mathfu::mat4` reference to a reference of your matrix type
shouldn't result in aliasing bugs. You do need to ensure that the alignment
restrictions are the same, however (see below).

Of course, this is only true for the specific situation described here in
Motive. If you reuse your conversion class elsewhere in your code, you may
be setting yourself up for trouble. If you do use casts, be aware that it is
a maintanence hazard.

# Alignment Restrictions  {#motive_guide_vector_types_alignment}

Note that `mathfu` types will be 16-byte aligned, if you're using the SIMD
option of `mathfu`. The SIMD option will make Motive run faster on most
platforms, so it's a good idea to enable it when you can.

If your types are only 4-byte aligned, then you will need to create copies of
them in your converter's `From` calls. These copies cannot be optimized away,
and are therefore pure excess call overhead.

For maximal efficiency, you may want to consider aligning your vector types to
match `mathfu` alignment.


