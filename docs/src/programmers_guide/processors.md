MotiveProcessors   {#motive_guide_processors}
================

# Creating Your Own Animation Algorithms

An animation algorithm is a formula that describes the motion of a variable.
Each animation algorithm is processed by a `MotiveProcessor`.

A `MotiveProcessor` holds the data for *every* `Motivator` that uses its
algorithm. Once per frame, `MotiveProcessor::AdvanceFrame` is called and
*all* `Motivators` are updated, in bulk. This bulk storage and processing allows
us to optimize with SIMD and multiple threads, which is important for
scalability.

You can add your own custom animation algorithms by deriving classes from
one of the `MotiveProcessor` subclasses. To create a class that animates a
one-dimensional variable, derive from `MotiveProcessor1f`. For a 4x4 matrix
variable, derive from `MotiveProcessorMatrix4f`.

# Example  {#motive_guide_processor_example}

Each `MotiveProcessor` subclass has its own interface that must be instantiated
by your derivation. Additionally, there are macros MOTIVE_INTERFACE() and
MOTIVE_INSTANCE() that you must add to your code.

For example, here is a simple one-dimensional algorithm that moves linearly
from the current value to the target value, in the target time.

Put your initialization code in a header file:

\snippet src/samples/linear_processor.cpp Own Processor LinearInit

The remaining code can be in a .cpp file. There is no need to expose it, since
it gets registered with the `MotiveEngine`, and all calls to it come through
`MotiveEngine`.

\snippet src/samples/linear_processor.cpp Own Processor LinearMotiveProcessor

The MOTIVE_INSTANCE() macro sets up the LinearMotiveProcessor to be registered
with the `MotiveEngine`, but it does not actually register it. The registration
must happen in the main program.

\snippet src/samples/linear_processor.cpp Own Processor Register

The `Register()` call is under `LinearInit` since it is generally the only
thing exposed in a header file.

Now that your processor is registered, you can instantiate a motivator like so,

\snippet src/samples/linear_processor.cpp Own Processor Create Instance

Your processor gets updated with all the other processors, during
`MotiveEngine::AdvanceFrame()`.

\snippet src/samples/linear_processor.cpp Own Processor Advance Simulation

The final output of our example is below. Notice that the variable animates
linearly from 10 down to -5.

    y = 10.000000
    |
    **
    | ***
    |    **
    |      ***
    |         ***
    |            **
    |              ***
    |                 ***
    |                    ***
    |                       **
    |                         ***
    |                            ***
    |                               ***
    |                                  **
    |                                    ****
    |                                       ***
    |                                          **
    |                                            ***
    |                                               ***
    |--------------------------------------------------***---------------------------
    |                                                     **
    |                                                       ***
    |                                                          ***
    |                                                             ***
    |                                                                **
    |                                                                  ***
    |                                                                     ***
    |                                                                        **
    |                                                                          ***
    |                                                                             **
    y = -5.000000

If you'd like to experiment more with this program, it is compiled for you
from `src/samples/linear_processor.cpp`.
