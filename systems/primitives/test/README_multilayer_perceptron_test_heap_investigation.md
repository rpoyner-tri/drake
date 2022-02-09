# Investigating heap use in MultilayerPerceptron

Unit test changes in this patch apply LimitMalloc to API calls of
interest. Using this modified program, we can investigate the heap usage and
(to some extent) its causes.

## Use `dump_limit_malloc_stacks`

This script will automate stack traces from all disallowed mallocs via
gdb. Build and run like so:

    $ bazel build -c dbg //systems/primitives:multilayer_perceptron_test
    $ tools/dynamic_analysis/dump_limit_malloc_stacks \
      bazel-bin/systems/primitives/multilayer_perceptron_test \
      |& tee stacks.out

## Winnow the data

Here's a completely gross `sed`-powered command line that will turn stacks.out
into something that can be loaded in Emacs and treated like build errors via
`M-x compilation-mode`.

    $ egrep 'Catchpoint| at ' stacks.out \
      | sed 's#^\(C.*\) at .*$#\1#g;s#^[^C].* at \(.*\)$#\1: error:#g;s#external/#bazel-drake/external/##g' \
      |egrep -v 'gtest|linux/' >stacks.err

We can summarize `stacks.err` even further, just looking at which lines of our
module are allocating, and counting the frequencies:

    $ grep perceptron.cc stacks.err |sort|uniq -c |sort -n
