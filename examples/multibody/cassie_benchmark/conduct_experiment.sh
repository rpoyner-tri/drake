#!/bin/bash
# Execute the complete set of steps for a well-controlled benchmark experiment.

set -e -u -o pipefail

die () {
    echo $"$@"
    exit 1
}

is_default_ubuntu () {
    [[ $(uname) = "Linux" && $(echo  $(lsb_release -irs)) = "Ubuntu 18.04" ]]
}

is_default_compiler () {
    # Use deep bash magic to assert variables are unset.
    [[ -z ${CC+x} && -z ${CXX+x} ]]
}

say () {
    echo
    echo === "$@" ===
    echo
}



say Validate input.
[[ "$#" -ge 1 ]] || die "missing argument: destination directory"
DESTINATION="$1"

say Validate environment.
is_default_ubuntu || die "experiments only supported on default platform"
is_default_compiler || die "experiments only supported with default compiler"

ME=$(readlink -f $0)
HERE=$(dirname $ME)
WORKSPACE=$(readlink -f $HERE/../../..)
say Workspace is "$WORKSPACE".


say Validate sudo access, to avoid later interruptions.
sudo -v

say Install tools for CPU speed control.
sudo apt install linux-tools-$(uname -r)

say Build code.
bazel build //examples/multibody/cassie_benchmark:record_results

say Wait for lingering activity to subside.
sync
sleep 10

say Control CPU speed variation.
sudo cpupower frequency-set --governor performance
sudo sh -c 'echo 1 > /sys/devices/system/cpu/intel_pstate/no_turbo'

say Run the experiment.
bazel run //examples/multibody/cassie_benchmark:record_results

say Undo temporary CPU configuration.
sudo cpupower frequency-set --governor powersave
sudo sh -c 'echo 0 > /sys/devices/system/cpu/intel_pstate/no_turbo'

say Save data.
"$WORKSPACE"/examples/multibody/cassie_benchmark/copy_results_to "$DESTINATION"


