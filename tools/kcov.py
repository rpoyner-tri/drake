#!/usr/bin/env python

import imp
import os
import subprocess
import sys

ME = os.path.realpath(__file__)


def is_lint_test():
    return "_cpplint" in " ".join(sys.argv[1:])


def is_wrapped_python(filename):
    with file(filename, 'r') as afile:
        text = afile.read()
        return text.startswith('#!/usr/bin/env python') and 'Main()' in text


def instrument_wrapped_python():
    old_execv = os.execv

    def my_execv(firstarg, arglist):
        os.execv = old_execv
        assert(firstarg == arglist[0])
        print "computed by bazel", arglist
        normal_kcov_run([arglist[1]] + arglist[3:])

    os.execv = my_execv
    wrapper = imp.load_source('wrapper', sys.argv[1])
    wrapper.Main()


def normal_kcov_run(args):
    print "normal kcov run args", args
    workspace = os.path.dirname(os.path.dirname(ME))
    arglist = [
        "kcov",
        "--include-path=%s" % workspace,
        "--exclude-pattern=thirdParty,externals",
        "%s/bazel-kcov" % workspace,
        ] + args
    print "kcov command", arglist
    sys.exit(subprocess.call(arglist))


def main():
    print "main", sys.argv
    if is_lint_test():
        os.execv(sys.argv[1], sys.argv[1:])
    elif is_wrapped_python(sys.argv[1]):
        instrument_wrapped_python()
    else:
        normal_kcov_run(sys.argv[1:])

if __name__ == "__main__":
    main()
