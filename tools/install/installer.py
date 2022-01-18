"""
Installation script generated from a Bazel `install` target.
"""

# N.B. Please ensure any significant behavior is tested by `install_meta_test`.
# Example execution:
#   $ bazel test //tools/install:py/install_meta_test --test_output=streamed --nocache_test_results  # noqa

# N.B. This is designed to emulate CMake's install mechanism. Do not add
# unnecessary print statements.


import argparse
import collections
import filecmp
import itertools
import os
import re
import shutil
import stat
import sys
from subprocess import check_output, check_call

# Stores subdirectories that have already been created.
subdirs = set()

# Mapping used to (a) check for unique shared library names and (b) provide a
# mapping from library name to paths for RPath fixes (where (a) is essential).
# Structure: Map[ basename (Str) => full_path ]
libraries_to_fix_rpath = {}
# These are binaries (or Python shared libraries) that require RPath fixes (and
# thus depend on `libraries_to_fix_rpath`), but by definition are not depended
# upon by other components, and thus need not be unique.
# Structure: List[ Tuple(basename, full_path) ]
binaries_to_fix_rpath = []
# Files that are not libraries, but may still require fixing.
# Structure: List[Str]
potential_binaries_to_fix_rpath = []
# Used for matching against libraries and extracting useful components.
# N.B. On linux, dynamic libraries may have their version number as a suffix
# (e.g. my_lib.so.x.y.z).
dylib_match = re.compile(r"(.*\.so)(\.\d+)*$|(.*\.dylib)$")


def is_relative_link(filepath):
    """Find if a file is a relative link.

    Bazel paths are assumed to always be absolute. If path is not absolute,
    the file is a link we want to keep.

    If the given `filepath` is not a link, the function returns `None`. If the
    given `filepath` is a link, the result will depend if the link is absolute
    or relative. The function is called recursively. If the result is not a
    link, `None` is returned. If the link is relative, the relative link is
    returned.
    """
    if os.path.islink(filepath):
        link = os.readlink(filepath)
        if not os.path.isabs(link):
            return link
        else:
            return is_relative_link(link)
    else:
        return None


def find_binary_executables():
    """Finds installed files that are binary executables to fix them up later.

    Takes `potential_binaries_to_fix_rpath` as input list, and updates
    `binaries_to_fix_rpath` with executables that need to be fixed up.
    """
    if not potential_binaries_to_fix_rpath:
        return
    # Checking file type with command `file` is the safest way to find
    # executables. Files without an extension are likely to be executables, but
    # it is not always the case.
    file_output = check_output(
        ["file"] + potential_binaries_to_fix_rpath).decode("utf-8")
    # On Linux, executables can be ELF shared objects.
    executable_match = re.compile(
        r"(.*):.*(ELF.*executable|shared object.*|Mach-O.*executable.*)")
    for line in file_output.splitlines():
        re_result = executable_match.match(line)
        if re_result is not None:
            dst_full = re_result.group(1)
            basename = os.path.basename(dst_full)
            binaries_to_fix_rpath.append((basename, dst_full))


def may_be_binary(dst_full):
    # Try to minimize the amount of work that `find_binary_executables`
    # must do.
    extensions = [".h", ".py", ".obj", ".cmake", ".1", ".hpp", ".txt"]
    for extension in extensions:
        if dst_full.endswith(extension):
            return False
    return True


def needs_install(args, src, dst):
    # Get canonical destination.
    dst_full = os.path.join(args.prefix, dst)

    # Check if destination exists.
    if not os.path.exists(dst_full):
        # Destination doesn't exist -> installation needed.
        return True

    # Check if files are different.
    if filecmp.cmp(src, dst_full, shallow=False):
        # Files are the same -> no installation needed.
        return False

    # File needs to be installed.
    return True


def copy_or_link(src, dst):
    """Copy file if it is not a relative link or recreate the symlink in `dst`.

    Copy the input file to the destination if it is not a relative link. If the
    file is a relative link, create a similar link in the destination folder.
    """
    relative_link = is_relative_link(src)
    if relative_link:
        os.symlink(relative_link, dst)
    else:
        shutil.copy2(src, dst)


def do_install(args, src, dst):
    global subdirs

    # In list-only mode, just display the filename, don't do any real work.
    if args.list:
        print(dst)
        return

    # Ensure destination subdirectory exists, creating it if necessary.
    subdir = os.path.dirname(dst)
    if subdir not in subdirs:
        subdir_full = os.path.join(args.prefix, subdir)
        if not os.path.exists(subdir_full):
            os.makedirs(subdir_full)
        subdirs.add(subdir)

    dst_full = os.path.join(args.prefix, dst)
    # Install file, if not up to date.
    if needs_install(args, src, dst):
        print("-- Installing: {}".format(dst_full))
        if os.path.exists(dst_full):
            os.remove(dst_full)
        copy_or_link(src, dst_full)
    else:
        # TODO(eric.cousineau): Unclear how RPath-patched file can be deemed
        # "up-to-date" by comparison?
        print("-- Up-to-date: {}".format(dst_full))
        # No need to check patching.
        return
    basename = os.path.basename(dst)
    if re.match(dylib_match, basename):  # It is a library.
        if dst.startswith("lib/python") and not basename.startswith("lib"):
            # Assume this is a Python C extension.
            binaries_to_fix_rpath.append((basename, dst_full))
        else:
            # Check that dependency is only referenced once
            # in the library dictionary. If it is referenced multiple times,
            # we do not know which one to use, and fail fast.
            if basename in libraries_to_fix_rpath:
                sys.stderr.write(
                    "Multiple installation rules found for {}."
                    .format(basename))
                sys.exit(1)
            libraries_to_fix_rpath[basename] = dst_full
    elif may_be_binary(dst_full):  # May be an executable.
        potential_binaries_to_fix_rpath.append(dst_full)


def fix_rpaths_and_strip():
    # Add binary executables to list of files to be fixed up:
    find_binary_executables()
    # Only fix files that are installed now.
    fix_items = itertools.chain(
        libraries_to_fix_rpath.items(), binaries_to_fix_rpath)
    for basename, dst_full in fix_items:
        if os.path.islink(dst_full):
            # Skip files that are links. However, they need to be in the
            # dictionary to fixup other library and executable paths.
            continue
        # Enable write permissions to allow modification.
        os.chmod(dst_full, stat.S_IRUSR | stat.S_IWUSR | stat.S_IXUSR
                 | stat.S_IRGRP | stat.S_IXGRP | stat.S_IROTH | stat.S_IXOTH)
        if sys.platform == "darwin":
            # From the manual for BSD `strip`: for dynamic shared libraries,
            # the maximum level of stripping is usually -x (to remove all
            # non-global symbols).
            if strip:
                check_call([strip_tool, '-x', dst_full])
            macos_fix_rpaths(basename, dst_full)
        else:
            # Strip before running `patchelf`. Trying to strip after patching
            # the files is likely going to create the following error:
            # 'Not enough room for program headers, try linking with -N'
            if strip:
                check_call([strip_tool, dst_full])
            linux_fix_rpaths(dst_full)


def macos_fix_rpaths(basename, dst_full):
    # Update file (library, executable) ID (remove relative path).
    check_call(
        [install_name_tool, "-id", "@rpath/" + basename, dst_full]
        )
    # Check if file dependencies are specified with relative paths.
    file_output = check_output(["otool", "-L", dst_full]).decode("utf-8")
    for line in file_output.splitlines():
        # keep only file path, remove version information.
        relative_path = line.split(' (')[0].strip()
        # If path is relative, it needs to be replaced by absolute path.
        if "@loader_path" not in relative_path:
            continue
        dep_basename = os.path.basename(relative_path)
        # Look for the absolute path in the dictionary of fixup files to
        # find library paths.
        if dep_basename not in libraries_to_fix_rpath:
            continue
        lib_dirname = os.path.dirname(dst_full)
        diff_path = os.path.relpath(libraries_to_fix_rpath[dep_basename],
                                    lib_dirname)
        check_call(
            [install_name_tool,
             "-change", relative_path,
             os.path.join('@loader_path', diff_path),
             dst_full]
            )
    # Remove RPATH values that contain @loader_path. These are from the build
    # tree and are irrelevant in the install tree. RPATH is not necessary as
    # relative or absolute path to each library is already known.
    file_output = check_output(["otool", "-l", dst_full]).decode("utf-8")
    for line in file_output.splitlines():
        split_line = line.strip().split(' ')
        if len(split_line) >= 2 \
                and split_line[0] == 'path' \
                and split_line[1].startswith('@loader_path'):
            check_call(
                [install_name_tool, "-delete_rpath", split_line[1], dst_full]
            )


def linux_fix_rpaths(dst_full):
    # A conservative subset of the ld.so search path. These paths are added
    # to /etc/ld.so.conf by default or after the prerequisites install script
    # has been run. Query on a given system using `ldconfig -v`.
    ld_so_search_paths = [
        '/lib',
        '/lib/libblas',
        '/lib/liblapack',
        '/lib/x86_64-linux-gnu',
        '/lib32',
        '/libx32',
        '/usr/lib',
        '/usr/lib/x86_64-linux-gnu',
        '/usr/lib/x86_64-linux-gnu/libfakeroot',
        '/usr/lib/x86_64-linux-gnu/mesa-egl',
        '/usr/lib/x86_64-linux-gnu/mesa',
        '/usr/lib/x86_64-linux-gnu/pulseaudio',
        '/usr/lib32',
        '/usr/libx32',
        '/usr/local/lib',
    ]
    file_output = check_output(["ldd", dst_full]).decode("utf-8")
    rpath = []
    for line in file_output.splitlines():
        ldd_result = line.strip().split(' => ')
        if len(ldd_result) < 2:
            continue
        # Library in install prefix.
        if ldd_result[1] == 'not found' or ldd_result[1].startswith(prefix):
            re_result = re.match(dylib_match, ldd_result[0])
            # Look for the absolute path in the dictionary of libraries using
            # the library name without its possible version number.
            soname, _, _ = re_result.groups()
            if soname not in libraries_to_fix_rpath:
                continue
            lib_dirname = os.path.dirname(dst_full)
            diff_path = os.path.dirname(
                os.path.relpath(libraries_to_fix_rpath[soname], lib_dirname)
            )
            rpath.append('$ORIGIN' + '/' + diff_path)
        # System library not in ld.so search path.
        else:
            # Remove (hexadecimal) address from output leaving (at most) the
            # path to the library.
            ldd_regex = r"(.*\.so(?:\.\d+)*) \(0x[0-9a-f]+\)$"
            re_result = re.match(ldd_regex, ldd_result[1])
            if re_result:
                lib_dirname = os.path.dirname(
                    os.path.realpath(re_result.group(1))
                )
                if lib_dirname not in ld_so_search_paths:
                    rpath.append(lib_dirname + '/')

    # The above may have duplicated some items into the list.  Uniquify it
    # here, preserving order.  Note that we do not just use a set() above,
    # since order matters.
    rpath = collections.OrderedDict.fromkeys(rpath).keys()

    # Replace build tree RPATH with computed install tree RPATH. Build tree
    # RPATH are automatically removed by this call. RPATH will contain the
    # necessary absolute and relative paths to find the libraries that are
    # needed. RPATH will typically be set to `$ORIGIN` or `$ORIGIN/../../..`,
    # possibly concatenated with directories under /opt.
    str_rpath = ":".join(x for x in rpath)
    check_output(
        ["patchelf",
         "--force-rpath",  # We need to override LD_LIBRARY_PATH.
         "--set-rpath", str_rpath,
         dst_full]
    )


def do_create_java_launcher(args, filename, classpath, jvm_flags, main_class):
    # In list-only mode, just display the filename, don't do any real work.
    if args.list:
        print(filename)
        return

    filename = os.path.join(args.prefix, filename)
    print("-- Generating: {}".format(filename))

    # Make sure install directory exists.
    filepath = os.path.dirname(filename)
    if not os.path.exists(filepath):
        os.makedirs(filepath)
    # Converting classpath to string
    strclasspath = '"{}"'.format('" "'.join(classpath))
    # Write launcher.
    if os.path.exists(filename):
        os.chmod(filename, stat.S_IWUSR)
    with open(filename, 'w') as launcher_file:
        content = """#!/bin/bash
# autogenerated - do not edit.
set -euo pipefail

# This file is installed to <prefix>/bin.
readonly prefix=$(python3 -c 'import os;\
    print(os.path.realpath(os.path.join(\"'\"$0\"'\", os.pardir, os.pardir)))')

for jar_file in {}; do
  if [ -f "$jar_file" ]; then
    export CLASSPATH="${{CLASSPATH:+$CLASSPATH:}}$jar_file"
  fi
done

java {} {} "$@"
""".format(strclasspath, jvm_flags, main_class)
        launcher_file.write(content)
    os.chmod(filename, stat.S_IRUSR | stat.S_IRGRP | stat.S_IROTH
             | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)


def execute_install_actions(args):
    # TODO(jwnimmer-tri) Executing arbitrary Python code from the actions file
    # is an absurd implementation choice that we've inherited from the original
    # installer scripts.  We should rework the install.bzl <=> installer.py
    # specification format to use something other than open-ended Python code.
    with open(args.actions, "r", encoding="utf-8") as f:
        actions = f.read()
    def install(*pos_args):
        do_install(args, *pos_args)
    def create_java_launcher(*pos_args):
        do_create_java_launcher(args, *pos_args)
    myglobals = dict(
        # Allowed install actions.
        install=install,
        create_java_launcher=create_java_launcher,

        # Reduce attack surface (somewhat).
        __builtins__={},
    )
    exec(actions, myglobals, {})


def parse_command_line(argv):
    # Set up options.
    parser = argparse.ArgumentParser()
    parser.add_argument('prefix', type=str, help='Install prefix')
    parser.add_argument(
        '--actions', type=str, required=True,
        help='file path to installer actions')
    parser.add_argument(
        '--color', action='store_true', default=False,
        help='colorize the output')
    parser.add_argument(
        '--install_name_tool', type=str, default='install_name_tool',
        help='install_name_tool program')
    parser.add_argument(
        '--list', action='store_true', default=False,
        help='print the list of installed files; do not install anything')
    parser.add_argument(
        '--no_strip', dest='strip', action='store_false', default=True,
        help='do not strip symbols (for debugging)')
    parser.add_argument(
        '--strip_tool', type=str, default='strip',
        help='strip program')
    parser.add_argument(
        '--pre_clean', action='store_true', default=False,
        help='ensure clean install by removing `prefix` dir if it exists '
             'before installing')
    args = parser.parse_args(argv)

    # Transform install prefix if DESTDIR is set.
    # https://www.gnu.org/prep/standards/html_node/DESTDIR.html
    destdir = os.environ.get('DESTDIR')
    if destdir:
        args.prefix = destdir + args.prefix

    # Because Bazel executes us in a strange working directory and not the
    # working directory of the user's shell, enforce that the install
    # location is an absolute path so that the user is not surprised.
    if not os.path.isabs(args.prefix):
        parser.error(
            "Install prefix must be an absolute path (got '{}')\n".format(
                args.prefix))

    return args


def main():
    args = parse_command_line(sys.argv[1:])
    if args.color:
        ansi_color_escape = "\x1b[36m"
        ansi_reset_escape = "\x1b[0m"
    else:
        ansi_color_escape = ""
        ansi_reset_escape = ""

    if args.pre_clean:
        if os.path.isdir(args.prefix):
            print(f"Remove previous directory: {args.prefix}")
            shutil.rmtree(args.prefix)

    if args.strip:
        # Match the output of the CMake install/strip target
        # (https://git.io/fpdzK).
        print("{}Installing the project stripped...{}".format(
            ansi_color_escape, ansi_reset_escape))
    else:
        # Match the output of the CMake install target (https://git.io/fpdzo).
        print("{}Install the project...{}".format(
            ansi_color_escape, ansi_reset_escape))

    execute_install_actions(args)
    # Libraries paths may need to be updated in libraries and executables.
    fix_rpaths_and_strip()


if __name__ == "__main__":
    main()
