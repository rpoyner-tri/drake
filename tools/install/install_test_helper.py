import os
import shutil
import subprocess


def install(installed_subfolders=[]):
    """Install into a temporary directory, and return the path to it.

    Runs install script to install target in the bazel-provided temporary
    directory. The directory does not need to be removed as bazel takes care of
    that. If `installed_subfolders` are not found in installation directory, a
    string containing an error message is returned.
    """
    assert "TEST_TMPDIR" in os.environ, (
        "This may only be run from within `bazel test`")
    # Install target and its dependencies in scratch space.
    installation_folder = os.path.abspath(os.environ['TEST_TMPDIR'])
    subprocess.check_call(["install", installation_folder])
    content_install_folder = os.listdir(installation_folder)
    # Check that all expected folders are installed
    for f in installed_subfolders:
        if f not in content_install_folder:
            return str(f) + " not found in " + str(content_install_folder)
    return installation_folder
