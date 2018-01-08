# Default command. Do not run linting tests, so that the downstream test
# packages do not need to consume `drake`.
# *sigh*... Needs quotes.
_CMD_DEFAULT = "'bazel test //...'"

def workspace_test(
        name,
        workspace,
        cmd = _CMD_DEFAULT,
        data = []):
    """Provides a unittest given writeable access to a copy of a given workspace
    contained in the current project. """
    # Unfortunately, using a `glob(...)` does not play with existing Bazel
    # symlinks (you can't even ignore them, at least in Bazel 0.6.1).
    # For now, just pass in whatever filegroups are needed, and copy them
    # to enable as many unittests as possible (e.g. workflows).
    args = [cmd, "$(location {})".format(workspace)]
    for datum in data:
        args.append("$(locations {})".format(datum))
    native.sh_test(
        name = name,
        srcs = ["workspace_test.sh"],
        args = args,
        data = [workspace] + data,
    )
