load("//tools/workspace:github.bzl", "github_archive")

def nanobind_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        # local_repository_override = "/home/rico/third/nanobind",
        repository = "wjakob/nanobind",
        commit = "v2.9.2",
        sha256 = "8ce3667dce3e64fc06bfb9b778b6f48731482362fb89a43da156632266cd5a90",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/ndarray_extra_import.patch",
        ],
        mirrors = mirrors,
    )
