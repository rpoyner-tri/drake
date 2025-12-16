load("//tools/workspace:github.bzl", "github_archive")

def nanobind_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        # local_repository_override = "/home/rico/third/nanobind",
        repository = "wjakob/nanobind",
        commit = "v2.10.2",
        sha256 = "5bb7f866f6c9c64405308b69de7e7681d8f779323e345bd71a00199c1eaec073",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/ndarray_extra_import.patch",
        ],
        mirrors = mirrors,
    )
