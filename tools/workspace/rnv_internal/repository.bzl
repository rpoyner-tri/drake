load("//tools/workspace:github.bzl", "github_archive")

def rnv_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "hartwork/rnv",
        commit = "e2435bfd9e67a1e8b3a34bae6919ee572465d435",
        sha256 = "",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
