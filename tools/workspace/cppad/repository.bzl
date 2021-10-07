# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def cppad_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "coin-or/CppAD",  # License: EPL-2.0
        commit = "20210000.8",
        sha256 = "465a462329fb62110c4799577178d1f28d8c0083b385b7ea08ac82bb98873844",  # noqa
        build_file = "@drake//tools/workspace/cppad:package.BUILD.bazel",
        mirrors = mirrors,
    )
