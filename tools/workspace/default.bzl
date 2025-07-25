load("//tools/workspace:alias.bzl", "alias_repository")
load("//tools/workspace:mirrors.bzl", "DEFAULT_MIRRORS")
load("//tools/workspace/abseil_cpp_internal:repository.bzl", "abseil_cpp_internal_repository")  # noqa
load("//tools/workspace/bazel_skylib:repository.bzl", "bazel_skylib_repository")  # noqa
load("//tools/workspace/bazelisk:repository.bzl", "bazelisk_repository")
load("//tools/workspace/blas:repository.bzl", "blas_repository")
load("//tools/workspace/build_bazel_apple_support:repository.bzl", "build_bazel_apple_support_repository")  # noqa
load("//tools/workspace/buildifier:repository.bzl", "buildifier_repository")
load("//tools/workspace/ccd_internal:repository.bzl", "ccd_internal_repository")  # noqa
load("//tools/workspace/clang_cindex_python3_internal:repository.bzl", "clang_cindex_python3_internal_repository")  # noqa
load("//tools/workspace/clarabel_cpp_internal:repository.bzl", "clarabel_cpp_internal_repository")  # noqa
load("//tools/workspace/clp_internal:repository.bzl", "clp_internal_repository")  # noqa
load("//tools/workspace/coinutils_internal:repository.bzl", "coinutils_internal_repository")  # noqa
load("//tools/workspace/com_jidesoft_jide_oss:repository.bzl", "com_jidesoft_jide_oss_repository")  # noqa
load("//tools/workspace/common_robotics_utilities_internal:repository.bzl", "common_robotics_utilities_internal_repository")  # noqa
load("//tools/workspace/commons_io:repository.bzl", "commons_io_repository")
load("//tools/workspace/crate_universe:repository.bzl", "crate_universe_repositories")  # noqa
load("//tools/workspace/csdp_internal:repository.bzl", "csdp_internal_repository")  # noqa
load("//tools/workspace/curl_internal:repository.bzl", "curl_internal_repository")  # noqa
load("//tools/workspace/dm_control_internal:repository.bzl", "dm_control_internal_repository")  # noqa
load("//tools/workspace/doxygen:repository.bzl", "doxygen_repository")
load("//tools/workspace/drake_models:repository.bzl", "drake_models_repository")  # noqa
load("//tools/workspace/eigen:repository.bzl", "eigen_repository")
load("//tools/workspace/fcl_internal:repository.bzl", "fcl_internal_repository")  # noqa
load("//tools/workspace/fmt:repository.bzl", "fmt_repository")
load("//tools/workspace/gflags:repository.bzl", "gflags_repository")
load("//tools/workspace/gfortran:repository.bzl", "gfortran_repository")
load("//tools/workspace/github3_py_internal:repository.bzl", "github3_py_internal_repository")  # noqa
load("//tools/workspace/gklib_internal:repository.bzl", "gklib_internal_repository")  # noqa
load("//tools/workspace/glib:repository.bzl", "glib_repository")
load("//tools/workspace/googlebenchmark:repository.bzl", "googlebenchmark_repository")  # noqa
load("//tools/workspace/gtest:repository.bzl", "gtest_repository")
load("//tools/workspace/gurobi:repository.bzl", "gurobi_repository")
load("//tools/workspace/gymnasium_py:repository.bzl", "gymnasium_py_repository")  # noqa
load("//tools/workspace/gz_math_internal:repository.bzl", "gz_math_internal_repository")  # noqa
load("//tools/workspace/gz_utils_internal:repository.bzl", "gz_utils_internal_repository")  # noqa
load("//tools/workspace/highway_internal:repository.bzl", "highway_internal_repository")  # noqa
load("//tools/workspace/implib_so_internal:repository.bzl", "implib_so_internal_repository")  # noqa
load("//tools/workspace/ipopt_internal:repository.bzl", "ipopt_internal_repository")  # noqa
load("//tools/workspace/lapack:repository.bzl", "lapack_repository")
load("//tools/workspace/lapack_internal:repository.bzl", "lapack_internal_repository")  # noqa
load("//tools/workspace/lcm:repository.bzl", "lcm_repository")
load("//tools/workspace/libjpeg_turbo_internal:repository.bzl", "libjpeg_turbo_internal_repository")  # noqa
load("//tools/workspace/libpfm:repository.bzl", "libpfm_repository")
load("//tools/workspace/libpng_internal:repository.bzl", "libpng_internal_repository")  # noqa
load("//tools/workspace/libtiff_internal:repository.bzl", "libtiff_internal_repository")  # noqa
load("//tools/workspace/meshcat:repository.bzl", "meshcat_repository")
load("//tools/workspace/metis_internal:repository.bzl", "metis_internal_repository")  # noqa
load("//tools/workspace/mosek:repository.bzl", "mosek_repository")
load("//tools/workspace/mpmath_py_internal:repository.bzl", "mpmath_py_internal_repository")  # noqa
load("//tools/workspace/msgpack_internal:repository.bzl", "msgpack_internal_repository")  # noqa
load("//tools/workspace/mujoco_menagerie_internal:repository.bzl", "mujoco_menagerie_internal_repository")  # noqa
load("//tools/workspace/mypy_extensions_internal:repository.bzl", "mypy_extensions_internal_repository")  # noqa
load("//tools/workspace/mypy_internal:repository.bzl", "mypy_internal_repository")  # noqa
load("//tools/workspace/nanoflann_internal:repository.bzl", "nanoflann_internal_repository")  # noqa
load("//tools/workspace/nasm:repository.bzl", "nasm_repository")
load("//tools/workspace/net_sf_jchart2d:repository.bzl", "net_sf_jchart2d_repository")  # noqa
load("//tools/workspace/nlohmann_internal:repository.bzl", "nlohmann_internal_repository")  # noqa
load("//tools/workspace/nlopt_internal:repository.bzl", "nlopt_internal_repository")  # noqa
load("//tools/workspace/onetbb_internal:repository.bzl", "onetbb_internal_repository")  # noqa
load("//tools/workspace/opencl:repository.bzl", "opencl_repository")
load("//tools/workspace/openusd_internal:repository.bzl", "openusd_internal_repository")  # noqa
load("//tools/workspace/org_apache_xmlgraphics_commons:repository.bzl", "org_apache_xmlgraphics_commons_repository")  # noqa
load("//tools/workspace/osqp_internal:repository.bzl", "osqp_internal_repository")  # noqa
load("//tools/workspace/pathspec_internal:repository.bzl", "pathspec_internal_repository")  # noqa
load("//tools/workspace/picosha2_internal:repository.bzl", "picosha2_internal_repository")  # noqa
load("//tools/workspace/pkgconfig_blas_internal:repository.bzl", "pkgconfig_blas_internal_repository")  # noqa
load("//tools/workspace/pkgconfig_lapack_internal:repository.bzl", "pkgconfig_lapack_internal_repository")  # noqa
load("//tools/workspace/platforms:repository.bzl", "platforms_repository")
load("//tools/workspace/poisson_disk_sampling_internal:repository.bzl", "poisson_disk_sampling_internal_repository")  # noqa
load("//tools/workspace/pybind11:repository.bzl", "pybind11_repository")
load("//tools/workspace/pycodestyle:repository.bzl", "pycodestyle_repository")
load("//tools/workspace/python:repository.bzl", "python_repository")
load("//tools/workspace/qdldl_internal:repository.bzl", "qdldl_internal_repository")  # noqa
load("//tools/workspace/qhull_internal:repository.bzl", "qhull_internal_repository")  # noqa
load("//tools/workspace/ros_xacro_internal:repository.bzl", "ros_xacro_internal_repository")  # noqa
load("//tools/workspace/rules_cc:repository.bzl", "rules_cc_repository")  # noqa
load("//tools/workspace/rules_java:repository.bzl", "rules_java_repository")
load("//tools/workspace/rules_license:repository.bzl", "rules_license_repository")  # noqa
load("//tools/workspace/rules_python:repository.bzl", "rules_python_repository")  # noqa
load("//tools/workspace/rules_rust:repository.bzl", "rules_rust_repository")
load("//tools/workspace/rules_rust_tinyjson:repository.bzl", "rules_rust_tinyjson_repository")  # noqa
load("//tools/workspace/rules_shell:repository.bzl", "rules_shell_repository")
load("//tools/workspace/rust_toolchain:repository.bzl", "register_rust_toolchains", "rust_toolchain_repositories")  # noqa
load("//tools/workspace/scs_internal:repository.bzl", "scs_internal_repository")  # noqa
load("//tools/workspace/sdformat_internal:repository.bzl", "sdformat_internal_repository")  # noqa
load("//tools/workspace/snopt:repository.bzl", "snopt_repository")
load("//tools/workspace/spdlog:repository.bzl", "spdlog_repository")
load("//tools/workspace/spgrid_internal:repository.bzl", "spgrid_module_extension_impl")  # noqa
load("//tools/workspace/spral_internal:repository.bzl", "spral_internal_repository")  # noqa
load("//tools/workspace/stable_baselines3_internal:repository.bzl", "stable_baselines3_internal_repository")  # noqa
load("//tools/workspace/statsjs:repository.bzl", "statsjs_repository")
load("//tools/workspace/stduuid_internal:repository.bzl", "stduuid_internal_repository")  # noqa
load("//tools/workspace/styleguide:repository.bzl", "styleguide_repository")
load("//tools/workspace/suitesparse_internal:repository.bzl", "suitesparse_internal_repository")  # noqa
load("//tools/workspace/sympy_py_internal:repository.bzl", "sympy_py_internal_repository")  # noqa
load("//tools/workspace/tinygltf_internal:repository.bzl", "tinygltf_internal_repository")  # noqa
load("//tools/workspace/tinyobjloader_internal:repository.bzl", "tinyobjloader_internal_repository")  # noqa
load("//tools/workspace/tinyxml2_internal:repository.bzl", "tinyxml2_internal_repository")  # noqa
load("//tools/workspace/tomli_internal:repository.bzl", "tomli_internal_repository")  # noqa
load("//tools/workspace/typing_extensions_internal:repository.bzl", "typing_extensions_internal_repository")  # noqa
load("//tools/workspace/uritemplate_py_internal:repository.bzl", "uritemplate_py_internal_repository")  # noqa
load("//tools/workspace/usockets_internal:repository.bzl", "usockets_internal_repository")  # noqa
load("//tools/workspace/uwebsockets_internal:repository.bzl", "uwebsockets_internal_repository")  # noqa
load("//tools/workspace/voxelized_geometry_tools_internal:repository.bzl", "voxelized_geometry_tools_internal_repository")  # noqa
load("//tools/workspace/vtk_internal:repository.bzl", "vtk_internal_repository")  # noqa
load("//tools/workspace/x11:repository.bzl", "x11_repository")
load("//tools/workspace/xmlrunner_py:repository.bzl", "xmlrunner_py_repository")  # noqa
load("//tools/workspace/yaml_cpp_internal:repository.bzl", "yaml_cpp_internal_repository")  # noqa
load("//tools/workspace/zlib:repository.bzl", "zlib_repository")
load(":workspace_deprecation.bzl", print_workspace_deprecation_warning = "print_warning")  # noqa

# =============================================================================
# For Bazel projects using Drake as a dependency via the WORKSPACE mechanism.
# =============================================================================

def add_default_repositories(
        excludes = [],
        mirrors = DEFAULT_MIRRORS,
        _is_drake_self_call = False):
    """WARNING: Deprecated for removal on or after 2025-09-01.

    Declares workspace repositories for all externals needed by drake (other
    than those built into Bazel, of course). For users, this is intended to be
    loaded and called from a WORKSPACE file. (Drake also calls it internally
    in service of our module extension infrastructure.)

    Args:
        excludes: list of string names of repositories to exclude; this can
          be useful if a WORKSPACE file has already supplied its own external
          of a given name.
    """
    if not _is_drake_self_call:
        print_workspace_deprecation_warning("add_default_repositories")
    if "abseil_cpp_internal" not in excludes:
        abseil_cpp_internal_repository(name = "abseil_cpp_internal", mirrors = mirrors)  # noqa
    if "bazelisk" not in excludes:
        bazelisk_repository(name = "bazelisk", mirrors = mirrors, _is_drake_self_call = True)  # noqa
    if "bazel_skylib" not in excludes:
        bazel_skylib_repository(name = "bazel_skylib", mirrors = mirrors, _is_drake_self_call = True)  # noqa
    if "blas" not in excludes:
        blas_repository(name = "blas")
    if "build_bazel_apple_support" not in excludes:
        build_bazel_apple_support_repository(name = "build_bazel_apple_support", mirrors = mirrors, _is_drake_self_call = True)  # noqa
    if "buildifier" not in excludes:
        buildifier_repository(name = "buildifier", mirrors = mirrors)
    if "ccd_internal" not in excludes:
        ccd_internal_repository(name = "ccd_internal", mirrors = mirrors)
    if "clang_cindex_python3_internal" not in excludes:
        clang_cindex_python3_internal_repository(name = "clang_cindex_python3_internal", mirrors = mirrors)  # noqa
    if "clarabel_cpp_internal" not in excludes:
        clarabel_cpp_internal_repository(name = "clarabel_cpp_internal", mirrors = mirrors)  # noqa
    if "clp_internal" not in excludes:
        clp_internal_repository(name = "clp_internal", mirrors = mirrors)
    if "coinutils_internal" not in excludes:
        coinutils_internal_repository(name = "coinutils_internal", mirrors = mirrors)  # noqa
    if "com_jidesoft_jide_oss" not in excludes:
        com_jidesoft_jide_oss_repository(name = "com_jidesoft_jide_oss", mirrors = mirrors, _is_drake_self_call = True)  # noqa
    if "common_robotics_utilities_internal" not in excludes:
        common_robotics_utilities_internal_repository(name = "common_robotics_utilities_internal", mirrors = mirrors)  # noqa
    if "commons_io" not in excludes:
        commons_io_repository(name = "commons_io", mirrors = mirrors, _is_drake_self_call = True)  # noqa
    if "crate_universe" not in excludes:
        crate_universe_repositories(mirrors = mirrors, excludes = excludes, _is_drake_self_call = True)  # noqa
    if "csdp_internal" not in excludes:
        csdp_internal_repository(name = "csdp_internal", mirrors = mirrors)
    if "curl_internal" not in excludes:
        curl_internal_repository(name = "curl_internal", mirrors = mirrors)
    if "doxygen" not in excludes:
        doxygen_repository(name = "doxygen", mirrors = mirrors, _is_drake_self_call = True)  # noqa
    if "dm_control_internal" not in excludes:
        dm_control_internal_repository(name = "dm_control_internal", mirrors = mirrors)  # noqa
    if "drake_models" not in excludes:
        drake_models_repository(name = "drake_models", mirrors = mirrors)
    if "eigen" not in excludes:
        eigen_repository(name = "eigen")
    if "fcl_internal" not in excludes:
        fcl_internal_repository(name = "fcl_internal", mirrors = mirrors)
    if "fmt" not in excludes:
        fmt_repository(name = "fmt", mirrors = mirrors)
    if "gflags" not in excludes:
        gflags_repository(name = "gflags", mirrors = mirrors)
    if "gfortran" not in excludes:
        gfortran_repository(name = "gfortran", _is_drake_self_call = True)
    if "github3_py_internal" not in excludes:
        github3_py_internal_repository(name = "github3_py_internal", mirrors = mirrors)  # noqa
    if "gklib_internal" not in excludes:
        gklib_internal_repository(name = "gklib_internal", mirrors = mirrors)  # noqa
    if "glib" not in excludes:
        glib_repository(name = "glib")
    if "googlebenchmark" not in excludes:
        googlebenchmark_repository(name = "googlebenchmark", mirrors = mirrors)
    if "gtest" not in excludes:
        gtest_repository(name = "gtest", mirrors = mirrors)
    if "gurobi" not in excludes:
        gurobi_repository(name = "gurobi")
    if "gz_math_internal" not in excludes:
        gz_math_internal_repository(name = "gz_math_internal", mirrors = mirrors)  # noqa
    if "gz_utils_internal" not in excludes:
        gz_utils_internal_repository(name = "gz_utils_internal", mirrors = mirrors)  # noqa
    if "gymnasium_py" not in excludes:
        gymnasium_py_repository(name = "gymnasium_py", mirrors = mirrors, _is_drake_self_call = True)  # noqa
    if "highway_internal" not in excludes:
        highway_internal_repository(name = "highway_internal", mirrors = mirrors)  # noqa
    if "implib_so_internal" not in excludes:
        implib_so_internal_repository(name = "implib_so_internal", mirrors = mirrors)  # noqa
    if "ipopt_internal" not in excludes:
        ipopt_internal_repository(name = "ipopt_internal", mirrors = mirrors)  # noqa
    if "lapack" not in excludes:
        # @lapack is the alias (controlled by //tools/flags) that selects which
        # LAPACK library we'll use when building.
        lapack_repository(name = "lapack")
    if "lapack_internal" not in excludes:
        # @lapack_internal builds BLAS and/or LAPACK from source, but is only
        # conditionally used / referenced, depending on the //tools/flags.
        lapack_internal_repository(name = "lapack_internal", mirrors = mirrors)
    if "lcm" not in excludes:
        lcm_repository(name = "lcm", mirrors = mirrors)
    if "libjpeg_turbo_internal" not in excludes:
        libjpeg_turbo_internal_repository(name = "libjpeg_turbo_internal", mirrors = mirrors)  # noqa
    if "libpfm" not in excludes:
        libpfm_repository(name = "libpfm", _is_drake_self_call = True)
    if "libpng_internal" not in excludes:
        libpng_internal_repository(name = "libpng_internal", mirrors = mirrors)
    if "libtiff_internal" not in excludes:
        libtiff_internal_repository(name = "libtiff_internal", mirrors = mirrors)  # noqa
    if "meshcat" not in excludes:
        meshcat_repository(name = "meshcat", mirrors = mirrors)
    if "metis_internal" not in excludes:
        metis_internal_repository(name = "metis_internal", mirrors = mirrors)
    if "mosek" not in excludes:
        mosek_repository(name = "mosek", mirrors = mirrors)
    if "mpmath_py_internal" not in excludes:
        mpmath_py_internal_repository(name = "mpmath_py_internal", mirrors = mirrors)  # noqa
    if "msgpack_internal" not in excludes:
        msgpack_internal_repository(name = "msgpack_internal", mirrors = mirrors)  # noqa
    if "mujoco_menagerie_internal" not in excludes:
        mujoco_menagerie_internal_repository(name = "mujoco_menagerie_internal", mirrors = mirrors)  # noqa
    if "mypy_extensions_internal" not in excludes:
        mypy_extensions_internal_repository(name = "mypy_extensions_internal", mirrors = mirrors)  # noqa
    if "mypy_internal" not in excludes:
        mypy_internal_repository(name = "mypy_internal", mirrors = mirrors)
    if "nanoflann_internal" not in excludes:
        nanoflann_internal_repository(name = "nanoflann_internal", mirrors = mirrors)  # noqa
    if "nasm" not in excludes:
        nasm_repository(name = "nasm")
    if "net_sf_jchart2d" not in excludes:
        net_sf_jchart2d_repository(name = "net_sf_jchart2d", mirrors = mirrors, _is_drake_self_call = True)  # noqa
    if "nlohmann_internal" not in excludes:
        nlohmann_internal_repository(name = "nlohmann_internal", mirrors = mirrors)  # noqa
    if "nlopt_internal" not in excludes:
        nlopt_internal_repository(name = "nlopt_internal", mirrors = mirrors)
    if "onetbb_internal" not in excludes:
        onetbb_internal_repository(name = "onetbb_internal", mirrors = mirrors)
    if "opencl" not in excludes:
        opencl_repository(name = "opencl")
    if "openusd_internal" not in excludes:
        openusd_internal_repository(name = "openusd_internal", mirrors = mirrors)  # noqa
    if "org_apache_xmlgraphics_commons" not in excludes:
        org_apache_xmlgraphics_commons_repository(name = "org_apache_xmlgraphics_commons", mirrors = mirrors, _is_drake_self_call = True)  # noqa
    if "osqp_internal" not in excludes:
        osqp_internal_repository(name = "osqp_internal", mirrors = mirrors)
    if "pathspec_internal" not in excludes:
        pathspec_internal_repository(name = "pathspec_internal", mirrors = mirrors)  # noqa
    if "picosha2_internal" not in excludes:
        picosha2_internal_repository(name = "picosha2_internal", mirrors = mirrors)  # noqa
    if "pkgconfig_blas_internal" not in excludes:
        pkgconfig_blas_internal_repository(name = "pkgconfig_blas_internal")
    if "pkgconfig_lapack_internal" not in excludes:
        pkgconfig_lapack_internal_repository(name = "pkgconfig_lapack_internal")  # noqa
    if "platforms" not in excludes:
        platforms_repository(name = "platforms", mirrors = mirrors, _is_drake_self_call = True)  # noqa
    if "poisson_disk_sampling_internal" not in excludes:
        poisson_disk_sampling_internal_repository(name = "poisson_disk_sampling_internal", mirrors = mirrors)  # noqa
    if "pybind11" not in excludes:
        pybind11_repository(name = "pybind11", mirrors = mirrors)
    if "pycodestyle" not in excludes:
        pycodestyle_repository(name = "pycodestyle", mirrors = mirrors)
    if "python" not in excludes:
        python_repository(name = "python")
    if "qdldl_internal" not in excludes:
        qdldl_internal_repository(name = "qdldl_internal", mirrors = mirrors)
    if "qhull_internal" not in excludes:
        qhull_internal_repository(name = "qhull_internal", mirrors = mirrors)
    if "ros_xacro_internal" not in excludes:
        ros_xacro_internal_repository(name = "ros_xacro_internal", mirrors = mirrors)  # noqa
    if "rules_cc" not in excludes:
        rules_cc_repository(name = "rules_cc", mirrors = mirrors, _is_drake_self_call = True)  # noqa
    if "rules_java" not in excludes:
        rules_java_repository(name = "rules_java", mirrors = mirrors, _is_drake_self_call = True)  # noqa
    if "rules_license" not in excludes:
        rules_license_repository(name = "rules_license", mirrors = mirrors, _is_drake_self_call = True)  # noqa
    if "rules_python" not in excludes:
        rules_python_repository(name = "rules_python", mirrors = mirrors, _is_drake_self_call = True)  # noqa
    else:
        rules_python_repository(name = "rules_python", _constants_only = True, _is_drake_self_call = True)  # noqa
    if "rules_rust" not in excludes:
        rules_rust_repository(name = "rules_rust", mirrors = mirrors, _is_drake_self_call = True)  # noqa
    if "rules_rust_tinyjson" not in excludes:
        rules_rust_tinyjson_repository(name = "rules_rust_tinyjson", mirrors = mirrors, _is_drake_self_call = True)  # noqa
    if "rules_shell" not in excludes:
        rules_shell_repository(name = "rules_shell", mirrors = mirrors, _is_drake_self_call = True)  # noqa
    if "rust_toolchain" not in excludes:
        rust_toolchain_repositories(mirrors = mirrors, excludes = excludes, _is_drake_self_call = True)  # noqa
    if "scs_internal" not in excludes:
        scs_internal_repository(name = "scs_internal", mirrors = mirrors)
    if "sdformat_internal" not in excludes:
        sdformat_internal_repository(name = "sdformat_internal", mirrors = mirrors)  # noqa
    if "snopt" not in excludes:
        snopt_repository(name = "snopt")
    if "spdlog" not in excludes:
        spdlog_repository(name = "spdlog", mirrors = mirrors)
    if "spral_internal" not in excludes:
        spral_internal_repository(name = "spral_internal", mirrors = mirrors)
    if "stable_baselines3_internal" not in excludes:
        stable_baselines3_internal_repository(name = "stable_baselines3_internal", mirrors = mirrors)  # noqa
    if "statsjs" not in excludes:
        statsjs_repository(name = "statsjs", mirrors = mirrors, _is_drake_self_call = True)  # noqa
    if "stduuid_internal" not in excludes:
        stduuid_internal_repository(name = "stduuid_internal", mirrors = mirrors)  # noqa
    if "styleguide" not in excludes:
        styleguide_repository(name = "styleguide", mirrors = mirrors)
    if "suitesparse_internal" not in excludes:
        suitesparse_internal_repository(name = "suitesparse_internal", mirrors = mirrors)  # noqa
    if "sympy_py_internal" not in excludes:
        sympy_py_internal_repository(name = "sympy_py_internal", mirrors = mirrors)  # noqa
    if "tinygltf_internal" not in excludes:
        tinygltf_internal_repository(name = "tinygltf_internal", mirrors = mirrors)  # noqa
    if "tinyobjloader_internal" not in excludes:
        tinyobjloader_internal_repository(name = "tinyobjloader_internal", mirrors = mirrors)  # noqa
    if "tinyxml2_internal" not in excludes:
        tinyxml2_internal_repository(name = "tinyxml2_internal", mirrors = mirrors)  # noqa
    if "tomli_internal" not in excludes:
        tomli_internal_repository(name = "tomli_internal", mirrors = mirrors)
    if "typing_extensions_internal" not in excludes:
        typing_extensions_internal_repository(name = "typing_extensions_internal", mirrors = mirrors)  # noqa
    if "uritemplate_py_internal" not in excludes:
        uritemplate_py_internal_repository(name = "uritemplate_py_internal", mirrors = mirrors)  # noqa
    if "usockets_internal" not in excludes:
        usockets_internal_repository(name = "usockets_internal", mirrors = mirrors)  # noqa
    if "uwebsockets_internal" not in excludes:
        uwebsockets_internal_repository(name = "uwebsockets_internal", mirrors = mirrors)  # noqa
    if "voxelized_geometry_tools_internal" not in excludes:
        voxelized_geometry_tools_internal_repository(name = "voxelized_geometry_tools_internal", mirrors = mirrors)  # noqa
    if "vtk_internal" not in excludes:
        vtk_internal_repository(name = "vtk_internal", mirrors = mirrors)
    if "x11" not in excludes:
        x11_repository(name = "x11")
    if "xmlrunner_py" not in excludes:
        xmlrunner_py_repository(name = "xmlrunner_py", mirrors = mirrors, _is_drake_self_call = True)  # noqa
    if "yaml_cpp_internal" not in excludes:
        yaml_cpp_internal_repository(name = "yaml_cpp_internal", mirrors = mirrors)  # noqa
    if "zlib" not in excludes:
        zlib_repository(name = "zlib")

def add_default_toolchains(
        excludes = [],
        _is_drake_self_call = False):
    """WARNING: Deprecated for removal on or after 2025-09-01.

    Registers toolchains for each language (e.g., "py") not explicitly
    excluded and/or not using an automatically generated toolchain.

    Args:
        excludes: List of languages for which a toolchain should not be
            registered.
    """
    if not _is_drake_self_call:
        print_workspace_deprecation_warning("add_default_toolchains")
    if "py" not in excludes:
        native.register_toolchains("@python//:all")
    if "rust" not in excludes:
        register_rust_toolchains(_is_drake_self_call = True)

def add_default_workspace(
        repository_excludes = [],
        toolchain_excludes = [],
        mirrors = DEFAULT_MIRRORS,
        _is_drake_self_call = False):
    """WARNING: Deprecated for removal on or after 2025-09-01.

    Declares repositories in this WORKSPACE for each dependency of @drake
    (e.g., "eigen") that is not explicitly excluded, and register toolchains
    for each language (e.g., "py") not explicitly excluded and/or not using an
    automatically generated toolchain.

    Args:
        repository_excludes: List of repositories that should not be declared
            in this WORKSPACE.
        toolchain_excludes: List of languages for which a toolchain should not
            be registered.
        mirrors: Dictionary of mirrors from which to download repository files.
            See mirrors.bzl file in this directory for the file format and
            default values.
    """
    if not _is_drake_self_call:
        print_workspace_deprecation_warning("add_default_workspace")
    add_default_repositories(
        excludes = repository_excludes,
        mirrors = mirrors,
        _is_drake_self_call = True,
    )
    add_default_toolchains(
        excludes = toolchain_excludes,
        _is_drake_self_call = True,
    )

# =============================================================================
# For Bazel projects using Drake as a dependency via the MODULE mechanism.
# =============================================================================

# This is the list of modules that our MODULE.bazel already incorporates.
# It is cross-checked by the workspace_bzlmod_sync_test.py test.
REPOS_ALREADY_PROVIDED_BY_BAZEL_MODULES = [
    "build_bazel_apple_support",
    "bazel_features",
    "bazel_skylib",
    "eigen",
    "fmt",
    "platforms",
    "rust_toolchain",
    "rules_cc",
    "rules_java",
    "rules_license",
    "rules_python",
    "rules_rust",
    "rules_shell",
    "spdlog",
    "toolchains_llvm",
    "zlib",
]

# This is the list of repositories that Drake provides as a module extension
# for downstream projects; see comments in drake/MODULE.bazel for details.
# It is cross-checked by the workspace_bzlmod_sync_test.py test.
REPOS_EXPORTED = [
    "blas",
    "buildifier",
    "drake_models",
    "eigen",
    "fmt",
    "gflags",
    "glib",
    "gtest",
    "gurobi",
    "lapack",
    "lcm",
    "meshcat",
    "mosek",
    "opencl",
    "pybind11",
    "pycodestyle",
    "python",
    "snopt",
    "spdlog",
    "styleguide",
    "x11",
    "zlib",
]

def _drake_dep_repositories_impl(module_ctx):
    # This sequence should match REPOS_EXPORTED exactly.
    # Mismatches will be reported as errors by Bazel.
    mirrors = DEFAULT_MIRRORS
    blas_repository(name = "blas")
    buildifier_repository(name = "buildifier", mirrors = mirrors)
    drake_models_repository(name = "drake_models", mirrors = mirrors)
    gflags_repository(name = "gflags", mirrors = mirrors)
    glib_repository(name = "glib")
    gtest_repository(name = "gtest", mirrors = mirrors)
    gurobi_repository(name = "gurobi")
    lapack_repository(name = "lapack")
    lcm_repository(name = "lcm", mirrors = mirrors)
    meshcat_repository(name = "meshcat", mirrors = mirrors)
    mosek_repository(name = "mosek", mirrors = mirrors)
    opencl_repository(name = "opencl")
    pybind11_repository(name = "pybind11", mirrors = mirrors)
    pycodestyle_repository(name = "pycodestyle", mirrors = mirrors)
    python_repository(name = "python")
    snopt_repository(name = "snopt")
    styleguide_repository(name = "styleguide", mirrors = mirrors)
    x11_repository(name = "x11")
    zlib_repository(name = "zlib", _legacy_workspace = False)
    for name in ["eigen", "fmt", "spdlog"]:
        alias_repository(
            name = name,
            aliases = {name: "@drake//tools/workspace/" + name},
        )

drake_dep_repositories = module_extension(
    implementation = _drake_dep_repositories_impl,
    doc = """(Stable API) Provides access to Drake's dependencies for use by
    downstream projects. See comments in drake/MODULE.bazel for details.""",
)

def _internal_repositories_impl(module_ctx):
    # Add the repository rules (shared code with WORKSPACE mode).
    excludes = (
        REPOS_ALREADY_PROVIDED_BY_BAZEL_MODULES +
        REPOS_EXPORTED +
        ["crate_universe"]
    )
    add_default_repositories(
        excludes = excludes,
        _is_drake_self_call = True,
    )

    # Add the MODULE-only deps (not shared with WORKSPACE mode).
    spgrid_module_extension_impl(module_ctx)

internal_repositories = module_extension(
    implementation = _internal_repositories_impl,
    doc = """(Internal use only) Wraps the add_default_repositories repository
    rule into a bzlmod module extension, excluding repositories that are
    already covered by modules, drake_dep_repositories, and crate_universe.""",
)

def _internal_crate_universe_repositories_impl(module_ctx):
    crate_universe_repositories(
        mirrors = DEFAULT_MIRRORS,
        _is_drake_self_call = True,
    )

internal_crate_universe_repositories = module_extension(
    implementation = _internal_crate_universe_repositories_impl,
    doc = """(Internal use only) Wraps the crate_universe repository rules to
    be usable as a bzlmod module extension.""",
)
