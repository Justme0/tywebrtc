package(default_visibility = ["//visibility:public"])

# https://github.com/naivesystems/analyze/wiki/%E4%A6%82%E4%BD%95%E6%A3%80%E6%9F%A5%E4%BD%BF%E7%94%A8-Bazel-%E6%9E%84%E5%BB%BA%E7%9A%84%E9%A1%B9%E7%9B%AE
load("@hedron_compile_commands//:refresh_compile_commands.bzl", "refresh_compile_commands")

refresh_compile_commands(
    name = "refresh_compile_commands",
    targets = {
        "//src/tywebrtc": "",
    },
    exclude_headers = "all",
    exclude_external_sources = True,
)

# bazel doc https://bazel.build/reference/be/c-cpp#cc_library
cc_binary(
    name = "tywebrtc",
    srcs = glob(["src/**/*.cc", "src/**/*.h"], exclude = ["src/**/*_test.cc"],),

    # OPT: remove error=old-style-cast
    # shit sctp header file use condition compile
    copts = ["-D SCTP_DEBUG", "-Werror", "-Wall", "-Wextra", "-Wno-error=old-style-cast", "-Wno-error=deprecated-declarations"],

    # ffmpeg need bz2 tls drm, should be in WORKSPACE ffmpeg dep lib
    linkopts = ["-l dl -pthread -l z -l bz2 -l gnutls -l drm"],
    deps = [
        "@nlohmann_json//:json",
        "@tylib//:tylib",
        "@rsfec//:rsfec",
        "@sctp//:sctp_package",
        "@librtmp//:librtmp_package",
        "@srtp//:libsrtp_package",
        "@com_github_jupp0r_prometheus_cpp//pull",
        "@ffmpeg//:ffmpeg_package",
        "@opus//:opus_package",
        "@libvpx//:libvpx_package",
        "@x264//:libx264_package",
        "@srt//:srt_package",
        # srt depends openssl, so in the front
        "@openssl//:openssl_package",
        # "@libco//:libco_package",
        "@libco//colib:colib",
    ],
)

# gtest at least C++14 compiler
# http://google.github.io/googletest/quickstart-bazel.html
cc_test(
  name = "tywebrtc_test",
  size = "small",
  srcs = glob(["src/**/*_test.cc", "src/**/*.h"]),

  deps = [
    "@com_github_jupp0r_prometheus_cpp//pull",
    "@com_google_googletest//:gtest_main",
    "@librtmp//:librtmp_package",
    "@tylib//:tylib",
  ],
)
