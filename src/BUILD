package(default_visibility = ["//visibility:public"])

# bazel doc https://bazel.build/reference/be/c-cpp#cc_library
cc_binary(
    name = "tywebrtc",
    srcs = glob(["**/*.cc", "**/*.h"], exclude = ["**/*_test.cc"],),
	  # hdrs = glob(["**/*.h"]),
    # includes = ['.'],
    # OPT: remove error=old-style-cast
    # shit sctp header file use condition compile
	  copts = ["-D SCTP_DEBUG", "-I src", "-Werror", "-Wall", "-Wextra", "-Wno-error=old-style-cast", "-Wno-error=deprecated-declarations"],
    linkopts = ["-l dl -pthread -l z"],
    deps = [
        "@tylib//:tylib",
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
  srcs = glob([
    "**/*_test.cc",
    "**/*.h"
  ]),

  copts = [
    "-I src",
    # "-Iexternal/googletest/include",
  ],

  deps = [
    "@com_google_googletest//:gtest_main",
    "@tylib//:tylib",
    "@librtmp//:librtmp_package",
    "@com_github_jupp0r_prometheus_cpp//pull",
  ],
)
