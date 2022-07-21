package(default_visibility = ["//visibility:public"])

# exports_files(["log"])

cc_library(
    name = "tywebrtclib",
    srcs = glob(["**/*.cc"]),
	hdrs = glob(["**/*.h"]),
    # includes = ['.'],
	copts = ["-I src"],

    deps = [
        "@tylib//:tylib",
    ],
)

cc_binary(
    name = "server_tywebrtc",
    srcs = ["main.cc"],
    linkopts = [
        "-l ssl",
        "-l crypto",
        "-l srtp2",
    ],
	copts = ["-I src"],
    deps = [
        ":tywebrtclib",
    ],
)