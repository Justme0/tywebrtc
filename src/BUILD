package(default_visibility = ["//visibility:public"])

# bazel doc https://bazel.build/reference/be/c-cpp#cc_library
cc_binary(
    name = "server_tywebrtc",
    srcs = glob(["**/*.cc", "**/*.h"]),
	# hdrs = glob(["**/*.h"]),
    # includes = ['.'],
	copts = ["-I src"],
    linkopts = ["-l dl -pthreads"],
    deps = [
        "@tylib//:tylib",
        "@openssl//:openssl_package",
        "@srtp//:srtp_package",
    ],
)