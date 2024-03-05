# 1. tylib
local_repository(
    name = "tylib",
    path = "third_party/tylib",
)

# depend on non-bazel project
# https://bazel.build/docs/external?hl=zh-cn#bazel-projects

# 2. openssl
# https://stackoverflow.com/questions/49937820/include-headers-h-installed-in-non-standard-location
new_local_repository(
    name = "openssl",
    path = "third_party/openssl",
    # path = select({
    # "@bazel_tools//src/conditions:linux": "openssl",
    # "@bazel_tools//src/conditions:darwin": "/opt/homebrew/opt/openssl"}),

    # ssl rely on crypto lib, NOTE link order!
    # https://stackoverflow.com/questions/12917731/linking-issues-using-openssl-in-ubuntu#comment33092031_12917932
    # I experienced the same problem, however I was linking with -lcrypto. What fixed it for me was by linking with crypto after ssl. Why the order matters goes beyond me. 
    # @climax Order matters with libraries. Until libssl was processed, OPENSSL_add_all_algorithms_noconf etc were not needed. So when libcrypto was first processed, they were not included in the build. Later, when they were needed, libcrypto had already been processed.

    build_file_content = """
package(default_visibility = ["//visibility:public"])
cc_library(
    name = "openssl_package",
    srcs = ["libssl.a", "libcrypto.a"],
    hdrs = glob(["include/**/*.h"]),
    includes = ["include/"],
)
"""
)

# 3. srtp
new_local_repository(
    name = "srtp",
    path = "third_party/libsrtp",
    # path = select({
    # "@bazel_tools//src/conditions:linux": "srtp",
    # "@bazel_tools//src/conditions:darwin": "/opt/homebrew/opt/srtp"}),

    build_file_content = """
package(default_visibility = ["//visibility:public"])
cc_library(
    name = "libsrtp_package",
    srcs = ["libsrtp2.a"],
    hdrs = glob(["include/**/*.h"]),
    includes = ["include"],
)
"""
)

# 4. gtest
# doc https://google.github.io/googletest/quickstart-bazel.html#set-up-a-bazel-workspace
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

http_archive(
    name = "com_google_googletest",
    urls = ["https://github.com/google/googletest/archive/5ab508a01f9eb089207ee87fd547d290da39d015.zip"],
    strip_prefix = "googletest-5ab508a01f9eb089207ee87fd547d290da39d015",
)

# 5. prometheus-cpp client
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive", "http_file")

http_archive(
    name = "com_github_jupp0r_prometheus_cpp",
    strip_prefix = "prometheus-cpp-master",
    urls = ["https://github.com/jupp0r/prometheus-cpp/archive/master.zip"],
)

load("@com_github_jupp0r_prometheus_cpp//bazel:repositories.bzl", "prometheus_cpp_repositories")

prometheus_cpp_repositories()

# 6. sctp
new_local_repository(
    name = "sctp",
    path = "third_party/usrsctp",
    build_file_content = """
package(default_visibility = ["//visibility:public"])
cc_library(
    name = "sctp_package",
    srcs = ["usrsctplib/.libs/libusrsctp.a"],
    hdrs = ["."],
    includes = ["."],
)
"""
)

# 7. librtmp
new_local_repository(
    name = "librtmp",
    path = "third_party/rtmpdump",
    # path = select({
    # "@bazel_tools//src/conditions:linux": "srtp",
    # "@bazel_tools//src/conditions:darwin": "/opt/homebrew/opt/srtp"}),

    build_file_content = """
package(default_visibility = ["//visibility:public"])
cc_library(
    name = "librtmp_package",
    srcs = ["librtmp/librtmp.a"],
    hdrs = ["."],
    includes = ["."],
)
"""
)

# 8. ffmpeg
new_local_repository(
    name = "ffmpeg",
    path = "third_party/ffmpeg",
    # path = select({
    # "@bazel_tools//src/conditions:linux": "srtp",
    # "@bazel_tools//src/conditions:darwin": "/opt/homebrew/opt/srtp"}),

    build_file_content = """
package(default_visibility = ["//visibility:public"])
cc_library(
    name = "ffmpeg_package",
    srcs = ["libavformat/libavformat.a", "libavcodec/libavcodec.a", "libavutil/libavutil.a", "libswresample/libswresample.a", "libswscale/libswscale.a"],
    hdrs = ["."],
    includes = ["."],
)
"""
)

# 9. opus
new_local_repository(
    name = "opus",
    path = "third_party/opus",
    # path = select({
    # "@bazel_tools//src/conditions:linux": "srtp",
    # "@bazel_tools//src/conditions:darwin": "/opt/homebrew/opt/srtp"}),

    build_file_content = """
package(default_visibility = ["//visibility:public"])
cc_library(
    name = "opus_package",
    srcs = [".libs/libopus.a"],
    hdrs = glob(["include/**/*.h"]),
    includes = ["include"],
)
"""
)

# 10. libvpx
new_local_repository(
    name = "libvpx",
    path = "third_party/libvpx",
    # path = select({
    # "@bazel_tools//src/conditions:linux": "srtp",
    # "@bazel_tools//src/conditions:darwin": "/opt/homebrew/opt/srtp"}),

    build_file_content = """
package(default_visibility = ["//visibility:public"])
cc_library(
    name = "libvpx_package",
    srcs = ["libvpx.a"],
    hdrs = glob(["include/**/*.h"]),
    includes = ["include"],
)
"""
)

# 10. x264
new_local_repository(
    name = "x264",
    path = "third_party/x264",
    # path = select({
    # "@bazel_tools//src/conditions:linux": "srtp",
    # "@bazel_tools//src/conditions:darwin": "/opt/homebrew/opt/srtp"}),

    build_file_content = """
package(default_visibility = ["//visibility:public"])
cc_library(
    name = "libx264_package",
    srcs = ["libx264.a"],
    hdrs = glob(["include/**/*.h"]),
    includes = ["include"],
)
"""
)

# 11. srt
new_local_repository(
    name = "srt",
    path = "third_party/srt",
    # path = select({
    # "@bazel_tools//src/conditions:linux": "srtp",
    # "@bazel_tools//src/conditions:darwin": "/opt/homebrew/opt/srtp"}),

    build_file_content = """
package(default_visibility = ["//visibility:public"])
cc_library(
    name = "srt_package",
    srcs = ["libsrt.a"],
    hdrs = glob(["include/**/*.h"]),
    includes = ["include"],
)
"""
)

# 12. libco
# doc https://blog.csdn.net/GreyBtfly/article/details/83688996
# new_local_repository(
#     name = "libco",
#     path = "third_party",
#     # path = select({
#     # "@bazel_tools//src/conditions:linux": "srtp",
#     # "@bazel_tools//src/conditions:darwin": "/opt/homebrew/opt/srtp"}),
# 
#     build_file_content = """
# package(default_visibility = ["//visibility:public"])
# cc_library(
#     name = "libco_package",
#     srcs = ["libco/libcolib.a"],
#     hdrs = glob(["libco/*.h"]),
#     includes = ["libco"],
# )
# """
# )

# load("@bazel_tools//tools/build_defs/repo:local.bzl", "local_repository")

local_repository(
    name = "libco",
    path = "third_party/basic/",
)

# 13. rsfec
local_repository(
    name = "rsfec",
    path = "third_party/rsfec-cpp/",
)

# https://github.com/naivesystems/analyze/wiki/%E5%A6%82%E4%BD%95%E6%A3%80%E6%9F%A5%E4%BD%BF%E7%94%A8-Bazel-%E6%9E%84%E5%BB%BA%E7%9A%84%E9%A1%B9%E7%9B%AE
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

# Hedron's Compile Commands Extractor for Bazel
# https://github.com/hedronvision/bazel-compile-commands-extractor
http_archive(
    name = "hedron_compile_commands",

    # Replace the commit hash (daae6f40adfa5fdb7c89684cbe4d88b691c63b2d) in both places (below) with the latest (https://github.com/hedronvision/bazel-compile-commands-extractor/commits/main), rather than using the stale one here.
    # Even better, set up Renovate and let it do the work for you (see "Suggestion: Updates" in the README).
    url = "https://github.com/hedronvision/bazel-compile-commands-extractor/archive/daae6f40adfa5fdb7c89684cbe4d88b691c63b2d.tar.gz",
    strip_prefix = "bazel-compile-commands-extractor-daae6f40adfa5fdb7c89684cbe4d88b691c63b2d",
    # When you first run this tool, it'll recommend a sha256 hash to put here with a message like: "DEBUG: Rule 'hedron_compile_commands' indicated that a canonical reproducible form can be obtained by modifying arguments sha256 = ..."
)
load("@hedron_compile_commands//:workspace_setup.bzl", "hedron_compile_commands_setup")
hedron_compile_commands_setup()