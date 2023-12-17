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
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

http_archive(
    name = "com_google_googletest",
    urls = ["https://github.com/google/googletest/archive/609281088cfefc76f9d0ce82e1ff6c30cc3591e5.zip"],
    strip_prefix = "googletest-609281088cfefc76f9d0ce82e1ff6c30cc3591e5",
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

# libco
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