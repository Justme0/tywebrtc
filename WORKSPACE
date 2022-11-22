local_repository(
    name = "tylib",
    path = "third_party/tylib",
)

# for third-party lib
# https://stackoverflow.com/questions/49937820/include-headers-h-installed-in-non-standard-location
new_local_repository(
    name = "openssl",
    path = "third_party/openssl",
    # path = select({
    # "@bazel_tools//src/conditions:linux": "openssl",
    # "@bazel_tools//src/conditions:darwin": "/opt/homebrew/opt/openssl"}),

    build_file_content = """
package(default_visibility = ["//visibility:public"])
cc_library(
    name = "openssl_package",
    srcs = ["lib/libssl.a", "lib/libcrypto.a"],
    hdrs = glob(["include/**/*.h"]),
    includes = ["include/"],
)
"""
)

new_local_repository(
    name = "srtp",
    path = "third_party/srtp",
    # path = select({
    # "@bazel_tools//src/conditions:linux": "srtp",
    # "@bazel_tools//src/conditions:darwin": "/opt/homebrew/opt/srtp"}),

    build_file_content = """
package(default_visibility = ["//visibility:public"])
cc_library(
    name = "srtp_package",
    srcs = ["lib/libsrtp2.a"],
    hdrs = glob(["include/**/*.h"]),
    includes = ["include"],
)
"""
)

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

http_archive(
  name = "com_google_googletest",
  urls = ["https://github.com/google/googletest/archive/609281088cfefc76f9d0ce82e1ff6c30cc3591e5.zip"],
  strip_prefix = "googletest-609281088cfefc76f9d0ce82e1ff6c30cc3591e5",
)