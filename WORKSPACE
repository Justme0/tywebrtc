local_repository(
    name = "tylib",
    path = "third_party/tylib",
)

# for third-party lib
# https://stackoverflow.com/questions/49937820/include-headers-h-installed-in-non-standard-location
new_local_repository(
    name = "openssl",
    path = "/opt/homebrew/opt/openssl",
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
    path = "/opt/homebrew/opt/srtp",
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