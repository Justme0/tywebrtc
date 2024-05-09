# too newer:
# load("@protobuf//bazel:cc_proto_library.bzl", "cc_proto_library")
# load("@com_google_protobuf//:protobuf.bzl", "cc_proto_library")
# load("@rules_cc//cc:defs.bzl", "cc_library")
# load("@rules_proto//proto:defs.bzl", "proto_library")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "client_cc_proto",
    deps = [":client_proto"],
)
proto_library(
    name = "client_proto",
    srcs = ["client.proto"],
)
