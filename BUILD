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

