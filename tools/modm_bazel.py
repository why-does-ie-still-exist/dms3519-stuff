#!/usr/bin/env python3
"""Generate modm/BUILD.bazel and modm/flags.bzl from lbuild's Make output.

modm has no Bazel generator, but its `modm:build:make` module writes
modm/repo.mk with the exact compiler/linker flags and source list the
library needs.  This script parses that file so Bazel builds modm with the
same flags the modm project uses.  Run it after every `lbuild build`
(tools/generate.sh does both).
"""
import os
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
MODM = ROOT / "modm"
REPO_MK = MODM / "repo.mk"

SRC_EXTS = (".c", ".cpp", ".cxx", ".cc", ".c++", ".sx", ".s", ".S")


def parse_make(text):
    """Return {var: {profile: [values]}} for `VAR := \\` / `VAR += \\` lists.

    profile is "" for unconditional lists, or the `ifeq ($(profile),X)` name.
    Custom per-file rules are returned separately as {source: [flags]}.
    """
    vars_ = {}
    custom = {}
    profile = ""
    lines = text.replace("\\\n", " ").splitlines()
    i = 0
    while i < len(lines):
        line = lines[i].strip()
        m = re.match(r"ifeq \(\$\(profile\),(\w+)\)", line)
        if m:
            profile = m.group(1)
        elif line == "endif":
            profile = ""
        else:
            m = re.match(r"^([A-Z_0-9]+)\s*[:+]?=\s*(.*)$", line)
            if m and m.group(1) in (
                "CCFLAGS", "CFLAGS", "CXXFLAGS", "ASFLAGS", "LINKFLAGS",
                "ARCHFLAGS", "CPPDEFINES", "MODM_OBJS",
            ):
                vals = m.group(2).split()
                vars_.setdefault(m.group(1), {}).setdefault(profile, []).extend(vals)
            m = re.match(r"^\$\(BUILDPATH\)/(\S+)\.o:\s*\$\(OUTPATH\)/(\S+)", line)
            if m:
                # next line: @$(call compile_x,$@,$<, <flags>)
                rule = lines[i + 1].strip()
                fm = re.search(r"\$\(call compile_\w+,\$@,\$<,\s*(.*)\)\s*$", rule)
                if fm:
                    custom[m.group(2)] = fm.group(1).split()
                    i += 1
        i += 1
    return vars_, custom


def resolve_sources(objs):
    srcs = []
    for o in objs:
        rel = o.replace("$(BUILDPATH)/modm/", "")
        stem = rel[:-2]  # strip .o
        for ext in SRC_EXTS:
            if (MODM / (stem + ext)).exists():
                srcs.append(stem + ext)
                break
        else:
            sys.exit(f"modm_bazel: no source found for object {o}")
    return srcs


def clean_flags(flags):
    """Drop flags that only make sense for modm's own Make build."""
    out = []
    for f in flags:
        if f.startswith(("-ffile-prefix-map=", "-L$(CURDIR)", "-Tmodm/", "-Wl,-Map")):
            continue
        if f == "-I." or f.startswith("$("):  # make variable refs (ARCHFLAGS is added separately)
            continue
        out.append(f)
    return out


def bzl_list(name, values, indent=0):
    pad = " " * indent
    body = "".join(f'{pad}    "{v}",\n' for v in values)
    return f"{pad}{name} = [\n{body}{pad}]\n"


def main():
    if not REPO_MK.exists():
        sys.exit("modm_bazel: modm/repo.mk not found; run `lbuild build` first")
    vars_, custom = parse_make(REPO_MK.read_text())

    arch = vars_["ARCHFLAGS"][""]
    cc = clean_flags(vars_["CCFLAGS"][""])
    cc_release = vars_["CCFLAGS"].get("release", [])
    cc_debug = vars_["CCFLAGS"].get("debug", [])
    cflags = clean_flags(vars_["CFLAGS"][""])
    cxxflags = clean_flags(vars_["CXXFLAGS"][""])
    asflags = clean_flags(vars_["ASFLAGS"][""])
    linkflags = clean_flags(vars_["LINKFLAGS"][""])
    includes = [d[2:].replace("$(MODM_PATH)/", "") for d in vars_["CPPDEFINES"][""] if d.startswith("-I")]
    defines = [d[2:] for d in vars_["CPPDEFINES"][""] if d.startswith("-D")]
    srcs = resolve_sources(vars_["MODM_OBJS"][""])

    # Bazel's cc rules do not recognise modm's `.sx` assembly extension.
    asm_copies = [s for s in srcs if s.endswith(".sx")]
    srcs = [s[:-3] + ".S" if s.endswith(".sx") else s for s in srcs]

    # Sources that modm compiles with extra flags get their own library.
    special = {}
    for src, flags in custom.items():
        src = src.replace("modm/", "", 1)
        if src in srcs:
            srcs.remove(src)
            special[src] = flags

    flags_bzl = '"""Compiler flags extracted from modm/repo.mk by tools/modm_bazel.py."""\n\n'
    flags_bzl += bzl_list("MODM_ARCHFLAGS", arch)
    flags_bzl += bzl_list("MODM_CCFLAGS", cc)
    flags_bzl += bzl_list("MODM_CCFLAGS_RELEASE", cc_release)
    flags_bzl += bzl_list("MODM_CCFLAGS_DEBUG", cc_debug)
    flags_bzl += bzl_list("MODM_CFLAGS", cflags)
    flags_bzl += bzl_list("MODM_CXXFLAGS", cxxflags)
    flags_bzl += bzl_list("MODM_ASFLAGS", asflags)
    flags_bzl += bzl_list("MODM_LINKFLAGS", linkflags)
    flags_bzl += bzl_list("MODM_DEFINES", defines)
    flags_bzl += '''
# Flags for every C/C++ compile (modm's CCFLAGS + ARCHFLAGS + profile flags).
def modm_copts():
    return MODM_ARCHFLAGS + MODM_CCFLAGS + select({
        "@rules_cc//cc/compiler:gcc": [],
        "//conditions:default": [],
    }) + select({
        "//modm:opt", MODM_CCFLAGS_RELEASE,
        "//conditions:default": MODM_CCFLAGS_DEBUG,
    })
'''
    # Keep the select simple: opt build -> release flags, else debug flags.
    flags_bzl = flags_bzl.replace('''select({
        "@rules_cc//cc/compiler:gcc": [],
        "//conditions:default": [],
    }) + ''', "")
    flags_bzl = flags_bzl.replace('"//modm:opt", MODM_CCFLAGS_RELEASE,', '"//modm:opt": MODM_CCFLAGS_RELEASE,')

    build = '''# GENERATED by tools/modm_bazel.py from modm/repo.mk -- do not edit.
load("@bazel_skylib//rules:copy_file.bzl", "copy_file")
load("@rules_cc//cc:defs.bzl", "cc_library")
load(":flags.bzl", "MODM_ARCHFLAGS", "MODM_ASFLAGS", "MODM_CFLAGS", "MODM_CXXFLAGS", "MODM_DEFINES", "MODM_LINKFLAGS", "modm_copts")

package(default_visibility = ["//visibility:public"])

config_setting(
    name = "opt",
    values = {"compilation_mode": "opt"},
)

exports_files(["link/linkerscript.ld", "openocd.cfg"])

MODM_HDRS = glob([
    "src/**/*.h",
    "src/**/*.hpp",
    "ext/**/*.h",
    "ext/**/*.hpp",
])

'''
    for s in asm_copies:
        build += f'''copy_file(
    name = "{s.replace('/', '_')}",
    src = "{s}",
    out = "{s[:-3]}.S",
)

'''
    build += bzl_list("MODM_INCLUDES", includes)
    build += '''
cc_library(
    name = "modm",
    srcs = [
'''
    build += "".join(f'        "{s}",\n' for s in sorted(srcs))
    build += '''    ],
    hdrs = MODM_HDRS,
    conlyopts = MODM_CFLAGS,
    copts = modm_copts() + MODM_ARCHFLAGS + MODM_ASFLAGS,
    cxxopts = MODM_CXXFLAGS,
    defines = MODM_DEFINES,
    includes = MODM_INCLUDES,
    linkopts = MODM_LINKFLAGS + MODM_ARCHFLAGS + ["-T$(location link/linkerscript.ld)"],
    additional_linker_inputs = ["link/linkerscript.ld"],
    alwayslink = True,
    deps = [
'''
    build += "".join(f'        ":{s.replace("/", "_").replace(".", "_")}",\n' for s in special)
    build += '''    ],
)
'''
    for s, flags in special.items():
        build += f'''
cc_library(
    name = "{s.replace('/', '_').replace('.', '_')}",
    srcs = ["{s}"],
    hdrs = MODM_HDRS,
    conlyopts = MODM_CFLAGS,
    copts = modm_copts() + {flags!r},
    cxxopts = MODM_CXXFLAGS,
    defines = MODM_DEFINES,
    includes = MODM_INCLUDES,
    alwayslink = True,
)
'''
    (MODM / "flags.bzl").write_text(flags_bzl)
    (MODM / "BUILD.bazel").write_text(build)
    print(f"wrote modm/BUILD.bazel ({len(srcs) + len(special)} sources) and modm/flags.bzl")


if __name__ == "__main__":
    main()
