# GRPC global makefile
# This currently builds C and C++ code.
# This file has been automatically generated from a template file.
# Please look at the templates directory instead.
# This file can be regenerated from the template by running
# tools/buildgen/generate_projects.sh

# Copyright 2015, Google Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:
#
#     * Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above
# copyright notice, this list of conditions and the following disclaimer
# in the documentation and/or other materials provided with the
# distribution.
#     * Neither the name of Google Inc. nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.



# Basic platform detection
HOST_SYSTEM = $(shell uname | cut -f 1 -d_)
ifeq ($(SYSTEM),)
SYSTEM = $(HOST_SYSTEM)
endif
ifeq ($(SYSTEM),MSYS)
SYSTEM = MINGW32
endif
ifeq ($(SYSTEM),MINGW64)
SYSTEM = MINGW32
endif


MAKEFILE_PATH := $(abspath $(lastword $(MAKEFILE_LIST)))
ifndef BUILDDIR
BUILDDIR_ABSOLUTE = $(patsubst %/,%,$(dir $(MAKEFILE_PATH)))
else
BUILDDIR_ABSOLUTE = $(abspath $(BUILDDIR))
endif

HAS_GCC = $(shell which gcc > /dev/null 2> /dev/null && echo true || echo false)
HAS_CC = $(shell which cc > /dev/null 2> /dev/null && echo true || echo false)
HAS_CLANG = $(shell which clang > /dev/null 2> /dev/null && echo true || echo false)

ifeq ($(HAS_CC),true)
DEFAULT_CC = cc
DEFAULT_CXX = c++
else
ifeq ($(HAS_GCC),true)
DEFAULT_CC = gcc
DEFAULT_CXX = g++
else
ifeq ($(HAS_CLANG),true)
DEFAULT_CC = clang
DEFAULT_CXX = clang++
else
DEFAULT_CC = no_c_compiler
DEFAULT_CXX = no_c++_compiler
endif
endif
endif


BINDIR = $(BUILDDIR_ABSOLUTE)/bins
OBJDIR = $(BUILDDIR_ABSOLUTE)/objs
LIBDIR = $(BUILDDIR_ABSOLUTE)/libs
GENDIR = $(BUILDDIR_ABSOLUTE)/gens

# Configurations

VALID_CONFIG_opt = 1
CC_opt = $(DEFAULT_CC)
CXX_opt = $(DEFAULT_CXX)
LD_opt = $(DEFAULT_CC)
LDXX_opt = $(DEFAULT_CXX)
CPPFLAGS_opt = -O2
LDFLAGS_opt = -rdynamic
DEFINES_opt = NDEBUG

VALID_CONFIG_basicprof = 1
CC_basicprof = $(DEFAULT_CC)
CXX_basicprof = $(DEFAULT_CXX)
LD_basicprof = $(DEFAULT_CC)
LDXX_basicprof = $(DEFAULT_CXX)
CPPFLAGS_basicprof = -O2 -DGRPC_BASIC_PROFILER -DGRPC_TIMERS_RDTSC
LDFLAGS_basicprof =
DEFINES_basicprof = NDEBUG

VALID_CONFIG_stapprof = 1
CC_stapprof = $(DEFAULT_CC)
CXX_stapprof = $(DEFAULT_CXX)
LD_stapprof = $(DEFAULT_CC)
LDXX_stapprof = $(DEFAULT_CXX)
CPPFLAGS_stapprof = -O2 -DGRPC_STAP_PROFILER
LDFLAGS_stapprof =
DEFINES_stapprof = NDEBUG

VALID_CONFIG_dbg = 1
CC_dbg = $(DEFAULT_CC)
CXX_dbg = $(DEFAULT_CXX)
LD_dbg = $(DEFAULT_CC)
LDXX_dbg = $(DEFAULT_CXX)
CPPFLAGS_dbg = -O0
LDFLAGS_dbg = -rdynamic
DEFINES_dbg = _DEBUG DEBUG

VALID_CONFIG_mutrace = 1
CC_mutrace = $(DEFAULT_CC)
CXX_mutrace = $(DEFAULT_CXX)
LD_mutrace = $(DEFAULT_CC)
LDXX_mutrace = $(DEFAULT_CXX)
CPPFLAGS_mutrace = -O0
LDFLAGS_mutrace = -rdynamic
DEFINES_mutrace = _DEBUG DEBUG

VALID_CONFIG_valgrind = 1
REQUIRE_CUSTOM_LIBRARIES_valgrind = 1
CC_valgrind = $(DEFAULT_CC)
CXX_valgrind = $(DEFAULT_CXX)
LD_valgrind = $(DEFAULT_CC)
LDXX_valgrind = $(DEFAULT_CXX)
CPPFLAGS_valgrind = -O0
LDFLAGS_valgrind = -rdynamic
DEFINES_valgrind = _DEBUG DEBUG GRPC_TEST_SLOWDOWN_BUILD_FACTOR=20

VALID_CONFIG_tsan = 1
REQUIRE_CUSTOM_LIBRARIES_tsan = 1
CC_tsan = clang
CXX_tsan = clang++
LD_tsan = clang
LDXX_tsan = clang++
CFLAGS_tsan = -O0 -fsanitize=thread -fno-omit-frame-pointer -Wno-unused-command-line-argument
CXXFLAGS_tsan = -O0 -fsanitize=thread -fno-omit-frame-pointer -Wno-unused-command-line-argument
LDFLAGS_tsan = -fsanitize=thread
DEFINES_tsan = NDEBUG GRPC_TEST_SLOWDOWN_BUILD_FACTOR=10

VALID_CONFIG_asan = 1
REQUIRE_CUSTOM_LIBRARIES_asan = 1
CC_asan = clang
CXX_asan = clang++
LD_asan = clang
LDXX_asan = clang++
CFLAGS_asan = -O0 -fsanitize=address -fno-omit-frame-pointer -Wno-unused-command-line-argument
CXXFLAGS_asan = -O0 -fsanitize=address -fno-omit-frame-pointer -Wno-unused-command-line-argument
LDFLAGS_asan = -fsanitize=address
DEFINES_asan = GRPC_TEST_SLOWDOWN_BUILD_FACTOR=3

VALID_CONFIG_msan = 1
REQUIRE_CUSTOM_LIBRARIES_msan = 1
CC_msan = clang
CXX_msan = clang++-libc++
LD_msan = clang
LDXX_msan = clang++-libc++
CFLAGS_msan = -O0 -fsanitize=memory -fsanitize-memory-track-origins -fno-omit-frame-pointer -DGTEST_HAS_TR1_TUPLE=0 -DGTEST_USE_OWN_TR1_TUPLE=1 -Wno-unused-command-line-argument
CXXFLAGS_msan = -O0 -fsanitize=memory -fsanitize-memory-track-origins -fno-omit-frame-pointer -DGTEST_HAS_TR1_TUPLE=0 -DGTEST_USE_OWN_TR1_TUPLE=1 -Wno-unused-command-line-argument
LDFLAGS_msan = -fsanitize=memory -DGTEST_HAS_TR1_TUPLE=0 -DGTEST_USE_OWN_TR1_TUPLE=1
DEFINES_msan = NDEBUG GRPC_TEST_SLOWDOWN_BUILD_FACTOR=4

VALID_CONFIG_ubsan = 1
REQUIRE_CUSTOM_LIBRARIES_ubsan = 1
CC_ubsan = clang
CXX_ubsan = clang++
LD_ubsan = clang
LDXX_ubsan = clang++
CFLAGS_ubsan = -O1 -fsanitize=undefined -fno-omit-frame-pointer -Wno-unused-command-line-argument
CXXFLAGS_ubsan = -O1 -fsanitize=undefined -fno-omit-frame-pointer -Wno-unused-command-line-argument
LDFLAGS_ubsan = -fsanitize=undefined
DEFINES_ubsan = NDEBUG GRPC_TEST_SLOWDOWN_BUILD_FACTOR=3

VALID_CONFIG_gcov = 1
CC_gcov = gcc
CXX_gcov = g++
LD_gcov = gcc
LDXX_gcov = g++
CFLAGS_gcov = -O0 -fprofile-arcs -ftest-coverage -Wno-return-type
CXXFLAGS_gcov = -O0 -fprofile-arcs -ftest-coverage -Wno-return-type
LDFLAGS_gcov = -fprofile-arcs -ftest-coverage -rdynamic
DEFINES_gcov = _DEBUG DEBUG GPR_GCOV


# General settings.
# You may want to change these depending on your system.

prefix ?= /usr/local

PROTOC = protoc
DTRACE = dtrace
CONFIG ?= opt
CC = $(CC_$(CONFIG))
CXX = $(CXX_$(CONFIG))
LD = $(LD_$(CONFIG))
LDXX = $(LDXX_$(CONFIG))
AR = ar
ifeq ($(SYSTEM),Linux)
STRIP = strip --strip-unneeded
else
ifeq ($(SYSTEM),Darwin)
STRIP = strip -x
else
STRIP = strip
endif
endif
INSTALL = install
RM = rm -f
PKG_CONFIG = pkg-config

ifndef VALID_CONFIG_$(CONFIG)
$(error Invalid CONFIG value '$(CONFIG)')
endif

ifeq ($(SYSTEM),Linux)
TMPOUT = /dev/null
else
TMPOUT = `mktemp /tmp/test-out-XXXXXX`
endif

# Detect if we can use C++11
CXX11_CHECK_CMD = $(CXX) -std=c++11 -o $(TMPOUT) -c test/build/c++11.cc
HAS_CXX11 = $(shell $(CXX11_CHECK_CMD) 2> /dev/null && echo true || echo false)

# The HOST compiler settings are used to compile the protoc plugins.
# In most cases, you won't have to change anything, but if you are
# cross-compiling, you can override these variables from GNU make's
# command line: make CC=cross-gcc HOST_CC=gcc

HOST_CC = $(CC)
HOST_CXX = $(CXX)
HOST_LD = $(LD)
HOST_LDXX = $(LDXX)

ifdef EXTRA_DEFINES
DEFINES += $(EXTRA_DEFINES)
endif

CFLAGS += -std=c99 -Wsign-conversion -Wconversion -Wshadow
ifeq ($(HAS_CXX11),true)
CXXFLAGS += -std=c++11
else
CXXFLAGS += -std=c++0x
endif
CFLAGS += -g -Wall -Wextra -Werror -Wno-long-long -Wno-unused-parameter
CXXFLAGS += -g -Wall -Wextra -Werror -Wno-long-long -Wno-unused-parameter
LDFLAGS += -g

CPPFLAGS += $(CPPFLAGS_$(CONFIG))
CFLAGS += $(CFLAGS_$(CONFIG))
CXXFLAGS += $(CXXFLAGS_$(CONFIG))
DEFINES += $(DEFINES_$(CONFIG)) INSTALL_PREFIX=\"$(prefix)\"
LDFLAGS += $(LDFLAGS_$(CONFIG))

ifneq ($(SYSTEM),MINGW32)
PIC_CPPFLAGS = -fPIC
CPPFLAGS += -fPIC
LDFLAGS += -fPIC
endif

INCLUDES = . include $(GENDIR)
LDFLAGS += -Llibs/$(CONFIG)

ifeq ($(SYSTEM),Darwin)
ifneq ($(wildcard /usr/local/ssl/include),)
INCLUDES += /usr/local/ssl/include
endif
ifneq ($(wildcard /opt/local/include),)
INCLUDES += /opt/local/include
endif
ifneq ($(wildcard /usr/local/include),)
INCLUDES += /usr/local/include
endif
LIBS = m z
ifneq ($(wildcard /usr/local/ssl/lib),)
LDFLAGS += -L/usr/local/ssl/lib
endif
ifneq ($(wildcard /opt/local/lib),)
LDFLAGS += -L/opt/local/lib
endif
ifneq ($(wildcard /usr/local/lib),)
LDFLAGS += -L/usr/local/lib
endif
endif

ifeq ($(SYSTEM),Linux)
LIBS = rt m pthread
LDFLAGS += -pthread
endif

ifeq ($(SYSTEM),MINGW32)
LIBS = m pthread
LDFLAGS += -pthread
endif

GTEST_LIB = -Ithird_party/googletest/include -Ithird_party/googletest third_party/googletest/src/gtest-all.cc
GTEST_LIB += -lgflags
ifeq ($(V),1)
E = @:
Q =
else
E = @echo
Q = @
endif

VERSION = 0.12.0.0

CPPFLAGS_NO_ARCH += $(addprefix -I, $(INCLUDES)) $(addprefix -D, $(DEFINES))
CPPFLAGS += $(CPPFLAGS_NO_ARCH) $(ARCH_FLAGS)

LDFLAGS += $(ARCH_FLAGS)
LDLIBS += $(addprefix -l, $(LIBS))
LDLIBSXX += $(addprefix -l, $(LIBSXX))

HOST_CPPFLAGS = $(CPPFLAGS)
HOST_CFLAGS = $(CFLAGS)
HOST_CXXFLAGS = $(CXXFLAGS)
HOST_LDFLAGS = $(LDFLAGS)
HOST_LDLIBS = $(LDLIBS)

# These are automatically computed variables.
# There shouldn't be any need to change anything from now on.

-include cache.mk

CACHE_MK =

HAS_PKG_CONFIG ?= $(shell command -v $(PKG_CONFIG) >/dev/null 2>&1 && echo true || echo false)

ifeq ($(HAS_PKG_CONFIG), true)
CACHE_MK += HAS_PKG_CONFIG = true,
endif

PC_TEMPLATE = prefix=$(prefix),exec_prefix=\$${prefix},includedir=\$${prefix}/include,libdir=\$${exec_prefix}/lib,,Name: $(PC_NAME),Description: $(PC_DESCRIPTION),Version: $(VERSION),Cflags: -I\$${includedir} $(PC_CFLAGS),Requires.private: $(PC_REQUIRES_PRIVATE),Libs: -L\$${libdir} $(PC_LIB),Libs.private: $(PC_LIBS_PRIVATE)

# gpr .pc file
PC_NAME = gRPC Portable Runtime
PC_DESCRIPTION = gRPC Portable Runtime
PC_CFLAGS = -pthread
PC_REQUIRES_PRIVATE =
PC_LIBS_PRIVATE = -lpthread
PC_LIB = -lgpr
ifneq ($(SYSTEM),Darwin)
PC_LIBS_PRIVATE += -lrt
endif
GPR_PC_FILE := $(PC_TEMPLATE)

ifeq ($(SYSTEM),MINGW32)
SHARED_EXT = dll
endif
ifeq ($(SYSTEM),Darwin)
SHARED_EXT = dylib
endif
ifeq ($(SHARED_EXT),)
SHARED_EXT = so.$(VERSION)
endif

ifeq ($(wildcard .git),)
IS_GIT_FOLDER = false
else
IS_GIT_FOLDER = true
endif

ifeq ($(SYSTEM),Linux)
OPENSSL_REQUIRES_DL = true
endif

ifeq ($(SYSTEM),Darwin)
OPENSSL_REQUIRES_DL = true
endif

ifeq ($(HAS_PKG_CONFIG),true)
OPENSSL_ALPN_CHECK_CMD = $(PKG_CONFIG) --atleast-version=1.0.2 openssl
OPENSSL_NPN_CHECK_CMD = $(PKG_CONFIG) --atleast-version=1.0.1 openssl
ZLIB_CHECK_CMD = $(PKG_CONFIG) --exists zlib
PROTOBUF_CHECK_CMD = $(PKG_CONFIG) --atleast-version=3.0.0-alpha-3 protobuf
else # HAS_PKG_CONFIG

ifeq ($(SYSTEM),MINGW32)
OPENSSL_LIBS = ssl32 eay32
else
OPENSSL_LIBS = ssl crypto
endif

OPENSSL_ALPN_CHECK_CMD = $(CC) $(CFLAGS) $(CPPFLAGS) -o $(TMPOUT) test/build/openssl-alpn.c $(addprefix -l, $(OPENSSL_LIBS)) $(LDFLAGS)
OPENSSL_NPN_CHECK_CMD = $(CC) $(CFLAGS) $(CPPFLAGS) -o $(TMPOUT) test/build/openssl-npn.c $(addprefix -l, $(OPENSSL_LIBS)) $(LDFLAGS)
ZLIB_CHECK_CMD = $(CC) $(CFLAGS) $(CPPFLAGS) -o $(TMPOUT) test/build/zlib.c -lz $(LDFLAGS)
PROTOBUF_CHECK_CMD = $(CXX) $(CXXFLAGS) $(CPPFLAGS) -o $(TMPOUT) test/build/protobuf.cc -lprotobuf $(LDFLAGS)

ifeq ($(OPENSSL_REQUIRES_DL),true)
OPENSSL_ALPN_CHECK_CMD += -ldl
OPENSSL_NPN_CHECK_CMD += -ldl
endif

endif # HAS_PKG_CONFIG

PERFTOOLS_CHECK_CMD = $(CC) $(CFLAGS) $(CPPFLAGS) -o $(TMPOUT) test/build/perftools.c -lprofiler $(LDFLAGS)

PROTOC_CHECK_CMD = which protoc > /dev/null
PROTOC_CHECK_VERSION_CMD = protoc --version | grep -q libprotoc.3
DTRACE_CHECK_CMD = which dtrace > /dev/null
SYSTEMTAP_HEADERS_CHECK_CMD = $(CC) $(CFLAGS) $(CPPFLAGS) -o $(TMPOUT) test/build/systemtap.c $(LDFLAGS)
ZOOKEEPER_CHECK_CMD = $(CC) $(CFLAGS) $(CPPFLAGS) -o $(TMPOUT) test/build/zookeeper.c $(LDFLAGS) -lzookeeper_mt

ifndef REQUIRE_CUSTOM_LIBRARIES_$(CONFIG)
HAS_SYSTEM_PERFTOOLS ?= $(shell $(PERFTOOLS_CHECK_CMD) 2> /dev/null && echo true || echo false)
ifeq ($(HAS_SYSTEM_PERFTOOLS),true)
DEFINES += GRPC_HAVE_PERFTOOLS
LIBS += profiler
CACHE_MK += HAS_SYSTEM_PERFTOOLS = true,
endif
endif

HAS_SYSTEM_PROTOBUF_VERIFY = $(shell $(PROTOBUF_CHECK_CMD) 2> /dev/null && echo true || echo false)
ifndef REQUIRE_CUSTOM_LIBRARIES_$(CONFIG)
HAS_SYSTEM_OPENSSL_ALPN ?= $(shell $(OPENSSL_ALPN_CHECK_CMD) 2> /dev/null && echo true || echo false)
ifeq ($(HAS_SYSTEM_OPENSSL_ALPN),true)
HAS_SYSTEM_OPENSSL_NPN = true
CACHE_MK += HAS_SYSTEM_OPENSSL_ALPN = true,
else
HAS_SYSTEM_OPENSSL_NPN ?= $(shell $(OPENSSL_NPN_CHECK_CMD) 2> /dev/null && echo true || echo false)
endif
ifeq ($(HAS_SYSTEM_OPENSSL_NPN),true)
CACHE_MK += HAS_SYSTEM_OPENSSL_NPN = true,
endif
HAS_SYSTEM_ZLIB ?= $(shell $(ZLIB_CHECK_CMD) 2> /dev/null && echo true || echo false)
ifeq ($(HAS_SYSTEM_ZLIB),true)
CACHE_MK += HAS_SYSTEM_ZLIB = true,
endif
HAS_SYSTEM_PROTOBUF ?= $(HAS_SYSTEM_PROTOBUF_VERIFY)
ifeq ($(HAS_SYSTEM_PROTOBUF),true)
CACHE_MK += HAS_SYSTEM_PROTOBUF = true,
endif
else
# override system libraries if the config requires a custom compiled library
HAS_SYSTEM_OPENSSL_ALPN = false
HAS_SYSTEM_OPENSSL_NPN = false
HAS_SYSTEM_ZLIB = false
HAS_SYSTEM_PROTOBUF = false
endif

HAS_PROTOC ?= $(shell $(PROTOC_CHECK_CMD) 2> /dev/null && echo true || echo false)
ifeq ($(HAS_PROTOC),true)
CACHE_MK += HAS_PROTOC = true,
HAS_VALID_PROTOC ?= $(shell $(PROTOC_CHECK_VERSION_CMD) 2> /dev/null && echo true || echo false)
ifeq ($(HAS_VALID_PROTOC),true)
CACHE_MK += HAS_VALID_PROTOC = true,
endif
else
HAS_VALID_PROTOC = false
endif

# Check for Systemtap (https://sourceware.org/systemtap/), first by making sure <sys/sdt.h> is present
# in the system and secondly by checking for the "dtrace" binary (on Linux, this is part of the Systemtap
# distribution. It's part of the base system on BSD/Solaris machines).
ifndef HAS_SYSTEMTAP
HAS_SYSTEMTAP_HEADERS = $(shell $(SYSTEMTAP_HEADERS_CHECK_CMD) 2> /dev/null && echo true || echo false)
HAS_DTRACE = $(shell $(DTRACE_CHECK_CMD) 2> /dev/null && echo true || echo false)
HAS_SYSTEMTAP = false
ifeq ($(HAS_SYSTEMTAP_HEADERS),true)
ifeq ($(HAS_DTRACE),true)
HAS_SYSTEMTAP = true
endif
endif
endif

ifeq ($(HAS_SYSTEMTAP),true)
CACHE_MK += HAS_SYSTEMTAP = true,
endif

HAS_ZOOKEEPER = $(shell $(ZOOKEEPER_CHECK_CMD) 2> /dev/null && echo true || echo false)

# Note that for testing purposes, one can do:
#   make HAS_EMBEDDED_OPENSSL_ALPN=false
# to emulate the fact we do not have OpenSSL in the third_party folder.
ifeq ($(wildcard third_party/boringssl/include/openssl/ssl.h),)
HAS_EMBEDDED_OPENSSL_ALPN = false
else
HAS_EMBEDDED_OPENSSL_ALPN = true
endif

ifeq ($(wildcard third_party/zlib/zlib.h),)
HAS_EMBEDDED_ZLIB = false
else
HAS_EMBEDDED_ZLIB = true
endif

ifeq ($(wildcard third_party/protobuf/src/google/protobuf/descriptor.pb.h),)
HAS_EMBEDDED_PROTOBUF = false
ifneq ($(HAS_VALID_PROTOC),true)
NO_PROTOC = true
endif
else
HAS_EMBEDDED_PROTOBUF = true
endif

PC_REQUIRES_GRPC = gpr
PC_LIBS_GRPC =

ifeq ($(HAS_SYSTEM_ZLIB),false)
ifeq ($(HAS_EMBEDDED_ZLIB),true)
ZLIB_DEP = $(LIBDIR)/$(CONFIG)/zlib/libz.a
ZLIB_MERGE_LIBS = $(LIBDIR)/$(CONFIG)/zlib/libz.a
CPPFLAGS += -Ithird_party/zlib
LDFLAGS += -L$(LIBDIR)/$(CONFIG)/zlib
else
DEP_MISSING += zlib
endif
else
ifeq ($(HAS_PKG_CONFIG),true)
CPPFLAGS += $(shell $(PKG_CONFIG) --cflags zlib)
LDFLAGS += $(shell $(PKG_CONFIG) --libs-only-L zlib)
LIBS += $(patsubst -l%,%,$(shell $(PKG_CONFIG) --libs-only-l zlib))
PC_REQUIRES_GRPC += zlib
else
PC_LIBS_GRPC += -lz
LIBS += z
endif
endif

OPENSSL_PKG_CONFIG = false

PC_REQUIRES_SECURE =
PC_LIBS_SECURE =

ifeq ($(HAS_SYSTEM_OPENSSL_ALPN),true)
EMBED_OPENSSL ?= false
NO_SECURE ?= false
else # HAS_SYSTEM_OPENSSL_ALPN=false
ifeq ($(HAS_EMBEDDED_OPENSSL_ALPN),true)
EMBED_OPENSSL ?= true
NO_SECURE ?= false
else # HAS_EMBEDDED_OPENSSL_ALPN=false
ifeq ($(HAS_SYSTEM_OPENSSL_NPN),true)
EMBED_OPENSSL ?= false
NO_SECURE ?= false
else
NO_SECURE ?= true
endif # HAS_SYSTEM_OPENSSL_NPN=true
endif # HAS_EMBEDDED_OPENSSL_ALPN
endif # HAS_SYSTEM_OPENSSL_ALPN

OPENSSL_DEP :=
OPENSSL_MERGE_LIBS :=
ifeq ($(NO_SECURE),false)
ifeq ($(EMBED_OPENSSL),true)
OPENSSL_DEP += $(LIBDIR)/$(CONFIG)/libboringssl.a
OPENSSL_MERGE_LIBS += $(LIBDIR)/$(CONFIG)/libboringssl.a
# need to prefix these to ensure overriding system libraries
CPPFLAGS := -Ithird_party/boringssl/include $(CPPFLAGS)
ifeq ($(OPENSSL_REQUIRES_DL),true)
LIBS_SECURE = dl
endif # OPENSSL_REQUIRES_DL
else # EMBED_OPENSSL=false
ifeq ($(HAS_PKG_CONFIG),true)
OPENSSL_PKG_CONFIG = true
PC_REQUIRES_SECURE = openssl
CPPFLAGS := $(shell $(PKG_CONFIG) --cflags openssl) $(CPPFLAGS)
LDFLAGS_OPENSSL_PKG_CONFIG = $(shell $(PKG_CONFIG) --libs-only-L openssl)
ifeq ($(SYSTEM),Linux)
ifneq ($(LDFLAGS_OPENSSL_PKG_CONFIG),)
LDFLAGS_OPENSSL_PKG_CONFIG += $(shell $(PKG_CONFIG) --libs-only-L openssl | sed s/L/Wl,-rpath,/)
endif # LDFLAGS_OPENSSL_PKG_CONFIG=''
endif # System=Linux
LDFLAGS := $(LDFLAGS_OPENSSL_PKG_CONFIG) $(LDFLAGS)
else # HAS_PKG_CONFIG=false
LIBS_SECURE = $(OPENSSL_LIBS)
endif # HAS_PKG_CONFIG
ifeq ($(HAS_SYSTEM_OPENSSL_NPN),true)
CPPFLAGS += -DTSI_OPENSSL_ALPN_SUPPORT=0
LIBS_SECURE = $(OPENSSL_LIBS)
endif # HAS_SYSTEM_OPENSSL_NPN
ifeq ($(OPENSSL_REQUIRES_DL),true)
LIBS_SECURE += dl
PC_LIBS_SECURE = $(addprefix -l, $(LIBS_SECURE))
endif # OPENSSL_REQUIRES_DL=true
endif # EMBED_OPENSSL
endif # NO_SECURE

ifeq ($(OPENSSL_PKG_CONFIG),true)
LDLIBS_SECURE += $(shell $(PKG_CONFIG) --libs-only-l openssl)
else
LDLIBS_SECURE += $(addprefix -l, $(LIBS_SECURE))
endif

# grpc .pc file
PC_NAME = gRPC
PC_DESCRIPTION = high performance general RPC framework
PC_CFLAGS =
PC_REQUIRES_PRIVATE = $(PC_REQUIRES_GRPC) $(PC_REQUIRES_SECURE)
PC_LIBS_PRIVATE = $(PC_LIBS_GRPC) $(PC_LIBS_SECURE)
PC_LIB = -lgrpc
GRPC_PC_FILE := $(PC_TEMPLATE)

# gprc_unsecure .pc file
PC_NAME = gRPC unsecure
PC_DESCRIPTION = high performance general RPC framework without SSL
PC_CFLAGS =
PC_REQUIRES_PRIVATE = $(PC_REQUIRES_GRPC)
PC_LIBS_PRIVATE = $(PC_LIBS_GRPC)
PC_LIB = -lgrpc
GRPC_UNSECURE_PC_FILE := $(PC_TEMPLATE)

# gprc_zookeeper .pc file
PC_NAME = gRPC zookeeper
PC_DESCRIPTION = gRPC's zookeeper plugin
PC_CFLAGS =
PC_REQUIRES_PRIVATE =
PC_LIBS_PRIVATE = -lzookeeper_mt
GRPC_ZOOKEEPER_PC_FILE := $(PC_TEMPLATE)

PROTOBUF_PKG_CONFIG = false

PC_REQUIRES_GRPCXX =
PC_LIBS_GRPCXX =

CPPFLAGS := -Ithird_party/googletest/include $(CPPFLAGS)

ifeq ($(HAS_SYSTEM_PROTOBUF),true)
ifeq ($(HAS_PKG_CONFIG),true)
PROTOBUF_PKG_CONFIG = true
PC_REQUIRES_GRPCXX = protobuf
CPPFLAGS := $(shell $(PKG_CONFIG) --cflags protobuf) $(CPPFLAGS)
LDFLAGS_PROTOBUF_PKG_CONFIG = $(shell $(PKG_CONFIG) --libs-only-L protobuf)
ifeq ($(SYSTEM),Linux)
ifneq ($(LDFLAGS_PROTOBUF_PKG_CONFIG),)
LDFLAGS_PROTOBUF_PKG_CONFIG += $(shell $(PKG_CONFIG) --libs-only-L protobuf | sed s/L/Wl,-rpath,/)
endif
endif
else
PC_LIBS_GRPCXX = -lprotobuf
endif
else
ifeq ($(HAS_EMBEDDED_PROTOBUF),true)
PROTOBUF_DEP = $(LIBDIR)/$(CONFIG)/protobuf/libprotobuf.a
CPPFLAGS := -Ithird_party/protobuf/src $(CPPFLAGS)
LDFLAGS := -L$(LIBDIR)/$(CONFIG)/protobuf $(LDFLAGS)
PROTOC = $(BINDIR)/$(CONFIG)/protobuf/protoc
else
NO_PROTOBUF = true
endif
endif

LIBS_PROTOBUF = protobuf
LIBS_PROTOC = protoc protobuf

HOST_LDLIBS_PROTOC += $(addprefix -l, $(LIBS_PROTOC))

ifeq ($(PROTOBUF_PKG_CONFIG),true)
LDLIBS_PROTOBUF += $(shell $(PKG_CONFIG) --libs-only-l protobuf)
else
LDLIBS_PROTOBUF += $(addprefix -l, $(LIBS_PROTOBUF))
endif

# grpc++ .pc file
PC_NAME = gRPC++
PC_DESCRIPTION = C++ wrapper for gRPC
PC_CFLAGS =
PC_REQUIRES_PRIVATE = grpc $(PC_REQUIRES_GRPCXX)
PC_LIBS_PRIVATE = $(PC_LIBS_GRPCXX)
PC_LIB = -lgrpc++
GRPCXX_PC_FILE := $(PC_TEMPLATE)

# grpc++_unsecure .pc file
PC_NAME = gRPC++ unsecure
PC_DESCRIPTION = C++ wrapper for gRPC without SSL
PC_CFLAGS =
PC_REQUIRES_PRIVATE = grpc_unsecure $(PC_REQUIRES_GRPCXX)
PC_LIBS_PRIVATE = $(PC_LIBS_GRPCXX)
PC_LIB = -lgrpc++
GRPCXX_UNSECURE_PC_FILE := $(PC_TEMPLATE)

ifeq ($(MAKECMDGOALS),clean)
NO_DEPS = true
endif

INSTALL_OK = false
ifeq ($(HAS_VALID_PROTOC),true)
ifeq ($(HAS_SYSTEM_PROTOBUF_VERIFY),true)
INSTALL_OK = true
endif
endif

.SECONDARY = %.pb.h %.pb.cc

PROTOC_PLUGINS = $(BINDIR)/$(CONFIG)/grpc_cpp_plugin $(BINDIR)/$(CONFIG)/grpc_csharp_plugin $(BINDIR)/$(CONFIG)/grpc_objective_c_plugin $(BINDIR)/$(CONFIG)/grpc_python_plugin $(BINDIR)/$(CONFIG)/grpc_ruby_plugin
ifeq ($(DEP_MISSING),)
all: static shared plugins
dep_error:
	@echo "You shouldn't see this message - all of your dependencies are correct."
else
all: dep_error git_update stop

dep_error:
	@echo
	@echo "DEPENDENCY ERROR"
	@echo
	@echo "You are missing system dependencies that are essential to build grpc,"
	@echo "and the third_party directory doesn't have them:"
	@echo
	@echo "  $(DEP_MISSING)"
	@echo
	@echo "Installing the development packages for your system will solve"
	@echo "this issue. Please consult INSTALL to get more information."
	@echo
	@echo "If you need information about why these tests failed, run:"
	@echo
	@echo "  make run_dep_checks"
	@echo
endif

git_update:
ifeq ($(IS_GIT_FOLDER),true)
	@echo "Additionally, since you are in a git clone, you can download the"
	@echo "missing dependencies in third_party by running the following command:"
	@echo
	@echo "  git submodule update --init"
	@echo
endif

openssl_dep_error: openssl_dep_message git_update stop

protobuf_dep_error: protobuf_dep_message git_update stop

protoc_dep_error: protoc_dep_message git_update stop

openssl_dep_message:
	@echo
	@echo "DEPENDENCY ERROR"
	@echo
	@echo "The target you are trying to run requires OpenSSL."
	@echo "Your system doesn't have it, and neither does the third_party directory."
	@echo
	@echo "Please consult INSTALL to get more information."
	@echo
	@echo "If you need information about why these tests failed, run:"
	@echo
	@echo "  make run_dep_checks"
	@echo

protobuf_dep_message:
	@echo
	@echo "DEPENDENCY ERROR"
	@echo
	@echo "The target you are trying to run requires protobuf 3.0.0+"
	@echo "Your system doesn't have it, and neither does the third_party directory."
	@echo
	@echo "Please consult INSTALL to get more information."
	@echo
	@echo "If you need information about why these tests failed, run:"
	@echo
	@echo "  make run_dep_checks"
	@echo

protoc_dep_message:
	@echo
	@echo "DEPENDENCY ERROR"
	@echo
	@echo "The target you are trying to run requires protobuf-compiler 3.0.0+"
	@echo "Your system doesn't have it, and neither does the third_party directory."
	@echo
	@echo "Please consult INSTALL to get more information."
	@echo
	@echo "If you need information about why these tests failed, run:"
	@echo
	@echo "  make run_dep_checks"
	@echo

systemtap_dep_error:
	@echo
	@echo "DEPENDENCY ERROR"
	@echo
	@echo "Under the '$(CONFIG)' configutation, the target you are trying "
	@echo "to build requires systemtap 2.7+ (on Linux) or dtrace (on other "
	@echo "platforms such as Solaris and *BSD). "
	@echo
	@echo "Please consult INSTALL to get more information."
	@echo

stop:
	@false

algorithm_test: $(BINDIR)/$(CONFIG)/algorithm_test
alloc_test: $(BINDIR)/$(CONFIG)/alloc_test
alpn_test: $(BINDIR)/$(CONFIG)/alpn_test
bin_encoder_test: $(BINDIR)/$(CONFIG)/bin_encoder_test
channel_create_test: $(BINDIR)/$(CONFIG)/channel_create_test
chttp2_hpack_encoder_test: $(BINDIR)/$(CONFIG)/chttp2_hpack_encoder_test
chttp2_status_conversion_test: $(BINDIR)/$(CONFIG)/chttp2_status_conversion_test
chttp2_stream_map_test: $(BINDIR)/$(CONFIG)/chttp2_stream_map_test
chttp2_varint_test: $(BINDIR)/$(CONFIG)/chttp2_varint_test
compression_test: $(BINDIR)/$(CONFIG)/compression_test
dns_resolver_test: $(BINDIR)/$(CONFIG)/dns_resolver_test
dualstack_socket_test: $(BINDIR)/$(CONFIG)/dualstack_socket_test
endpoint_pair_test: $(BINDIR)/$(CONFIG)/endpoint_pair_test
fd_conservation_posix_test: $(BINDIR)/$(CONFIG)/fd_conservation_posix_test
fd_posix_test: $(BINDIR)/$(CONFIG)/fd_posix_test
fling_client: $(BINDIR)/$(CONFIG)/fling_client
fling_server: $(BINDIR)/$(CONFIG)/fling_server
fling_stream_test: $(BINDIR)/$(CONFIG)/fling_stream_test
fling_test: $(BINDIR)/$(CONFIG)/fling_test
gen_hpack_tables: $(BINDIR)/$(CONFIG)/gen_hpack_tables
gen_legal_metadata_characters: $(BINDIR)/$(CONFIG)/gen_legal_metadata_characters
gpr_avl_test: $(BINDIR)/$(CONFIG)/gpr_avl_test
gpr_cmdline_test: $(BINDIR)/$(CONFIG)/gpr_cmdline_test
gpr_cpu_test: $(BINDIR)/$(CONFIG)/gpr_cpu_test
gpr_env_test: $(BINDIR)/$(CONFIG)/gpr_env_test
gpr_file_test: $(BINDIR)/$(CONFIG)/gpr_file_test
gpr_histogram_test: $(BINDIR)/$(CONFIG)/gpr_histogram_test
gpr_host_port_test: $(BINDIR)/$(CONFIG)/gpr_host_port_test
gpr_log_test: $(BINDIR)/$(CONFIG)/gpr_log_test
gpr_slice_buffer_test: $(BINDIR)/$(CONFIG)/gpr_slice_buffer_test
gpr_slice_test: $(BINDIR)/$(CONFIG)/gpr_slice_test
gpr_stack_lockfree_test: $(BINDIR)/$(CONFIG)/gpr_stack_lockfree_test
gpr_string_test: $(BINDIR)/$(CONFIG)/gpr_string_test
gpr_sync_test: $(BINDIR)/$(CONFIG)/gpr_sync_test
gpr_thd_test: $(BINDIR)/$(CONFIG)/gpr_thd_test
gpr_time_test: $(BINDIR)/$(CONFIG)/gpr_time_test
gpr_tls_test: $(BINDIR)/$(CONFIG)/gpr_tls_test
gpr_useful_test: $(BINDIR)/$(CONFIG)/gpr_useful_test
grpc_auth_context_test: $(BINDIR)/$(CONFIG)/grpc_auth_context_test
grpc_base64_test: $(BINDIR)/$(CONFIG)/grpc_base64_test
grpc_byte_buffer_reader_test: $(BINDIR)/$(CONFIG)/grpc_byte_buffer_reader_test
grpc_channel_args_test: $(BINDIR)/$(CONFIG)/grpc_channel_args_test
grpc_channel_stack_test: $(BINDIR)/$(CONFIG)/grpc_channel_stack_test
grpc_completion_queue_test: $(BINDIR)/$(CONFIG)/grpc_completion_queue_test
grpc_create_jwt: $(BINDIR)/$(CONFIG)/grpc_create_jwt
grpc_credentials_test: $(BINDIR)/$(CONFIG)/grpc_credentials_test
grpc_fetch_oauth2: $(BINDIR)/$(CONFIG)/grpc_fetch_oauth2
grpc_invalid_channel_args_test: $(BINDIR)/$(CONFIG)/grpc_invalid_channel_args_test
grpc_json_token_test: $(BINDIR)/$(CONFIG)/grpc_json_token_test
grpc_jwt_verifier_test: $(BINDIR)/$(CONFIG)/grpc_jwt_verifier_test
grpc_print_google_default_creds_token: $(BINDIR)/$(CONFIG)/grpc_print_google_default_creds_token
grpc_security_connector_test: $(BINDIR)/$(CONFIG)/grpc_security_connector_test
grpc_verify_jwt: $(BINDIR)/$(CONFIG)/grpc_verify_jwt
hpack_parser_test: $(BINDIR)/$(CONFIG)/hpack_parser_test
hpack_table_test: $(BINDIR)/$(CONFIG)/hpack_table_test
httpcli_format_request_test: $(BINDIR)/$(CONFIG)/httpcli_format_request_test
httpcli_parser_test: $(BINDIR)/$(CONFIG)/httpcli_parser_test
httpcli_test: $(BINDIR)/$(CONFIG)/httpcli_test
httpscli_test: $(BINDIR)/$(CONFIG)/httpscli_test
init_test: $(BINDIR)/$(CONFIG)/init_test
invalid_call_argument_test: $(BINDIR)/$(CONFIG)/invalid_call_argument_test
json_rewrite: $(BINDIR)/$(CONFIG)/json_rewrite
json_rewrite_test: $(BINDIR)/$(CONFIG)/json_rewrite_test
json_stream_error_test: $(BINDIR)/$(CONFIG)/json_stream_error_test
json_test: $(BINDIR)/$(CONFIG)/json_test
lame_client_test: $(BINDIR)/$(CONFIG)/lame_client_test
lb_policies_test: $(BINDIR)/$(CONFIG)/lb_policies_test
low_level_ping_pong_benchmark: $(BINDIR)/$(CONFIG)/low_level_ping_pong_benchmark
message_compress_test: $(BINDIR)/$(CONFIG)/message_compress_test
multiple_server_queues_test: $(BINDIR)/$(CONFIG)/multiple_server_queues_test
murmur_hash_test: $(BINDIR)/$(CONFIG)/murmur_hash_test
no_server_test: $(BINDIR)/$(CONFIG)/no_server_test
resolve_address_test: $(BINDIR)/$(CONFIG)/resolve_address_test
secure_channel_create_test: $(BINDIR)/$(CONFIG)/secure_channel_create_test
secure_endpoint_test: $(BINDIR)/$(CONFIG)/secure_endpoint_test
server_chttp2_test: $(BINDIR)/$(CONFIG)/server_chttp2_test
server_test: $(BINDIR)/$(CONFIG)/server_test
set_initial_connect_string_test: $(BINDIR)/$(CONFIG)/set_initial_connect_string_test
sockaddr_resolver_test: $(BINDIR)/$(CONFIG)/sockaddr_resolver_test
sockaddr_utils_test: $(BINDIR)/$(CONFIG)/sockaddr_utils_test
socket_utils_test: $(BINDIR)/$(CONFIG)/socket_utils_test
tcp_client_posix_test: $(BINDIR)/$(CONFIG)/tcp_client_posix_test
tcp_posix_test: $(BINDIR)/$(CONFIG)/tcp_posix_test
tcp_server_posix_test: $(BINDIR)/$(CONFIG)/tcp_server_posix_test
time_averaged_stats_test: $(BINDIR)/$(CONFIG)/time_averaged_stats_test
timeout_encoding_test: $(BINDIR)/$(CONFIG)/timeout_encoding_test
timer_heap_test: $(BINDIR)/$(CONFIG)/timer_heap_test
timer_list_test: $(BINDIR)/$(CONFIG)/timer_list_test
timers_test: $(BINDIR)/$(CONFIG)/timers_test
transport_connectivity_state_test: $(BINDIR)/$(CONFIG)/transport_connectivity_state_test
transport_metadata_test: $(BINDIR)/$(CONFIG)/transport_metadata_test
transport_security_test: $(BINDIR)/$(CONFIG)/transport_security_test
udp_server_test: $(BINDIR)/$(CONFIG)/udp_server_test
uri_parser_test: $(BINDIR)/$(CONFIG)/uri_parser_test
workqueue_test: $(BINDIR)/$(CONFIG)/workqueue_test
async_end2end_test: $(BINDIR)/$(CONFIG)/async_end2end_test
async_streaming_ping_pong_test: $(BINDIR)/$(CONFIG)/async_streaming_ping_pong_test
async_unary_ping_pong_test: $(BINDIR)/$(CONFIG)/async_unary_ping_pong_test
auth_property_iterator_test: $(BINDIR)/$(CONFIG)/auth_property_iterator_test
channel_arguments_test: $(BINDIR)/$(CONFIG)/channel_arguments_test
cli_call_test: $(BINDIR)/$(CONFIG)/cli_call_test
client_crash_test: $(BINDIR)/$(CONFIG)/client_crash_test
client_crash_test_server: $(BINDIR)/$(CONFIG)/client_crash_test_server
credentials_test: $(BINDIR)/$(CONFIG)/credentials_test
cxx_byte_buffer_test: $(BINDIR)/$(CONFIG)/cxx_byte_buffer_test
cxx_slice_test: $(BINDIR)/$(CONFIG)/cxx_slice_test
cxx_string_ref_test: $(BINDIR)/$(CONFIG)/cxx_string_ref_test
cxx_time_test: $(BINDIR)/$(CONFIG)/cxx_time_test
end2end_test: $(BINDIR)/$(CONFIG)/end2end_test
generic_end2end_test: $(BINDIR)/$(CONFIG)/generic_end2end_test
grpc_cli: $(BINDIR)/$(CONFIG)/grpc_cli
grpc_cpp_plugin: $(BINDIR)/$(CONFIG)/grpc_cpp_plugin
grpc_csharp_plugin: $(BINDIR)/$(CONFIG)/grpc_csharp_plugin
grpc_objective_c_plugin: $(BINDIR)/$(CONFIG)/grpc_objective_c_plugin
grpc_python_plugin: $(BINDIR)/$(CONFIG)/grpc_python_plugin
grpc_ruby_plugin: $(BINDIR)/$(CONFIG)/grpc_ruby_plugin
interop_client: $(BINDIR)/$(CONFIG)/interop_client
interop_server: $(BINDIR)/$(CONFIG)/interop_server
interop_test: $(BINDIR)/$(CONFIG)/interop_test
metrics_client: $(BINDIR)/$(CONFIG)/metrics_client
mock_test: $(BINDIR)/$(CONFIG)/mock_test
qps_driver: $(BINDIR)/$(CONFIG)/qps_driver
qps_interarrival_test: $(BINDIR)/$(CONFIG)/qps_interarrival_test
qps_openloop_test: $(BINDIR)/$(CONFIG)/qps_openloop_test
qps_test: $(BINDIR)/$(CONFIG)/qps_test
qps_worker: $(BINDIR)/$(CONFIG)/qps_worker
reconnect_interop_client: $(BINDIR)/$(CONFIG)/reconnect_interop_client
reconnect_interop_server: $(BINDIR)/$(CONFIG)/reconnect_interop_server
secure_auth_context_test: $(BINDIR)/$(CONFIG)/secure_auth_context_test
secure_sync_unary_ping_pong_test: $(BINDIR)/$(CONFIG)/secure_sync_unary_ping_pong_test
server_crash_test: $(BINDIR)/$(CONFIG)/server_crash_test
server_crash_test_client: $(BINDIR)/$(CONFIG)/server_crash_test_client
shutdown_test: $(BINDIR)/$(CONFIG)/shutdown_test
status_test: $(BINDIR)/$(CONFIG)/status_test
streaming_throughput_test: $(BINDIR)/$(CONFIG)/streaming_throughput_test
stress_test: $(BINDIR)/$(CONFIG)/stress_test
sync_streaming_ping_pong_test: $(BINDIR)/$(CONFIG)/sync_streaming_ping_pong_test
sync_unary_ping_pong_test: $(BINDIR)/$(CONFIG)/sync_unary_ping_pong_test
thread_stress_test: $(BINDIR)/$(CONFIG)/thread_stress_test
zookeeper_test: $(BINDIR)/$(CONFIG)/zookeeper_test
boringssl_aes_test: $(BINDIR)/$(CONFIG)/boringssl_aes_test
boringssl_base64_test: $(BINDIR)/$(CONFIG)/boringssl_base64_test
boringssl_bio_test: $(BINDIR)/$(CONFIG)/boringssl_bio_test
boringssl_bn_test: $(BINDIR)/$(CONFIG)/boringssl_bn_test
boringssl_bytestring_test: $(BINDIR)/$(CONFIG)/boringssl_bytestring_test
boringssl_aead_test: $(BINDIR)/$(CONFIG)/boringssl_aead_test
boringssl_cipher_test: $(BINDIR)/$(CONFIG)/boringssl_cipher_test
boringssl_cmac_test: $(BINDIR)/$(CONFIG)/boringssl_cmac_test
boringssl_constant_time_test: $(BINDIR)/$(CONFIG)/boringssl_constant_time_test
boringssl_ed25519_test: $(BINDIR)/$(CONFIG)/boringssl_ed25519_test
boringssl_x25519_test: $(BINDIR)/$(CONFIG)/boringssl_x25519_test
boringssl_dh_test: $(BINDIR)/$(CONFIG)/boringssl_dh_test
boringssl_digest_test: $(BINDIR)/$(CONFIG)/boringssl_digest_test
boringssl_dsa_test: $(BINDIR)/$(CONFIG)/boringssl_dsa_test
boringssl_ec_test: $(BINDIR)/$(CONFIG)/boringssl_ec_test
boringssl_example_mul: $(BINDIR)/$(CONFIG)/boringssl_example_mul
boringssl_ecdsa_test: $(BINDIR)/$(CONFIG)/boringssl_ecdsa_test
boringssl_err_test: $(BINDIR)/$(CONFIG)/boringssl_err_test
boringssl_evp_extra_test: $(BINDIR)/$(CONFIG)/boringssl_evp_extra_test
boringssl_evp_test: $(BINDIR)/$(CONFIG)/boringssl_evp_test
boringssl_pbkdf_test: $(BINDIR)/$(CONFIG)/boringssl_pbkdf_test
boringssl_hkdf_test: $(BINDIR)/$(CONFIG)/boringssl_hkdf_test
boringssl_hmac_test: $(BINDIR)/$(CONFIG)/boringssl_hmac_test
boringssl_lhash_test: $(BINDIR)/$(CONFIG)/boringssl_lhash_test
boringssl_gcm_test: $(BINDIR)/$(CONFIG)/boringssl_gcm_test
boringssl_pkcs12_test: $(BINDIR)/$(CONFIG)/boringssl_pkcs12_test
boringssl_pkcs8_test: $(BINDIR)/$(CONFIG)/boringssl_pkcs8_test
boringssl_poly1305_test: $(BINDIR)/$(CONFIG)/boringssl_poly1305_test
boringssl_refcount_test: $(BINDIR)/$(CONFIG)/boringssl_refcount_test
boringssl_rsa_test: $(BINDIR)/$(CONFIG)/boringssl_rsa_test
boringssl_thread_test: $(BINDIR)/$(CONFIG)/boringssl_thread_test
boringssl_pkcs7_test: $(BINDIR)/$(CONFIG)/boringssl_pkcs7_test
boringssl_tab_test: $(BINDIR)/$(CONFIG)/boringssl_tab_test
boringssl_v3name_test: $(BINDIR)/$(CONFIG)/boringssl_v3name_test
boringssl_pqueue_test: $(BINDIR)/$(CONFIG)/boringssl_pqueue_test
boringssl_ssl_test: $(BINDIR)/$(CONFIG)/boringssl_ssl_test
h2_census_test: $(BINDIR)/$(CONFIG)/h2_census_test
h2_compress_test: $(BINDIR)/$(CONFIG)/h2_compress_test
h2_fakesec_test: $(BINDIR)/$(CONFIG)/h2_fakesec_test
h2_full_test: $(BINDIR)/$(CONFIG)/h2_full_test
h2_full+pipe_test: $(BINDIR)/$(CONFIG)/h2_full+pipe_test
h2_full+poll_test: $(BINDIR)/$(CONFIG)/h2_full+poll_test
h2_full+poll+pipe_test: $(BINDIR)/$(CONFIG)/h2_full+poll+pipe_test
h2_oauth2_test: $(BINDIR)/$(CONFIG)/h2_oauth2_test
h2_proxy_test: $(BINDIR)/$(CONFIG)/h2_proxy_test
h2_sockpair_test: $(BINDIR)/$(CONFIG)/h2_sockpair_test
h2_sockpair+trace_test: $(BINDIR)/$(CONFIG)/h2_sockpair+trace_test
h2_sockpair_1byte_test: $(BINDIR)/$(CONFIG)/h2_sockpair_1byte_test
h2_ssl_test: $(BINDIR)/$(CONFIG)/h2_ssl_test
h2_ssl+poll_test: $(BINDIR)/$(CONFIG)/h2_ssl+poll_test
h2_ssl_proxy_test: $(BINDIR)/$(CONFIG)/h2_ssl_proxy_test
h2_uchannel_test: $(BINDIR)/$(CONFIG)/h2_uchannel_test
h2_uds_test: $(BINDIR)/$(CONFIG)/h2_uds_test
h2_uds+poll_test: $(BINDIR)/$(CONFIG)/h2_uds+poll_test
h2_census_nosec_test: $(BINDIR)/$(CONFIG)/h2_census_nosec_test
h2_compress_nosec_test: $(BINDIR)/$(CONFIG)/h2_compress_nosec_test
h2_full_nosec_test: $(BINDIR)/$(CONFIG)/h2_full_nosec_test
h2_full+pipe_nosec_test: $(BINDIR)/$(CONFIG)/h2_full+pipe_nosec_test
h2_full+poll_nosec_test: $(BINDIR)/$(CONFIG)/h2_full+poll_nosec_test
h2_full+poll+pipe_nosec_test: $(BINDIR)/$(CONFIG)/h2_full+poll+pipe_nosec_test
h2_proxy_nosec_test: $(BINDIR)/$(CONFIG)/h2_proxy_nosec_test
h2_sockpair_nosec_test: $(BINDIR)/$(CONFIG)/h2_sockpair_nosec_test
h2_sockpair+trace_nosec_test: $(BINDIR)/$(CONFIG)/h2_sockpair+trace_nosec_test
h2_sockpair_1byte_nosec_test: $(BINDIR)/$(CONFIG)/h2_sockpair_1byte_nosec_test
h2_uchannel_nosec_test: $(BINDIR)/$(CONFIG)/h2_uchannel_nosec_test
h2_uds_nosec_test: $(BINDIR)/$(CONFIG)/h2_uds_nosec_test
h2_uds+poll_nosec_test: $(BINDIR)/$(CONFIG)/h2_uds+poll_nosec_test
badreq_bad_client_test: $(BINDIR)/$(CONFIG)/badreq_bad_client_test
connection_prefix_bad_client_test: $(BINDIR)/$(CONFIG)/connection_prefix_bad_client_test
headers_bad_client_test: $(BINDIR)/$(CONFIG)/headers_bad_client_test
initial_settings_frame_bad_client_test: $(BINDIR)/$(CONFIG)/initial_settings_frame_bad_client_test
server_registered_method_bad_client_test: $(BINDIR)/$(CONFIG)/server_registered_method_bad_client_test
simple_request_bad_client_test: $(BINDIR)/$(CONFIG)/simple_request_bad_client_test
unknown_frame_bad_client_test: $(BINDIR)/$(CONFIG)/unknown_frame_bad_client_test
window_overflow_bad_client_test: $(BINDIR)/$(CONFIG)/window_overflow_bad_client_test
bad_ssl_alpn_server: $(BINDIR)/$(CONFIG)/bad_ssl_alpn_server
bad_ssl_cert_server: $(BINDIR)/$(CONFIG)/bad_ssl_cert_server
bad_ssl_alpn_test: $(BINDIR)/$(CONFIG)/bad_ssl_alpn_test
bad_ssl_cert_test: $(BINDIR)/$(CONFIG)/bad_ssl_cert_test

run_dep_checks:
	$(OPENSSL_ALPN_CHECK_CMD) || true
	$(OPENSSL_NPN_CHECK_CMD) || true
	$(ZLIB_CHECK_CMD) || true
	$(PERFTOOLS_CHECK_CMD) || true
	$(PROTOBUF_CHECK_CMD) || true
	$(PROTOC_CHECK_VERSION_CMD) || true
	$(ZOOKEEPER_CHECK_CMD) || true

$(LIBDIR)/$(CONFIG)/zlib/libz.a:
	$(E) "[MAKE]    Building zlib"
	$(Q)(cd third_party/zlib ; CC="$(CC)" CFLAGS="$(CFLAGS_$(CONFIG)) $(PIC_CPPFLAGS) -fvisibility=hidden $(CPPFLAGS_$(CONFIG)) $(ZLIB_CFLAGS_EXTRA)" ./configure --static)
	$(Q)$(MAKE) -C third_party/zlib clean
	$(Q)$(MAKE) -C third_party/zlib
	$(Q)mkdir -p $(LIBDIR)/$(CONFIG)/zlib
	$(Q)cp third_party/zlib/libz.a $(LIBDIR)/$(CONFIG)/zlib

third_party/protobuf/configure:
	$(E) "[AUTOGEN] Preparing protobuf"
	$(Q)(cd third_party/protobuf ; autoreconf -f -i -Wall,no-obsolete)

$(LIBDIR)/$(CONFIG)/protobuf/libprotobuf.a: third_party/protobuf/configure
	$(E) "[MAKE]    Building protobuf"
	$(Q)(cd third_party/protobuf ; CC="$(CC)" CXX="$(CXX)" LDFLAGS="$(LDFLAGS_$(CONFIG)) -g $(PROTOBUF_LDFLAGS_EXTRA)" CPPFLAGS="$(PIC_CPPFLAGS) $(CPPFLAGS_$(CONFIG)) -g $(PROTOBUF_CPPFLAGS_EXTRA)" ./configure --disable-shared --enable-static)
	$(Q)$(MAKE) -C third_party/protobuf clean
	$(Q)$(MAKE) -C third_party/protobuf
	$(Q)mkdir -p $(LIBDIR)/$(CONFIG)/protobuf
	$(Q)mkdir -p $(BINDIR)/$(CONFIG)/protobuf
	$(Q)cp third_party/protobuf/src/.libs/libprotoc.a $(LIBDIR)/$(CONFIG)/protobuf
	$(Q)cp third_party/protobuf/src/.libs/libprotobuf.a $(LIBDIR)/$(CONFIG)/protobuf
	$(Q)cp third_party/protobuf/src/protoc $(BINDIR)/$(CONFIG)/protobuf

static: static_c static_cxx

static_c: pc_c pc_c_unsecure cache.mk pc_gpr pc_c_zookeeper $(LIBDIR)/$(CONFIG)/libgpr.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.a static_zookeeper_libs


static_cxx: pc_cxx pc_cxx_unsecure pc_gpr cache.mk  $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc++_unsecure.a

shared: shared_c shared_cxx

shared_c: pc_c pc_c_unsecure pc_gpr cache.mk pc_c_zookeeper $(LIBDIR)/$(CONFIG)/libgpr.$(SHARED_EXT) $(LIBDIR)/$(CONFIG)/libgrpc.$(SHARED_EXT) $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.$(SHARED_EXT) shared_zookeeper_libs

shared_cxx: pc_cxx pc_cxx_unsecure cache.mk $(LIBDIR)/$(CONFIG)/libgrpc++.$(SHARED_EXT) $(LIBDIR)/$(CONFIG)/libgrpc++_unsecure.$(SHARED_EXT)

shared_csharp: shared_c  $(LIBDIR)/$(CONFIG)/libgrpc_csharp_ext.$(SHARED_EXT)
ifeq ($(HAS_ZOOKEEPER),true)
static_zookeeper_libs: $(LIBDIR)/$(CONFIG)/libgrpc_zookeeper.a
shared_zookeeper_libs: $(LIBDIR)/$(CONFIG)/libgrpc_zookeeper.$(SHARED_EXT)
else

static_zookeeper_libs:

shared_zookeeper_libs:

endif

grpc_csharp_ext: shared_csharp

plugins: $(PROTOC_PLUGINS)

privatelibs: privatelibs_c privatelibs_cxx

privatelibs_c:  $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util_unsecure.a $(LIBDIR)/$(CONFIG)/libreconnect_server.a $(LIBDIR)/$(CONFIG)/libtest_tcp_server.a $(LIBDIR)/$(CONFIG)/libend2end_tests.a $(LIBDIR)/$(CONFIG)/libend2end_nosec_tests.a $(LIBDIR)/$(CONFIG)/libend2end_certs.a $(LIBDIR)/$(CONFIG)/libbad_client_test.a $(LIBDIR)/$(CONFIG)/libbad_ssl_test_server.a
pc_gpr: $(LIBDIR)/$(CONFIG)/pkgconfig/gpr.pc

pc_c: $(LIBDIR)/$(CONFIG)/pkgconfig/grpc.pc

pc_c_unsecure: $(LIBDIR)/$(CONFIG)/pkgconfig/grpc_unsecure.pc

ifeq ($(HAS_ZOOKEEPER),true)
pc_c_zookeeper: $(LIBDIR)/$(CONFIG)/pkgconfig/grpc_zookeeper.pc
else
pc_c_zookeeper:
endif

pc_cxx: $(LIBDIR)/$(CONFIG)/pkgconfig/grpc++.pc

pc_cxx_unsecure: $(LIBDIR)/$(CONFIG)/pkgconfig/grpc++_unsecure.pc

privatelibs_cxx:  $(LIBDIR)/$(CONFIG)/libgrpc++_test_config.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libinterop_client_helper.a $(LIBDIR)/$(CONFIG)/libinterop_client_main.a $(LIBDIR)/$(CONFIG)/libinterop_server_helper.a $(LIBDIR)/$(CONFIG)/libinterop_server_main.a $(LIBDIR)/$(CONFIG)/libqps.a $(LIBDIR)/$(CONFIG)/libboringssl_test_util.a $(LIBDIR)/$(CONFIG)/libboringssl_aes_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_base64_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_bio_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_bn_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_bytestring_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_aead_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_cipher_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_cmac_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_ed25519_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_x25519_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_dh_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_digest_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_ec_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_ecdsa_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_err_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_evp_extra_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_evp_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_pbkdf_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_hmac_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_pkcs12_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_pkcs8_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_poly1305_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_rsa_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_ssl_test_lib.a

ifeq ($(HAS_ZOOKEEPER),true)
privatelibs_zookeeper: 
else
privatelibs_zookeeper:
endif


buildtests: buildtests_c buildtests_cxx buildtests_zookeeper

buildtests_c: privatelibs_c \
  $(BINDIR)/$(CONFIG)/algorithm_test \
  $(BINDIR)/$(CONFIG)/alloc_test \
  $(BINDIR)/$(CONFIG)/alpn_test \
  $(BINDIR)/$(CONFIG)/bin_encoder_test \
  $(BINDIR)/$(CONFIG)/channel_create_test \
  $(BINDIR)/$(CONFIG)/chttp2_hpack_encoder_test \
  $(BINDIR)/$(CONFIG)/chttp2_status_conversion_test \
  $(BINDIR)/$(CONFIG)/chttp2_stream_map_test \
  $(BINDIR)/$(CONFIG)/chttp2_varint_test \
  $(BINDIR)/$(CONFIG)/compression_test \
  $(BINDIR)/$(CONFIG)/dns_resolver_test \
  $(BINDIR)/$(CONFIG)/dualstack_socket_test \
  $(BINDIR)/$(CONFIG)/endpoint_pair_test \
  $(BINDIR)/$(CONFIG)/fd_conservation_posix_test \
  $(BINDIR)/$(CONFIG)/fd_posix_test \
  $(BINDIR)/$(CONFIG)/fling_client \
  $(BINDIR)/$(CONFIG)/fling_server \
  $(BINDIR)/$(CONFIG)/fling_stream_test \
  $(BINDIR)/$(CONFIG)/fling_test \
  $(BINDIR)/$(CONFIG)/gpr_avl_test \
  $(BINDIR)/$(CONFIG)/gpr_cmdline_test \
  $(BINDIR)/$(CONFIG)/gpr_cpu_test \
  $(BINDIR)/$(CONFIG)/gpr_env_test \
  $(BINDIR)/$(CONFIG)/gpr_file_test \
  $(BINDIR)/$(CONFIG)/gpr_histogram_test \
  $(BINDIR)/$(CONFIG)/gpr_host_port_test \
  $(BINDIR)/$(CONFIG)/gpr_log_test \
  $(BINDIR)/$(CONFIG)/gpr_slice_buffer_test \
  $(BINDIR)/$(CONFIG)/gpr_slice_test \
  $(BINDIR)/$(CONFIG)/gpr_stack_lockfree_test \
  $(BINDIR)/$(CONFIG)/gpr_string_test \
  $(BINDIR)/$(CONFIG)/gpr_sync_test \
  $(BINDIR)/$(CONFIG)/gpr_thd_test \
  $(BINDIR)/$(CONFIG)/gpr_time_test \
  $(BINDIR)/$(CONFIG)/gpr_tls_test \
  $(BINDIR)/$(CONFIG)/gpr_useful_test \
  $(BINDIR)/$(CONFIG)/grpc_auth_context_test \
  $(BINDIR)/$(CONFIG)/grpc_base64_test \
  $(BINDIR)/$(CONFIG)/grpc_byte_buffer_reader_test \
  $(BINDIR)/$(CONFIG)/grpc_channel_args_test \
  $(BINDIR)/$(CONFIG)/grpc_channel_stack_test \
  $(BINDIR)/$(CONFIG)/grpc_completion_queue_test \
  $(BINDIR)/$(CONFIG)/grpc_credentials_test \
  $(BINDIR)/$(CONFIG)/grpc_invalid_channel_args_test \
  $(BINDIR)/$(CONFIG)/grpc_json_token_test \
  $(BINDIR)/$(CONFIG)/grpc_jwt_verifier_test \
  $(BINDIR)/$(CONFIG)/grpc_security_connector_test \
  $(BINDIR)/$(CONFIG)/hpack_parser_test \
  $(BINDIR)/$(CONFIG)/hpack_table_test \
  $(BINDIR)/$(CONFIG)/httpcli_format_request_test \
  $(BINDIR)/$(CONFIG)/httpcli_parser_test \
  $(BINDIR)/$(CONFIG)/httpcli_test \
  $(BINDIR)/$(CONFIG)/httpscli_test \
  $(BINDIR)/$(CONFIG)/init_test \
  $(BINDIR)/$(CONFIG)/invalid_call_argument_test \
  $(BINDIR)/$(CONFIG)/json_rewrite \
  $(BINDIR)/$(CONFIG)/json_rewrite_test \
  $(BINDIR)/$(CONFIG)/json_stream_error_test \
  $(BINDIR)/$(CONFIG)/json_test \
  $(BINDIR)/$(CONFIG)/lame_client_test \
  $(BINDIR)/$(CONFIG)/lb_policies_test \
  $(BINDIR)/$(CONFIG)/message_compress_test \
  $(BINDIR)/$(CONFIG)/multiple_server_queues_test \
  $(BINDIR)/$(CONFIG)/murmur_hash_test \
  $(BINDIR)/$(CONFIG)/no_server_test \
  $(BINDIR)/$(CONFIG)/resolve_address_test \
  $(BINDIR)/$(CONFIG)/secure_channel_create_test \
  $(BINDIR)/$(CONFIG)/secure_endpoint_test \
  $(BINDIR)/$(CONFIG)/server_chttp2_test \
  $(BINDIR)/$(CONFIG)/server_test \
  $(BINDIR)/$(CONFIG)/set_initial_connect_string_test \
  $(BINDIR)/$(CONFIG)/sockaddr_resolver_test \
  $(BINDIR)/$(CONFIG)/sockaddr_utils_test \
  $(BINDIR)/$(CONFIG)/socket_utils_test \
  $(BINDIR)/$(CONFIG)/tcp_client_posix_test \
  $(BINDIR)/$(CONFIG)/tcp_posix_test \
  $(BINDIR)/$(CONFIG)/tcp_server_posix_test \
  $(BINDIR)/$(CONFIG)/time_averaged_stats_test \
  $(BINDIR)/$(CONFIG)/timeout_encoding_test \
  $(BINDIR)/$(CONFIG)/timer_heap_test \
  $(BINDIR)/$(CONFIG)/timer_list_test \
  $(BINDIR)/$(CONFIG)/timers_test \
  $(BINDIR)/$(CONFIG)/transport_connectivity_state_test \
  $(BINDIR)/$(CONFIG)/transport_metadata_test \
  $(BINDIR)/$(CONFIG)/transport_security_test \
  $(BINDIR)/$(CONFIG)/udp_server_test \
  $(BINDIR)/$(CONFIG)/uri_parser_test \
  $(BINDIR)/$(CONFIG)/workqueue_test \
  $(BINDIR)/$(CONFIG)/h2_census_test \
  $(BINDIR)/$(CONFIG)/h2_compress_test \
  $(BINDIR)/$(CONFIG)/h2_fakesec_test \
  $(BINDIR)/$(CONFIG)/h2_full_test \
  $(BINDIR)/$(CONFIG)/h2_full+pipe_test \
  $(BINDIR)/$(CONFIG)/h2_full+poll_test \
  $(BINDIR)/$(CONFIG)/h2_full+poll+pipe_test \
  $(BINDIR)/$(CONFIG)/h2_oauth2_test \
  $(BINDIR)/$(CONFIG)/h2_proxy_test \
  $(BINDIR)/$(CONFIG)/h2_sockpair_test \
  $(BINDIR)/$(CONFIG)/h2_sockpair+trace_test \
  $(BINDIR)/$(CONFIG)/h2_sockpair_1byte_test \
  $(BINDIR)/$(CONFIG)/h2_ssl_test \
  $(BINDIR)/$(CONFIG)/h2_ssl+poll_test \
  $(BINDIR)/$(CONFIG)/h2_ssl_proxy_test \
  $(BINDIR)/$(CONFIG)/h2_uchannel_test \
  $(BINDIR)/$(CONFIG)/h2_uds_test \
  $(BINDIR)/$(CONFIG)/h2_uds+poll_test \
  $(BINDIR)/$(CONFIG)/h2_census_nosec_test \
  $(BINDIR)/$(CONFIG)/h2_compress_nosec_test \
  $(BINDIR)/$(CONFIG)/h2_full_nosec_test \
  $(BINDIR)/$(CONFIG)/h2_full+pipe_nosec_test \
  $(BINDIR)/$(CONFIG)/h2_full+poll_nosec_test \
  $(BINDIR)/$(CONFIG)/h2_full+poll+pipe_nosec_test \
  $(BINDIR)/$(CONFIG)/h2_proxy_nosec_test \
  $(BINDIR)/$(CONFIG)/h2_sockpair_nosec_test \
  $(BINDIR)/$(CONFIG)/h2_sockpair+trace_nosec_test \
  $(BINDIR)/$(CONFIG)/h2_sockpair_1byte_nosec_test \
  $(BINDIR)/$(CONFIG)/h2_uchannel_nosec_test \
  $(BINDIR)/$(CONFIG)/h2_uds_nosec_test \
  $(BINDIR)/$(CONFIG)/h2_uds+poll_nosec_test \
  $(BINDIR)/$(CONFIG)/badreq_bad_client_test \
  $(BINDIR)/$(CONFIG)/connection_prefix_bad_client_test \
  $(BINDIR)/$(CONFIG)/headers_bad_client_test \
  $(BINDIR)/$(CONFIG)/initial_settings_frame_bad_client_test \
  $(BINDIR)/$(CONFIG)/server_registered_method_bad_client_test \
  $(BINDIR)/$(CONFIG)/simple_request_bad_client_test \
  $(BINDIR)/$(CONFIG)/unknown_frame_bad_client_test \
  $(BINDIR)/$(CONFIG)/window_overflow_bad_client_test \
  $(BINDIR)/$(CONFIG)/bad_ssl_alpn_server \
  $(BINDIR)/$(CONFIG)/bad_ssl_cert_server \
  $(BINDIR)/$(CONFIG)/bad_ssl_alpn_test \
  $(BINDIR)/$(CONFIG)/bad_ssl_cert_test \


buildtests_cxx: buildtests_zookeeper privatelibs_cxx \
  $(BINDIR)/$(CONFIG)/async_end2end_test \
  $(BINDIR)/$(CONFIG)/async_streaming_ping_pong_test \
  $(BINDIR)/$(CONFIG)/async_unary_ping_pong_test \
  $(BINDIR)/$(CONFIG)/auth_property_iterator_test \
  $(BINDIR)/$(CONFIG)/channel_arguments_test \
  $(BINDIR)/$(CONFIG)/cli_call_test \
  $(BINDIR)/$(CONFIG)/client_crash_test \
  $(BINDIR)/$(CONFIG)/client_crash_test_server \
  $(BINDIR)/$(CONFIG)/credentials_test \
  $(BINDIR)/$(CONFIG)/cxx_byte_buffer_test \
  $(BINDIR)/$(CONFIG)/cxx_slice_test \
  $(BINDIR)/$(CONFIG)/cxx_string_ref_test \
  $(BINDIR)/$(CONFIG)/cxx_time_test \
  $(BINDIR)/$(CONFIG)/end2end_test \
  $(BINDIR)/$(CONFIG)/generic_end2end_test \
  $(BINDIR)/$(CONFIG)/grpc_cli \
  $(BINDIR)/$(CONFIG)/interop_client \
  $(BINDIR)/$(CONFIG)/interop_server \
  $(BINDIR)/$(CONFIG)/interop_test \
  $(BINDIR)/$(CONFIG)/metrics_client \
  $(BINDIR)/$(CONFIG)/mock_test \
  $(BINDIR)/$(CONFIG)/qps_interarrival_test \
  $(BINDIR)/$(CONFIG)/qps_openloop_test \
  $(BINDIR)/$(CONFIG)/qps_test \
  $(BINDIR)/$(CONFIG)/reconnect_interop_client \
  $(BINDIR)/$(CONFIG)/reconnect_interop_server \
  $(BINDIR)/$(CONFIG)/secure_auth_context_test \
  $(BINDIR)/$(CONFIG)/secure_sync_unary_ping_pong_test \
  $(BINDIR)/$(CONFIG)/server_crash_test \
  $(BINDIR)/$(CONFIG)/server_crash_test_client \
  $(BINDIR)/$(CONFIG)/shutdown_test \
  $(BINDIR)/$(CONFIG)/status_test \
  $(BINDIR)/$(CONFIG)/streaming_throughput_test \
  $(BINDIR)/$(CONFIG)/stress_test \
  $(BINDIR)/$(CONFIG)/sync_streaming_ping_pong_test \
  $(BINDIR)/$(CONFIG)/sync_unary_ping_pong_test \
  $(BINDIR)/$(CONFIG)/thread_stress_test \
  $(BINDIR)/$(CONFIG)/boringssl_aes_test \
  $(BINDIR)/$(CONFIG)/boringssl_base64_test \
  $(BINDIR)/$(CONFIG)/boringssl_bio_test \
  $(BINDIR)/$(CONFIG)/boringssl_bn_test \
  $(BINDIR)/$(CONFIG)/boringssl_bytestring_test \
  $(BINDIR)/$(CONFIG)/boringssl_aead_test \
  $(BINDIR)/$(CONFIG)/boringssl_cipher_test \
  $(BINDIR)/$(CONFIG)/boringssl_cmac_test \
  $(BINDIR)/$(CONFIG)/boringssl_constant_time_test \
  $(BINDIR)/$(CONFIG)/boringssl_ed25519_test \
  $(BINDIR)/$(CONFIG)/boringssl_x25519_test \
  $(BINDIR)/$(CONFIG)/boringssl_dh_test \
  $(BINDIR)/$(CONFIG)/boringssl_digest_test \
  $(BINDIR)/$(CONFIG)/boringssl_dsa_test \
  $(BINDIR)/$(CONFIG)/boringssl_ec_test \
  $(BINDIR)/$(CONFIG)/boringssl_example_mul \
  $(BINDIR)/$(CONFIG)/boringssl_ecdsa_test \
  $(BINDIR)/$(CONFIG)/boringssl_err_test \
  $(BINDIR)/$(CONFIG)/boringssl_evp_extra_test \
  $(BINDIR)/$(CONFIG)/boringssl_evp_test \
  $(BINDIR)/$(CONFIG)/boringssl_pbkdf_test \
  $(BINDIR)/$(CONFIG)/boringssl_hkdf_test \
  $(BINDIR)/$(CONFIG)/boringssl_hmac_test \
  $(BINDIR)/$(CONFIG)/boringssl_lhash_test \
  $(BINDIR)/$(CONFIG)/boringssl_gcm_test \
  $(BINDIR)/$(CONFIG)/boringssl_pkcs12_test \
  $(BINDIR)/$(CONFIG)/boringssl_pkcs8_test \
  $(BINDIR)/$(CONFIG)/boringssl_poly1305_test \
  $(BINDIR)/$(CONFIG)/boringssl_refcount_test \
  $(BINDIR)/$(CONFIG)/boringssl_rsa_test \
  $(BINDIR)/$(CONFIG)/boringssl_thread_test \
  $(BINDIR)/$(CONFIG)/boringssl_pkcs7_test \
  $(BINDIR)/$(CONFIG)/boringssl_tab_test \
  $(BINDIR)/$(CONFIG)/boringssl_v3name_test \
  $(BINDIR)/$(CONFIG)/boringssl_pqueue_test \
  $(BINDIR)/$(CONFIG)/boringssl_ssl_test \


ifeq ($(HAS_ZOOKEEPER),true)
buildtests_zookeeper: privatelibs_zookeeper \
 $(BINDIR)/$(CONFIG)/zookeeper_test \

else
buildtests_zookeeper:
endif


test: test_c test_cxx test_zookeeper

flaky_test: flaky_test_c flaky_test_cxx flaky_test_zookeeper

test_c: buildtests_c
	$(E) "[RUN]     Testing algorithm_test"
	$(Q) $(BINDIR)/$(CONFIG)/algorithm_test || ( echo test algorithm_test failed ; exit 1 )
	$(E) "[RUN]     Testing alloc_test"
	$(Q) $(BINDIR)/$(CONFIG)/alloc_test || ( echo test alloc_test failed ; exit 1 )
	$(E) "[RUN]     Testing alpn_test"
	$(Q) $(BINDIR)/$(CONFIG)/alpn_test || ( echo test alpn_test failed ; exit 1 )
	$(E) "[RUN]     Testing bin_encoder_test"
	$(Q) $(BINDIR)/$(CONFIG)/bin_encoder_test || ( echo test bin_encoder_test failed ; exit 1 )
	$(E) "[RUN]     Testing channel_create_test"
	$(Q) $(BINDIR)/$(CONFIG)/channel_create_test || ( echo test channel_create_test failed ; exit 1 )
	$(E) "[RUN]     Testing chttp2_hpack_encoder_test"
	$(Q) $(BINDIR)/$(CONFIG)/chttp2_hpack_encoder_test || ( echo test chttp2_hpack_encoder_test failed ; exit 1 )
	$(E) "[RUN]     Testing chttp2_status_conversion_test"
	$(Q) $(BINDIR)/$(CONFIG)/chttp2_status_conversion_test || ( echo test chttp2_status_conversion_test failed ; exit 1 )
	$(E) "[RUN]     Testing chttp2_stream_map_test"
	$(Q) $(BINDIR)/$(CONFIG)/chttp2_stream_map_test || ( echo test chttp2_stream_map_test failed ; exit 1 )
	$(E) "[RUN]     Testing chttp2_varint_test"
	$(Q) $(BINDIR)/$(CONFIG)/chttp2_varint_test || ( echo test chttp2_varint_test failed ; exit 1 )
	$(E) "[RUN]     Testing compression_test"
	$(Q) $(BINDIR)/$(CONFIG)/compression_test || ( echo test compression_test failed ; exit 1 )
	$(E) "[RUN]     Testing dns_resolver_test"
	$(Q) $(BINDIR)/$(CONFIG)/dns_resolver_test || ( echo test dns_resolver_test failed ; exit 1 )
	$(E) "[RUN]     Testing dualstack_socket_test"
	$(Q) $(BINDIR)/$(CONFIG)/dualstack_socket_test || ( echo test dualstack_socket_test failed ; exit 1 )
	$(E) "[RUN]     Testing endpoint_pair_test"
	$(Q) $(BINDIR)/$(CONFIG)/endpoint_pair_test || ( echo test endpoint_pair_test failed ; exit 1 )
	$(E) "[RUN]     Testing fd_conservation_posix_test"
	$(Q) $(BINDIR)/$(CONFIG)/fd_conservation_posix_test || ( echo test fd_conservation_posix_test failed ; exit 1 )
	$(E) "[RUN]     Testing fd_posix_test"
	$(Q) $(BINDIR)/$(CONFIG)/fd_posix_test || ( echo test fd_posix_test failed ; exit 1 )
	$(E) "[RUN]     Testing fling_stream_test"
	$(Q) $(BINDIR)/$(CONFIG)/fling_stream_test || ( echo test fling_stream_test failed ; exit 1 )
	$(E) "[RUN]     Testing fling_test"
	$(Q) $(BINDIR)/$(CONFIG)/fling_test || ( echo test fling_test failed ; exit 1 )
	$(E) "[RUN]     Testing gpr_avl_test"
	$(Q) $(BINDIR)/$(CONFIG)/gpr_avl_test || ( echo test gpr_avl_test failed ; exit 1 )
	$(E) "[RUN]     Testing gpr_cmdline_test"
	$(Q) $(BINDIR)/$(CONFIG)/gpr_cmdline_test || ( echo test gpr_cmdline_test failed ; exit 1 )
	$(E) "[RUN]     Testing gpr_cpu_test"
	$(Q) $(BINDIR)/$(CONFIG)/gpr_cpu_test || ( echo test gpr_cpu_test failed ; exit 1 )
	$(E) "[RUN]     Testing gpr_env_test"
	$(Q) $(BINDIR)/$(CONFIG)/gpr_env_test || ( echo test gpr_env_test failed ; exit 1 )
	$(E) "[RUN]     Testing gpr_file_test"
	$(Q) $(BINDIR)/$(CONFIG)/gpr_file_test || ( echo test gpr_file_test failed ; exit 1 )
	$(E) "[RUN]     Testing gpr_histogram_test"
	$(Q) $(BINDIR)/$(CONFIG)/gpr_histogram_test || ( echo test gpr_histogram_test failed ; exit 1 )
	$(E) "[RUN]     Testing gpr_host_port_test"
	$(Q) $(BINDIR)/$(CONFIG)/gpr_host_port_test || ( echo test gpr_host_port_test failed ; exit 1 )
	$(E) "[RUN]     Testing gpr_log_test"
	$(Q) $(BINDIR)/$(CONFIG)/gpr_log_test || ( echo test gpr_log_test failed ; exit 1 )
	$(E) "[RUN]     Testing gpr_slice_buffer_test"
	$(Q) $(BINDIR)/$(CONFIG)/gpr_slice_buffer_test || ( echo test gpr_slice_buffer_test failed ; exit 1 )
	$(E) "[RUN]     Testing gpr_slice_test"
	$(Q) $(BINDIR)/$(CONFIG)/gpr_slice_test || ( echo test gpr_slice_test failed ; exit 1 )
	$(E) "[RUN]     Testing gpr_stack_lockfree_test"
	$(Q) $(BINDIR)/$(CONFIG)/gpr_stack_lockfree_test || ( echo test gpr_stack_lockfree_test failed ; exit 1 )
	$(E) "[RUN]     Testing gpr_string_test"
	$(Q) $(BINDIR)/$(CONFIG)/gpr_string_test || ( echo test gpr_string_test failed ; exit 1 )
	$(E) "[RUN]     Testing gpr_sync_test"
	$(Q) $(BINDIR)/$(CONFIG)/gpr_sync_test || ( echo test gpr_sync_test failed ; exit 1 )
	$(E) "[RUN]     Testing gpr_thd_test"
	$(Q) $(BINDIR)/$(CONFIG)/gpr_thd_test || ( echo test gpr_thd_test failed ; exit 1 )
	$(E) "[RUN]     Testing gpr_time_test"
	$(Q) $(BINDIR)/$(CONFIG)/gpr_time_test || ( echo test gpr_time_test failed ; exit 1 )
	$(E) "[RUN]     Testing gpr_tls_test"
	$(Q) $(BINDIR)/$(CONFIG)/gpr_tls_test || ( echo test gpr_tls_test failed ; exit 1 )
	$(E) "[RUN]     Testing gpr_useful_test"
	$(Q) $(BINDIR)/$(CONFIG)/gpr_useful_test || ( echo test gpr_useful_test failed ; exit 1 )
	$(E) "[RUN]     Testing grpc_auth_context_test"
	$(Q) $(BINDIR)/$(CONFIG)/grpc_auth_context_test || ( echo test grpc_auth_context_test failed ; exit 1 )
	$(E) "[RUN]     Testing grpc_base64_test"
	$(Q) $(BINDIR)/$(CONFIG)/grpc_base64_test || ( echo test grpc_base64_test failed ; exit 1 )
	$(E) "[RUN]     Testing grpc_byte_buffer_reader_test"
	$(Q) $(BINDIR)/$(CONFIG)/grpc_byte_buffer_reader_test || ( echo test grpc_byte_buffer_reader_test failed ; exit 1 )
	$(E) "[RUN]     Testing grpc_channel_args_test"
	$(Q) $(BINDIR)/$(CONFIG)/grpc_channel_args_test || ( echo test grpc_channel_args_test failed ; exit 1 )
	$(E) "[RUN]     Testing grpc_channel_stack_test"
	$(Q) $(BINDIR)/$(CONFIG)/grpc_channel_stack_test || ( echo test grpc_channel_stack_test failed ; exit 1 )
	$(E) "[RUN]     Testing grpc_completion_queue_test"
	$(Q) $(BINDIR)/$(CONFIG)/grpc_completion_queue_test || ( echo test grpc_completion_queue_test failed ; exit 1 )
	$(E) "[RUN]     Testing grpc_credentials_test"
	$(Q) $(BINDIR)/$(CONFIG)/grpc_credentials_test || ( echo test grpc_credentials_test failed ; exit 1 )
	$(E) "[RUN]     Testing grpc_invalid_channel_args_test"
	$(Q) $(BINDIR)/$(CONFIG)/grpc_invalid_channel_args_test || ( echo test grpc_invalid_channel_args_test failed ; exit 1 )
	$(E) "[RUN]     Testing grpc_json_token_test"
	$(Q) $(BINDIR)/$(CONFIG)/grpc_json_token_test || ( echo test grpc_json_token_test failed ; exit 1 )
	$(E) "[RUN]     Testing grpc_jwt_verifier_test"
	$(Q) $(BINDIR)/$(CONFIG)/grpc_jwt_verifier_test || ( echo test grpc_jwt_verifier_test failed ; exit 1 )
	$(E) "[RUN]     Testing grpc_security_connector_test"
	$(Q) $(BINDIR)/$(CONFIG)/grpc_security_connector_test || ( echo test grpc_security_connector_test failed ; exit 1 )
	$(E) "[RUN]     Testing hpack_parser_test"
	$(Q) $(BINDIR)/$(CONFIG)/hpack_parser_test || ( echo test hpack_parser_test failed ; exit 1 )
	$(E) "[RUN]     Testing hpack_table_test"
	$(Q) $(BINDIR)/$(CONFIG)/hpack_table_test || ( echo test hpack_table_test failed ; exit 1 )
	$(E) "[RUN]     Testing httpcli_format_request_test"
	$(Q) $(BINDIR)/$(CONFIG)/httpcli_format_request_test || ( echo test httpcli_format_request_test failed ; exit 1 )
	$(E) "[RUN]     Testing httpcli_parser_test"
	$(Q) $(BINDIR)/$(CONFIG)/httpcli_parser_test || ( echo test httpcli_parser_test failed ; exit 1 )
	$(E) "[RUN]     Testing httpcli_test"
	$(Q) $(BINDIR)/$(CONFIG)/httpcli_test || ( echo test httpcli_test failed ; exit 1 )
	$(E) "[RUN]     Testing httpscli_test"
	$(Q) $(BINDIR)/$(CONFIG)/httpscli_test || ( echo test httpscli_test failed ; exit 1 )
	$(E) "[RUN]     Testing init_test"
	$(Q) $(BINDIR)/$(CONFIG)/init_test || ( echo test init_test failed ; exit 1 )
	$(E) "[RUN]     Testing invalid_call_argument_test"
	$(Q) $(BINDIR)/$(CONFIG)/invalid_call_argument_test || ( echo test invalid_call_argument_test failed ; exit 1 )
	$(E) "[RUN]     Testing json_rewrite_test"
	$(Q) $(BINDIR)/$(CONFIG)/json_rewrite_test || ( echo test json_rewrite_test failed ; exit 1 )
	$(E) "[RUN]     Testing json_stream_error_test"
	$(Q) $(BINDIR)/$(CONFIG)/json_stream_error_test || ( echo test json_stream_error_test failed ; exit 1 )
	$(E) "[RUN]     Testing json_test"
	$(Q) $(BINDIR)/$(CONFIG)/json_test || ( echo test json_test failed ; exit 1 )
	$(E) "[RUN]     Testing lame_client_test"
	$(Q) $(BINDIR)/$(CONFIG)/lame_client_test || ( echo test lame_client_test failed ; exit 1 )
	$(E) "[RUN]     Testing lb_policies_test"
	$(Q) $(BINDIR)/$(CONFIG)/lb_policies_test || ( echo test lb_policies_test failed ; exit 1 )
	$(E) "[RUN]     Testing message_compress_test"
	$(Q) $(BINDIR)/$(CONFIG)/message_compress_test || ( echo test message_compress_test failed ; exit 1 )
	$(E) "[RUN]     Testing multiple_server_queues_test"
	$(Q) $(BINDIR)/$(CONFIG)/multiple_server_queues_test || ( echo test multiple_server_queues_test failed ; exit 1 )
	$(E) "[RUN]     Testing murmur_hash_test"
	$(Q) $(BINDIR)/$(CONFIG)/murmur_hash_test || ( echo test murmur_hash_test failed ; exit 1 )
	$(E) "[RUN]     Testing no_server_test"
	$(Q) $(BINDIR)/$(CONFIG)/no_server_test || ( echo test no_server_test failed ; exit 1 )
	$(E) "[RUN]     Testing resolve_address_test"
	$(Q) $(BINDIR)/$(CONFIG)/resolve_address_test || ( echo test resolve_address_test failed ; exit 1 )
	$(E) "[RUN]     Testing secure_channel_create_test"
	$(Q) $(BINDIR)/$(CONFIG)/secure_channel_create_test || ( echo test secure_channel_create_test failed ; exit 1 )
	$(E) "[RUN]     Testing secure_endpoint_test"
	$(Q) $(BINDIR)/$(CONFIG)/secure_endpoint_test || ( echo test secure_endpoint_test failed ; exit 1 )
	$(E) "[RUN]     Testing server_chttp2_test"
	$(Q) $(BINDIR)/$(CONFIG)/server_chttp2_test || ( echo test server_chttp2_test failed ; exit 1 )
	$(E) "[RUN]     Testing server_test"
	$(Q) $(BINDIR)/$(CONFIG)/server_test || ( echo test server_test failed ; exit 1 )
	$(E) "[RUN]     Testing set_initial_connect_string_test"
	$(Q) $(BINDIR)/$(CONFIG)/set_initial_connect_string_test || ( echo test set_initial_connect_string_test failed ; exit 1 )
	$(E) "[RUN]     Testing sockaddr_resolver_test"
	$(Q) $(BINDIR)/$(CONFIG)/sockaddr_resolver_test || ( echo test sockaddr_resolver_test failed ; exit 1 )
	$(E) "[RUN]     Testing sockaddr_utils_test"
	$(Q) $(BINDIR)/$(CONFIG)/sockaddr_utils_test || ( echo test sockaddr_utils_test failed ; exit 1 )
	$(E) "[RUN]     Testing socket_utils_test"
	$(Q) $(BINDIR)/$(CONFIG)/socket_utils_test || ( echo test socket_utils_test failed ; exit 1 )
	$(E) "[RUN]     Testing tcp_client_posix_test"
	$(Q) $(BINDIR)/$(CONFIG)/tcp_client_posix_test || ( echo test tcp_client_posix_test failed ; exit 1 )
	$(E) "[RUN]     Testing tcp_posix_test"
	$(Q) $(BINDIR)/$(CONFIG)/tcp_posix_test || ( echo test tcp_posix_test failed ; exit 1 )
	$(E) "[RUN]     Testing tcp_server_posix_test"
	$(Q) $(BINDIR)/$(CONFIG)/tcp_server_posix_test || ( echo test tcp_server_posix_test failed ; exit 1 )
	$(E) "[RUN]     Testing time_averaged_stats_test"
	$(Q) $(BINDIR)/$(CONFIG)/time_averaged_stats_test || ( echo test time_averaged_stats_test failed ; exit 1 )
	$(E) "[RUN]     Testing timeout_encoding_test"
	$(Q) $(BINDIR)/$(CONFIG)/timeout_encoding_test || ( echo test timeout_encoding_test failed ; exit 1 )
	$(E) "[RUN]     Testing timer_heap_test"
	$(Q) $(BINDIR)/$(CONFIG)/timer_heap_test || ( echo test timer_heap_test failed ; exit 1 )
	$(E) "[RUN]     Testing timer_list_test"
	$(Q) $(BINDIR)/$(CONFIG)/timer_list_test || ( echo test timer_list_test failed ; exit 1 )
	$(E) "[RUN]     Testing timers_test"
	$(Q) $(BINDIR)/$(CONFIG)/timers_test || ( echo test timers_test failed ; exit 1 )
	$(E) "[RUN]     Testing transport_connectivity_state_test"
	$(Q) $(BINDIR)/$(CONFIG)/transport_connectivity_state_test || ( echo test transport_connectivity_state_test failed ; exit 1 )
	$(E) "[RUN]     Testing transport_metadata_test"
	$(Q) $(BINDIR)/$(CONFIG)/transport_metadata_test || ( echo test transport_metadata_test failed ; exit 1 )
	$(E) "[RUN]     Testing transport_security_test"
	$(Q) $(BINDIR)/$(CONFIG)/transport_security_test || ( echo test transport_security_test failed ; exit 1 )
	$(E) "[RUN]     Testing udp_server_test"
	$(Q) $(BINDIR)/$(CONFIG)/udp_server_test || ( echo test udp_server_test failed ; exit 1 )
	$(E) "[RUN]     Testing uri_parser_test"
	$(Q) $(BINDIR)/$(CONFIG)/uri_parser_test || ( echo test uri_parser_test failed ; exit 1 )
	$(E) "[RUN]     Testing workqueue_test"
	$(Q) $(BINDIR)/$(CONFIG)/workqueue_test || ( echo test workqueue_test failed ; exit 1 )
	$(E) "[RUN]     Testing badreq_bad_client_test"
	$(Q) $(BINDIR)/$(CONFIG)/badreq_bad_client_test || ( echo test badreq_bad_client_test failed ; exit 1 )
	$(E) "[RUN]     Testing connection_prefix_bad_client_test"
	$(Q) $(BINDIR)/$(CONFIG)/connection_prefix_bad_client_test || ( echo test connection_prefix_bad_client_test failed ; exit 1 )
	$(E) "[RUN]     Testing headers_bad_client_test"
	$(Q) $(BINDIR)/$(CONFIG)/headers_bad_client_test || ( echo test headers_bad_client_test failed ; exit 1 )
	$(E) "[RUN]     Testing initial_settings_frame_bad_client_test"
	$(Q) $(BINDIR)/$(CONFIG)/initial_settings_frame_bad_client_test || ( echo test initial_settings_frame_bad_client_test failed ; exit 1 )
	$(E) "[RUN]     Testing server_registered_method_bad_client_test"
	$(Q) $(BINDIR)/$(CONFIG)/server_registered_method_bad_client_test || ( echo test server_registered_method_bad_client_test failed ; exit 1 )
	$(E) "[RUN]     Testing simple_request_bad_client_test"
	$(Q) $(BINDIR)/$(CONFIG)/simple_request_bad_client_test || ( echo test simple_request_bad_client_test failed ; exit 1 )
	$(E) "[RUN]     Testing unknown_frame_bad_client_test"
	$(Q) $(BINDIR)/$(CONFIG)/unknown_frame_bad_client_test || ( echo test unknown_frame_bad_client_test failed ; exit 1 )
	$(E) "[RUN]     Testing window_overflow_bad_client_test"
	$(Q) $(BINDIR)/$(CONFIG)/window_overflow_bad_client_test || ( echo test window_overflow_bad_client_test failed ; exit 1 )
	$(E) "[RUN]     Testing bad_ssl_alpn_test"
	$(Q) $(BINDIR)/$(CONFIG)/bad_ssl_alpn_test || ( echo test bad_ssl_alpn_test failed ; exit 1 )
	$(E) "[RUN]     Testing bad_ssl_cert_test"
	$(Q) $(BINDIR)/$(CONFIG)/bad_ssl_cert_test || ( echo test bad_ssl_cert_test failed ; exit 1 )


flaky_test_c: buildtests_c


test_cxx: test_zookeeper buildtests_cxx
	$(E) "[RUN]     Testing async_end2end_test"
	$(Q) $(BINDIR)/$(CONFIG)/async_end2end_test || ( echo test async_end2end_test failed ; exit 1 )
	$(E) "[RUN]     Testing async_streaming_ping_pong_test"
	$(Q) $(BINDIR)/$(CONFIG)/async_streaming_ping_pong_test || ( echo test async_streaming_ping_pong_test failed ; exit 1 )
	$(E) "[RUN]     Testing async_unary_ping_pong_test"
	$(Q) $(BINDIR)/$(CONFIG)/async_unary_ping_pong_test || ( echo test async_unary_ping_pong_test failed ; exit 1 )
	$(E) "[RUN]     Testing auth_property_iterator_test"
	$(Q) $(BINDIR)/$(CONFIG)/auth_property_iterator_test || ( echo test auth_property_iterator_test failed ; exit 1 )
	$(E) "[RUN]     Testing channel_arguments_test"
	$(Q) $(BINDIR)/$(CONFIG)/channel_arguments_test || ( echo test channel_arguments_test failed ; exit 1 )
	$(E) "[RUN]     Testing cli_call_test"
	$(Q) $(BINDIR)/$(CONFIG)/cli_call_test || ( echo test cli_call_test failed ; exit 1 )
	$(E) "[RUN]     Testing client_crash_test"
	$(Q) $(BINDIR)/$(CONFIG)/client_crash_test || ( echo test client_crash_test failed ; exit 1 )
	$(E) "[RUN]     Testing credentials_test"
	$(Q) $(BINDIR)/$(CONFIG)/credentials_test || ( echo test credentials_test failed ; exit 1 )
	$(E) "[RUN]     Testing cxx_byte_buffer_test"
	$(Q) $(BINDIR)/$(CONFIG)/cxx_byte_buffer_test || ( echo test cxx_byte_buffer_test failed ; exit 1 )
	$(E) "[RUN]     Testing cxx_slice_test"
	$(Q) $(BINDIR)/$(CONFIG)/cxx_slice_test || ( echo test cxx_slice_test failed ; exit 1 )
	$(E) "[RUN]     Testing cxx_string_ref_test"
	$(Q) $(BINDIR)/$(CONFIG)/cxx_string_ref_test || ( echo test cxx_string_ref_test failed ; exit 1 )
	$(E) "[RUN]     Testing cxx_time_test"
	$(Q) $(BINDIR)/$(CONFIG)/cxx_time_test || ( echo test cxx_time_test failed ; exit 1 )
	$(E) "[RUN]     Testing end2end_test"
	$(Q) $(BINDIR)/$(CONFIG)/end2end_test || ( echo test end2end_test failed ; exit 1 )
	$(E) "[RUN]     Testing generic_end2end_test"
	$(Q) $(BINDIR)/$(CONFIG)/generic_end2end_test || ( echo test generic_end2end_test failed ; exit 1 )
	$(E) "[RUN]     Testing interop_test"
	$(Q) $(BINDIR)/$(CONFIG)/interop_test || ( echo test interop_test failed ; exit 1 )
	$(E) "[RUN]     Testing mock_test"
	$(Q) $(BINDIR)/$(CONFIG)/mock_test || ( echo test mock_test failed ; exit 1 )
	$(E) "[RUN]     Testing qps_test"
	$(Q) $(BINDIR)/$(CONFIG)/qps_test || ( echo test qps_test failed ; exit 1 )
	$(E) "[RUN]     Testing secure_auth_context_test"
	$(Q) $(BINDIR)/$(CONFIG)/secure_auth_context_test || ( echo test secure_auth_context_test failed ; exit 1 )
	$(E) "[RUN]     Testing secure_sync_unary_ping_pong_test"
	$(Q) $(BINDIR)/$(CONFIG)/secure_sync_unary_ping_pong_test || ( echo test secure_sync_unary_ping_pong_test failed ; exit 1 )
	$(E) "[RUN]     Testing server_crash_test"
	$(Q) $(BINDIR)/$(CONFIG)/server_crash_test || ( echo test server_crash_test failed ; exit 1 )
	$(E) "[RUN]     Testing shutdown_test"
	$(Q) $(BINDIR)/$(CONFIG)/shutdown_test || ( echo test shutdown_test failed ; exit 1 )
	$(E) "[RUN]     Testing status_test"
	$(Q) $(BINDIR)/$(CONFIG)/status_test || ( echo test status_test failed ; exit 1 )
	$(E) "[RUN]     Testing streaming_throughput_test"
	$(Q) $(BINDIR)/$(CONFIG)/streaming_throughput_test || ( echo test streaming_throughput_test failed ; exit 1 )
	$(E) "[RUN]     Testing sync_streaming_ping_pong_test"
	$(Q) $(BINDIR)/$(CONFIG)/sync_streaming_ping_pong_test || ( echo test sync_streaming_ping_pong_test failed ; exit 1 )
	$(E) "[RUN]     Testing sync_unary_ping_pong_test"
	$(Q) $(BINDIR)/$(CONFIG)/sync_unary_ping_pong_test || ( echo test sync_unary_ping_pong_test failed ; exit 1 )
	$(E) "[RUN]     Testing thread_stress_test"
	$(Q) $(BINDIR)/$(CONFIG)/thread_stress_test || ( echo test thread_stress_test failed ; exit 1 )


flaky_test_cxx: buildtests_cxx


ifeq ($(HAS_ZOOKEEPER),true)
test_zookeeper: buildtests_zookeeper


flaky_test_zookeeper: buildtests_zookeeper

else
test_zookeeper:
flaky_test_zookeeper:
endif


test_python: static_c
	$(E) "[RUN]     Testing python code"
	$(Q) tools/run_tests/run_tests.py -lpython -c$(CONFIG)


tools: tools_c tools_cxx


tools_c: privatelibs_c $(BINDIR)/$(CONFIG)/gen_hpack_tables $(BINDIR)/$(CONFIG)/gen_legal_metadata_characters $(BINDIR)/$(CONFIG)/grpc_create_jwt $(BINDIR)/$(CONFIG)/grpc_fetch_oauth2 $(BINDIR)/$(CONFIG)/grpc_print_google_default_creds_token $(BINDIR)/$(CONFIG)/grpc_verify_jwt

tools_cxx: privatelibs_cxx

buildbenchmarks: privatelibs $(BINDIR)/$(CONFIG)/low_level_ping_pong_benchmark $(BINDIR)/$(CONFIG)/qps_driver $(BINDIR)/$(CONFIG)/qps_worker

benchmarks: buildbenchmarks

strip: strip-static strip-shared

strip-static: strip-static_c strip-static_cxx

strip-shared: strip-shared_c strip-shared_cxx


# TODO(nnoble): the strip target is stripping in-place, instead
# of copying files in a temporary folder.
# This prevents proper debugging after running make install.

strip-static_c: static_c
ifeq ($(CONFIG),opt)
	$(E) "[STRIP]   Stripping libgpr.a"
	$(Q) $(STRIP) $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[STRIP]   Stripping libgrpc.a"
	$(Q) $(STRIP) $(LIBDIR)/$(CONFIG)/libgrpc.a
	$(E) "[STRIP]   Stripping libgrpc_unsecure.a"
	$(Q) $(STRIP) $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.a
ifeq ($(HAS_ZOOKEEPER),true)
	$(E) "[STRIP]   Stripping libgrpc_zookeeper.a"
	$(Q) $(STRIP) $(LIBDIR)/$(CONFIG)/libgrpc_zookeeper.a
endif
endif

strip-static_cxx: static_cxx
ifeq ($(CONFIG),opt)
	$(E) "[STRIP]   Stripping libgrpc++.a"
	$(Q) $(STRIP) $(LIBDIR)/$(CONFIG)/libgrpc++.a
	$(E) "[STRIP]   Stripping libgrpc++_unsecure.a"
	$(Q) $(STRIP) $(LIBDIR)/$(CONFIG)/libgrpc++_unsecure.a
endif

strip-shared_c: shared_c
ifeq ($(CONFIG),opt)
	$(E) "[STRIP]   Stripping libgpr.so"
	$(Q) $(STRIP) $(LIBDIR)/$(CONFIG)/libgpr.$(SHARED_EXT)
	$(E) "[STRIP]   Stripping libgrpc.so"
	$(Q) $(STRIP) $(LIBDIR)/$(CONFIG)/libgrpc.$(SHARED_EXT)
	$(E) "[STRIP]   Stripping libgrpc_unsecure.so"
	$(Q) $(STRIP) $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.$(SHARED_EXT)
ifeq ($(HAS_ZOOKEEPER),true)
	$(E) "[STRIP]   Stripping libgrpc_zookeeper.so"
	$(Q) $(STRIP) $(LIBDIR)/$(CONFIG)/libgrpc_zookeeper.$(SHARED_EXT)
endif
endif

strip-shared_cxx: shared_cxx
ifeq ($(CONFIG),opt)
	$(E) "[STRIP]   Stripping libgrpc++.so"
	$(Q) $(STRIP) $(LIBDIR)/$(CONFIG)/libgrpc++.$(SHARED_EXT)
	$(E) "[STRIP]   Stripping libgrpc++_unsecure.so"
	$(Q) $(STRIP) $(LIBDIR)/$(CONFIG)/libgrpc++_unsecure.$(SHARED_EXT)
endif

strip-shared_csharp: shared_csharp
ifeq ($(CONFIG),opt)
	$(E) "[STRIP]   Stripping libgrpc_csharp_ext.so"
	$(Q) $(STRIP) $(LIBDIR)/$(CONFIG)/libgrpc_csharp_ext.$(SHARED_EXT)
endif

cache.mk::
	$(E) "[MAKE]    Generating $@"
	$(Q) echo "$(CACHE_MK)" | tr , '\n' >$@

$(LIBDIR)/$(CONFIG)/pkgconfig/gpr.pc:
	$(E) "[MAKE]    Generating $@"
	$(Q) mkdir -p $(@D)
	$(Q) echo "$(GPR_PC_FILE)" | tr , '\n' >$@

$(LIBDIR)/$(CONFIG)/pkgconfig/grpc.pc:
	$(E) "[MAKE]    Generating $@"
	$(Q) mkdir -p $(@D)
	$(Q) echo "$(GRPC_PC_FILE)" | tr , '\n' >$@

$(LIBDIR)/$(CONFIG)/pkgconfig/grpc_unsecure.pc:
	$(E) "[MAKE]    Generating $@"
	$(Q) mkdir -p $(@D)
	$(Q) echo "$(GRPC_UNSECURE_PC_FILE)" | tr , '\n' >$@

$(LIBDIR)/$(CONFIG)/pkgconfig/grpc_zookeeper.pc:
	$(E) "[MAKE]    Generating $@"
	$(Q) mkdir -p $(@D)
	$(Q) echo -e "$(GRPC_ZOOKEEPER_PC_FILE)" >$@

$(LIBDIR)/$(CONFIG)/pkgconfig/grpc++.pc:
	$(E) "[MAKE]    Generating $@"
	$(Q) mkdir -p $(@D)
	$(Q) echo "$(GRPCXX_PC_FILE)" | tr , '\n' >$@

$(LIBDIR)/$(CONFIG)/pkgconfig/grpc++_unsecure.pc:
	$(E) "[MAKE]    Generating $@"
	$(Q) mkdir -p $(@D)
	$(Q) echo "$(GRPCXX_UNSECURE_PC_FILE)" | tr , '\n' >$@

ifeq ($(NO_PROTOC),true)
$(GENDIR)/test/cpp/qps/perf_db.pb.cc: protoc_dep_error
$(GENDIR)/test/cpp/qps/perf_db.grpc.pb.cc: protoc_dep_error
else
$(GENDIR)/test/cpp/qps/perf_db.pb.cc: test/cpp/qps/perf_db.proto $(PROTOBUF_DEP) $(PROTOC_PLUGINS)
	$(E) "[PROTOC]  Generating protobuf CC file from $<"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(PROTOC) --cpp_out=$(GENDIR) $<

$(GENDIR)/test/cpp/qps/perf_db.grpc.pb.cc: test/cpp/qps/perf_db.proto $(PROTOBUF_DEP) $(PROTOC_PLUGINS)
	$(E) "[GRPC]    Generating gRPC's protobuf service CC file from $<"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(PROTOC) --grpc_out=$(GENDIR) --plugin=protoc-gen-grpc=$(BINDIR)/$(CONFIG)/grpc_cpp_plugin $<
endif

ifeq ($(NO_PROTOC),true)
$(GENDIR)/test/cpp/util/echo.pb.cc: protoc_dep_error
$(GENDIR)/test/cpp/util/echo.grpc.pb.cc: protoc_dep_error
else
$(GENDIR)/test/cpp/util/echo.pb.cc: test/cpp/util/echo.proto $(PROTOBUF_DEP) $(PROTOC_PLUGINS)
	$(E) "[PROTOC]  Generating protobuf CC file from $<"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(PROTOC) --cpp_out=$(GENDIR) $<

$(GENDIR)/test/cpp/util/echo.grpc.pb.cc: test/cpp/util/echo.proto $(PROTOBUF_DEP) $(PROTOC_PLUGINS)
	$(E) "[GRPC]    Generating gRPC's protobuf service CC file from $<"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(PROTOC) --grpc_out=$(GENDIR) --plugin=protoc-gen-grpc=$(BINDIR)/$(CONFIG)/grpc_cpp_plugin $<
endif

ifeq ($(NO_PROTOC),true)
$(GENDIR)/test/cpp/util/echo_duplicate.pb.cc: protoc_dep_error
$(GENDIR)/test/cpp/util/echo_duplicate.grpc.pb.cc: protoc_dep_error
else
$(GENDIR)/test/cpp/util/echo_duplicate.pb.cc: test/cpp/util/echo_duplicate.proto $(PROTOBUF_DEP) $(PROTOC_PLUGINS)
	$(E) "[PROTOC]  Generating protobuf CC file from $<"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(PROTOC) --cpp_out=$(GENDIR) $<

$(GENDIR)/test/cpp/util/echo_duplicate.grpc.pb.cc: test/cpp/util/echo_duplicate.proto $(PROTOBUF_DEP) $(PROTOC_PLUGINS)
	$(E) "[GRPC]    Generating gRPC's protobuf service CC file from $<"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(PROTOC) --grpc_out=$(GENDIR) --plugin=protoc-gen-grpc=$(BINDIR)/$(CONFIG)/grpc_cpp_plugin $<
endif

ifeq ($(NO_PROTOC),true)
$(GENDIR)/test/cpp/util/messages.pb.cc: protoc_dep_error
$(GENDIR)/test/cpp/util/messages.grpc.pb.cc: protoc_dep_error
else
$(GENDIR)/test/cpp/util/messages.pb.cc: test/cpp/util/messages.proto $(PROTOBUF_DEP) $(PROTOC_PLUGINS)
	$(E) "[PROTOC]  Generating protobuf CC file from $<"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(PROTOC) --cpp_out=$(GENDIR) $<

$(GENDIR)/test/cpp/util/messages.grpc.pb.cc: test/cpp/util/messages.proto $(PROTOBUF_DEP) $(PROTOC_PLUGINS)
	$(E) "[GRPC]    Generating gRPC's protobuf service CC file from $<"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(PROTOC) --grpc_out=$(GENDIR) --plugin=protoc-gen-grpc=$(BINDIR)/$(CONFIG)/grpc_cpp_plugin $<
endif

ifeq ($(NO_PROTOC),true)
$(GENDIR)/test/proto/benchmarks/control.pb.cc: protoc_dep_error
$(GENDIR)/test/proto/benchmarks/control.grpc.pb.cc: protoc_dep_error
else
$(GENDIR)/test/proto/benchmarks/control.pb.cc: test/proto/benchmarks/control.proto $(PROTOBUF_DEP) $(PROTOC_PLUGINS)
	$(E) "[PROTOC]  Generating protobuf CC file from $<"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(PROTOC) --cpp_out=$(GENDIR) $<

$(GENDIR)/test/proto/benchmarks/control.grpc.pb.cc: test/proto/benchmarks/control.proto $(PROTOBUF_DEP) $(PROTOC_PLUGINS)
	$(E) "[GRPC]    Generating gRPC's protobuf service CC file from $<"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(PROTOC) --grpc_out=$(GENDIR) --plugin=protoc-gen-grpc=$(BINDIR)/$(CONFIG)/grpc_cpp_plugin $<
endif

ifeq ($(NO_PROTOC),true)
$(GENDIR)/test/proto/benchmarks/payloads.pb.cc: protoc_dep_error
$(GENDIR)/test/proto/benchmarks/payloads.grpc.pb.cc: protoc_dep_error
else
$(GENDIR)/test/proto/benchmarks/payloads.pb.cc: test/proto/benchmarks/payloads.proto $(PROTOBUF_DEP) $(PROTOC_PLUGINS)
	$(E) "[PROTOC]  Generating protobuf CC file from $<"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(PROTOC) --cpp_out=$(GENDIR) $<

$(GENDIR)/test/proto/benchmarks/payloads.grpc.pb.cc: test/proto/benchmarks/payloads.proto $(PROTOBUF_DEP) $(PROTOC_PLUGINS)
	$(E) "[GRPC]    Generating gRPC's protobuf service CC file from $<"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(PROTOC) --grpc_out=$(GENDIR) --plugin=protoc-gen-grpc=$(BINDIR)/$(CONFIG)/grpc_cpp_plugin $<
endif

ifeq ($(NO_PROTOC),true)
$(GENDIR)/test/proto/benchmarks/services.pb.cc: protoc_dep_error
$(GENDIR)/test/proto/benchmarks/services.grpc.pb.cc: protoc_dep_error
else
$(GENDIR)/test/proto/benchmarks/services.pb.cc: test/proto/benchmarks/services.proto $(PROTOBUF_DEP) $(PROTOC_PLUGINS)
	$(E) "[PROTOC]  Generating protobuf CC file from $<"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(PROTOC) --cpp_out=$(GENDIR) $<

$(GENDIR)/test/proto/benchmarks/services.grpc.pb.cc: test/proto/benchmarks/services.proto $(PROTOBUF_DEP) $(PROTOC_PLUGINS)
	$(E) "[GRPC]    Generating gRPC's protobuf service CC file from $<"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(PROTOC) --grpc_out=$(GENDIR) --plugin=protoc-gen-grpc=$(BINDIR)/$(CONFIG)/grpc_cpp_plugin $<
endif

ifeq ($(NO_PROTOC),true)
$(GENDIR)/test/proto/benchmarks/stats.pb.cc: protoc_dep_error
$(GENDIR)/test/proto/benchmarks/stats.grpc.pb.cc: protoc_dep_error
else
$(GENDIR)/test/proto/benchmarks/stats.pb.cc: test/proto/benchmarks/stats.proto $(PROTOBUF_DEP) $(PROTOC_PLUGINS)
	$(E) "[PROTOC]  Generating protobuf CC file from $<"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(PROTOC) --cpp_out=$(GENDIR) $<

$(GENDIR)/test/proto/benchmarks/stats.grpc.pb.cc: test/proto/benchmarks/stats.proto $(PROTOBUF_DEP) $(PROTOC_PLUGINS)
	$(E) "[GRPC]    Generating gRPC's protobuf service CC file from $<"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(PROTOC) --grpc_out=$(GENDIR) --plugin=protoc-gen-grpc=$(BINDIR)/$(CONFIG)/grpc_cpp_plugin $<
endif

ifeq ($(NO_PROTOC),true)
$(GENDIR)/test/proto/empty.pb.cc: protoc_dep_error
$(GENDIR)/test/proto/empty.grpc.pb.cc: protoc_dep_error
else
$(GENDIR)/test/proto/empty.pb.cc: test/proto/empty.proto $(PROTOBUF_DEP) $(PROTOC_PLUGINS)
	$(E) "[PROTOC]  Generating protobuf CC file from $<"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(PROTOC) --cpp_out=$(GENDIR) $<

$(GENDIR)/test/proto/empty.grpc.pb.cc: test/proto/empty.proto $(PROTOBUF_DEP) $(PROTOC_PLUGINS)
	$(E) "[GRPC]    Generating gRPC's protobuf service CC file from $<"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(PROTOC) --grpc_out=$(GENDIR) --plugin=protoc-gen-grpc=$(BINDIR)/$(CONFIG)/grpc_cpp_plugin $<
endif

ifeq ($(NO_PROTOC),true)
$(GENDIR)/test/proto/messages.pb.cc: protoc_dep_error
$(GENDIR)/test/proto/messages.grpc.pb.cc: protoc_dep_error
else
$(GENDIR)/test/proto/messages.pb.cc: test/proto/messages.proto $(PROTOBUF_DEP) $(PROTOC_PLUGINS)
	$(E) "[PROTOC]  Generating protobuf CC file from $<"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(PROTOC) --cpp_out=$(GENDIR) $<

$(GENDIR)/test/proto/messages.grpc.pb.cc: test/proto/messages.proto $(PROTOBUF_DEP) $(PROTOC_PLUGINS)
	$(E) "[GRPC]    Generating gRPC's protobuf service CC file from $<"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(PROTOC) --grpc_out=$(GENDIR) --plugin=protoc-gen-grpc=$(BINDIR)/$(CONFIG)/grpc_cpp_plugin $<
endif

ifeq ($(NO_PROTOC),true)
$(GENDIR)/test/proto/metrics.pb.cc: protoc_dep_error
$(GENDIR)/test/proto/metrics.grpc.pb.cc: protoc_dep_error
else
$(GENDIR)/test/proto/metrics.pb.cc: test/proto/metrics.proto $(PROTOBUF_DEP) $(PROTOC_PLUGINS)
	$(E) "[PROTOC]  Generating protobuf CC file from $<"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(PROTOC) --cpp_out=$(GENDIR) $<

$(GENDIR)/test/proto/metrics.grpc.pb.cc: test/proto/metrics.proto $(PROTOBUF_DEP) $(PROTOC_PLUGINS)
	$(E) "[GRPC]    Generating gRPC's protobuf service CC file from $<"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(PROTOC) --grpc_out=$(GENDIR) --plugin=protoc-gen-grpc=$(BINDIR)/$(CONFIG)/grpc_cpp_plugin $<
endif

ifeq ($(NO_PROTOC),true)
$(GENDIR)/test/proto/test.pb.cc: protoc_dep_error
$(GENDIR)/test/proto/test.grpc.pb.cc: protoc_dep_error
else
$(GENDIR)/test/proto/test.pb.cc: test/proto/test.proto $(PROTOBUF_DEP) $(PROTOC_PLUGINS)
	$(E) "[PROTOC]  Generating protobuf CC file from $<"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(PROTOC) --cpp_out=$(GENDIR) $<

$(GENDIR)/test/proto/test.grpc.pb.cc: test/proto/test.proto $(PROTOBUF_DEP) $(PROTOC_PLUGINS)
	$(E) "[GRPC]    Generating gRPC's protobuf service CC file from $<"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(PROTOC) --grpc_out=$(GENDIR) --plugin=protoc-gen-grpc=$(BINDIR)/$(CONFIG)/grpc_cpp_plugin $<
endif


ifeq ($(CONFIG),stapprof)
src/core/profiling/stap_timers.c: $(GENDIR)/src/core/profiling/stap_probes.h
ifeq ($(HAS_SYSTEMTAP),true)
$(GENDIR)/src/core/profiling/stap_probes.h: src/core/profiling/stap_probes.d
	$(E) "[DTRACE]  Compiling $<"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(DTRACE) -C -h -s $< -o $@
else
$(GENDIR)/src/core/profiling/stap_probes.h: systemtap_dep_error stop
endif
endif

$(OBJDIR)/$(CONFIG)/%.o : %.c
	$(E) "[C]       Compiling $<"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(CC) $(CFLAGS) $(CPPFLAGS) -MMD -MF $(addsuffix .dep, $(basename $@)) -c -o $@ $<

$(OBJDIR)/$(CONFIG)/%.o : $(GENDIR)/%.pb.cc
	$(E) "[CXX]     Compiling $<"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(CXX) $(CXXFLAGS) $(CPPFLAGS) -MMD -MF $(addsuffix .dep, $(basename $@)) -c -o $@ $<

$(OBJDIR)/$(CONFIG)/src/compiler/%.o : src/compiler/%.cc
	$(E) "[HOSTCXX] Compiling $<"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(HOST_CXX) $(HOST_CXXFLAGS) $(HOST_CPPFLAGS) -MMD -MF $(addsuffix .dep, $(basename $@)) -c -o $@ $<

$(OBJDIR)/$(CONFIG)/%.o : %.cc
	$(E) "[CXX]     Compiling $<"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(CXX) $(CXXFLAGS) $(CPPFLAGS) -MMD -MF $(addsuffix .dep, $(basename $@)) -c -o $@ $<

install: install_c install_cxx install-plugins install-certs verify-install

install_c: install-headers_c install-static_c install-shared_c

install_cxx: install-headers_cxx install-static_cxx install-shared_cxx

install_csharp: install-shared_csharp install_c

install_grpc_csharp_ext: install_csharp

install-headers: install-headers_c install-headers_cxx

install-headers_c:
	$(E) "[INSTALL] Installing public C headers"
	$(Q) $(foreach h, $(PUBLIC_HEADERS_C), $(INSTALL) -d $(prefix)/$(dir $(h)) && ) exit 0 || exit 1
	$(Q) $(foreach h, $(PUBLIC_HEADERS_C), $(INSTALL) $(h) $(prefix)/$(h) && ) exit 0 || exit 1

install-headers_cxx:
	$(E) "[INSTALL] Installing public C++ headers"
	$(Q) $(foreach h, $(PUBLIC_HEADERS_CXX), $(INSTALL) -d $(prefix)/$(dir $(h)) && ) exit 0 || exit 1
	$(Q) $(foreach h, $(PUBLIC_HEADERS_CXX), $(INSTALL) $(h) $(prefix)/$(h) && ) exit 0 || exit 1

install-static: install-static_c install-static_cxx

install-static_c: static_c strip-static_c install-pkg-config_c
	$(E) "[INSTALL] Installing libgpr.a"
	$(Q) $(INSTALL) -d $(prefix)/lib
	$(Q) $(INSTALL) $(LIBDIR)/$(CONFIG)/libgpr.a $(prefix)/lib/libgpr.a
	$(E) "[INSTALL] Installing libgrpc.a"
	$(Q) $(INSTALL) -d $(prefix)/lib
	$(Q) $(INSTALL) $(LIBDIR)/$(CONFIG)/libgrpc.a $(prefix)/lib/libgrpc.a
	$(E) "[INSTALL] Installing libgrpc_unsecure.a"
	$(Q) $(INSTALL) -d $(prefix)/lib
	$(Q) $(INSTALL) $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.a $(prefix)/lib/libgrpc_unsecure.a
ifeq ($(HAS_ZOOKEEPER),true)
	$(E) "[INSTALL] Installing libgrpc_zookeeper.a"
	$(Q) $(INSTALL) -d $(prefix)/lib
	$(Q) $(INSTALL) $(LIBDIR)/$(CONFIG)/libgrpc_zookeeper.a $(prefix)/lib/libgrpc_zookeeper.a
endif

install-static_cxx: static_cxx strip-static_cxx install-pkg-config_cxx
	$(E) "[INSTALL] Installing libgrpc++.a"
	$(Q) $(INSTALL) -d $(prefix)/lib
	$(Q) $(INSTALL) $(LIBDIR)/$(CONFIG)/libgrpc++.a $(prefix)/lib/libgrpc++.a
	$(E) "[INSTALL] Installing libgrpc++_unsecure.a"
	$(Q) $(INSTALL) -d $(prefix)/lib
	$(Q) $(INSTALL) $(LIBDIR)/$(CONFIG)/libgrpc++_unsecure.a $(prefix)/lib/libgrpc++_unsecure.a



install-shared_c: shared_c strip-shared_c install-pkg-config_c
ifeq ($(SYSTEM),MINGW32)
	$(E) "[INSTALL] Installing gpr.$(SHARED_EXT)"
	$(Q) $(INSTALL) -d $(prefix)/lib
	$(Q) $(INSTALL) $(LIBDIR)/$(CONFIG)/gpr.$(SHARED_EXT) $(prefix)/lib/gpr.$(SHARED_EXT)
	$(Q) $(INSTALL) $(LIBDIR)/$(CONFIG)/libgpr-imp.a $(prefix)/lib/libgpr-imp.a
else
	$(E) "[INSTALL] Installing libgpr.$(SHARED_EXT)"
	$(Q) $(INSTALL) -d $(prefix)/lib
	$(Q) $(INSTALL) $(LIBDIR)/$(CONFIG)/libgpr.$(SHARED_EXT) $(prefix)/lib/libgpr.$(SHARED_EXT)
ifneq ($(SYSTEM),Darwin)
	$(Q) ln -sf libgpr.$(SHARED_EXT) $(prefix)/lib/libgpr.so.0
	$(Q) ln -sf libgpr.$(SHARED_EXT) $(prefix)/lib/libgpr.so
endif
endif
ifeq ($(SYSTEM),MINGW32)
	$(E) "[INSTALL] Installing grpc.$(SHARED_EXT)"
	$(Q) $(INSTALL) -d $(prefix)/lib
	$(Q) $(INSTALL) $(LIBDIR)/$(CONFIG)/grpc.$(SHARED_EXT) $(prefix)/lib/grpc.$(SHARED_EXT)
	$(Q) $(INSTALL) $(LIBDIR)/$(CONFIG)/libgrpc-imp.a $(prefix)/lib/libgrpc-imp.a
else
	$(E) "[INSTALL] Installing libgrpc.$(SHARED_EXT)"
	$(Q) $(INSTALL) -d $(prefix)/lib
	$(Q) $(INSTALL) $(LIBDIR)/$(CONFIG)/libgrpc.$(SHARED_EXT) $(prefix)/lib/libgrpc.$(SHARED_EXT)
ifneq ($(SYSTEM),Darwin)
	$(Q) ln -sf libgrpc.$(SHARED_EXT) $(prefix)/lib/libgrpc.so.0
	$(Q) ln -sf libgrpc.$(SHARED_EXT) $(prefix)/lib/libgrpc.so
endif
endif
ifeq ($(SYSTEM),MINGW32)
	$(E) "[INSTALL] Installing grpc_unsecure.$(SHARED_EXT)"
	$(Q) $(INSTALL) -d $(prefix)/lib
	$(Q) $(INSTALL) $(LIBDIR)/$(CONFIG)/grpc_unsecure.$(SHARED_EXT) $(prefix)/lib/grpc_unsecure.$(SHARED_EXT)
	$(Q) $(INSTALL) $(LIBDIR)/$(CONFIG)/libgrpc_unsecure-imp.a $(prefix)/lib/libgrpc_unsecure-imp.a
else
	$(E) "[INSTALL] Installing libgrpc_unsecure.$(SHARED_EXT)"
	$(Q) $(INSTALL) -d $(prefix)/lib
	$(Q) $(INSTALL) $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.$(SHARED_EXT) $(prefix)/lib/libgrpc_unsecure.$(SHARED_EXT)
ifneq ($(SYSTEM),Darwin)
	$(Q) ln -sf libgrpc_unsecure.$(SHARED_EXT) $(prefix)/lib/libgrpc_unsecure.so.0
	$(Q) ln -sf libgrpc_unsecure.$(SHARED_EXT) $(prefix)/lib/libgrpc_unsecure.so
endif
endif
ifeq ($(HAS_ZOOKEEPER),true)
ifeq ($(SYSTEM),MINGW32)
	$(E) "[INSTALL] Installing grpc_zookeeper.$(SHARED_EXT)"
	$(Q) $(INSTALL) -d $(prefix)/lib
	$(Q) $(INSTALL) $(LIBDIR)/$(CONFIG)/grpc_zookeeper.$(SHARED_EXT) $(prefix)/lib/grpc_zookeeper.$(SHARED_EXT)
	$(Q) $(INSTALL) $(LIBDIR)/$(CONFIG)/libgrpc_zookeeper-imp.a $(prefix)/lib/libgrpc_zookeeper-imp.a
else
	$(E) "[INSTALL] Installing libgrpc_zookeeper.$(SHARED_EXT)"
	$(Q) $(INSTALL) -d $(prefix)/lib
	$(Q) $(INSTALL) $(LIBDIR)/$(CONFIG)/libgrpc_zookeeper.$(SHARED_EXT) $(prefix)/lib/libgrpc_zookeeper.$(SHARED_EXT)
ifneq ($(SYSTEM),Darwin)
	$(Q) ln -sf libgrpc_zookeeper.$(SHARED_EXT) $(prefix)/lib/libgrpc_zookeeper.so.0
	$(Q) ln -sf libgrpc_zookeeper.$(SHARED_EXT) $(prefix)/lib/libgrpc_zookeeper.so
endif
endif
endif
ifneq ($(SYSTEM),MINGW32)
ifneq ($(SYSTEM),Darwin)
	$(Q) ldconfig || true
endif
endif


install-shared_cxx: shared_cxx strip-shared_cxx install-shared_c install-pkg-config_cxx
ifeq ($(SYSTEM),MINGW32)
	$(E) "[INSTALL] Installing grpc++.$(SHARED_EXT)"
	$(Q) $(INSTALL) -d $(prefix)/lib
	$(Q) $(INSTALL) $(LIBDIR)/$(CONFIG)/grpc++.$(SHARED_EXT) $(prefix)/lib/grpc++.$(SHARED_EXT)
	$(Q) $(INSTALL) $(LIBDIR)/$(CONFIG)/libgrpc++-imp.a $(prefix)/lib/libgrpc++-imp.a
else
	$(E) "[INSTALL] Installing libgrpc++.$(SHARED_EXT)"
	$(Q) $(INSTALL) -d $(prefix)/lib
	$(Q) $(INSTALL) $(LIBDIR)/$(CONFIG)/libgrpc++.$(SHARED_EXT) $(prefix)/lib/libgrpc++.$(SHARED_EXT)
ifneq ($(SYSTEM),Darwin)
	$(Q) ln -sf libgrpc++.$(SHARED_EXT) $(prefix)/lib/libgrpc++.so.0
	$(Q) ln -sf libgrpc++.$(SHARED_EXT) $(prefix)/lib/libgrpc++.so
endif
endif
ifeq ($(SYSTEM),MINGW32)
	$(E) "[INSTALL] Installing grpc++_unsecure.$(SHARED_EXT)"
	$(Q) $(INSTALL) -d $(prefix)/lib
	$(Q) $(INSTALL) $(LIBDIR)/$(CONFIG)/grpc++_unsecure.$(SHARED_EXT) $(prefix)/lib/grpc++_unsecure.$(SHARED_EXT)
	$(Q) $(INSTALL) $(LIBDIR)/$(CONFIG)/libgrpc++_unsecure-imp.a $(prefix)/lib/libgrpc++_unsecure-imp.a
else
	$(E) "[INSTALL] Installing libgrpc++_unsecure.$(SHARED_EXT)"
	$(Q) $(INSTALL) -d $(prefix)/lib
	$(Q) $(INSTALL) $(LIBDIR)/$(CONFIG)/libgrpc++_unsecure.$(SHARED_EXT) $(prefix)/lib/libgrpc++_unsecure.$(SHARED_EXT)
ifneq ($(SYSTEM),Darwin)
	$(Q) ln -sf libgrpc++_unsecure.$(SHARED_EXT) $(prefix)/lib/libgrpc++_unsecure.so.0
	$(Q) ln -sf libgrpc++_unsecure.$(SHARED_EXT) $(prefix)/lib/libgrpc++_unsecure.so
endif
endif
ifeq ($(HAS_ZOOKEEPER),true)
endif
ifneq ($(SYSTEM),MINGW32)
ifneq ($(SYSTEM),Darwin)
	$(Q) ldconfig || true
endif
endif


install-shared_csharp: shared_csharp strip-shared_csharp
ifeq ($(SYSTEM),MINGW32)
	$(E) "[INSTALL] Installing grpc_csharp_ext.$(SHARED_EXT)"
	$(Q) $(INSTALL) -d $(prefix)/lib
	$(Q) $(INSTALL) $(LIBDIR)/$(CONFIG)/grpc_csharp_ext.$(SHARED_EXT) $(prefix)/lib/grpc_csharp_ext.$(SHARED_EXT)
	$(Q) $(INSTALL) $(LIBDIR)/$(CONFIG)/libgrpc_csharp_ext-imp.a $(prefix)/lib/libgrpc_csharp_ext-imp.a
else
	$(E) "[INSTALL] Installing libgrpc_csharp_ext.$(SHARED_EXT)"
	$(Q) $(INSTALL) -d $(prefix)/lib
	$(Q) $(INSTALL) $(LIBDIR)/$(CONFIG)/libgrpc_csharp_ext.$(SHARED_EXT) $(prefix)/lib/libgrpc_csharp_ext.$(SHARED_EXT)
ifneq ($(SYSTEM),Darwin)
	$(Q) ln -sf libgrpc_csharp_ext.$(SHARED_EXT) $(prefix)/lib/libgrpc_csharp_ext.so.0
	$(Q) ln -sf libgrpc_csharp_ext.$(SHARED_EXT) $(prefix)/lib/libgrpc_csharp_ext.so
endif
endif
ifeq ($(HAS_ZOOKEEPER),true)
endif
ifneq ($(SYSTEM),MINGW32)
ifneq ($(SYSTEM),Darwin)
	$(Q) ldconfig || true
endif
endif


install-plugins: $(PROTOC_PLUGINS)
ifeq ($(SYSTEM),MINGW32)
	$(Q) false
else
	$(E) "[INSTALL] Installing grpc protoc plugins"
	$(Q) $(INSTALL) -d $(prefix)/bin
	$(Q) $(INSTALL) $(BINDIR)/$(CONFIG)/grpc_cpp_plugin $(prefix)/bin/grpc_cpp_plugin
	$(Q) $(INSTALL) -d $(prefix)/bin
	$(Q) $(INSTALL) $(BINDIR)/$(CONFIG)/grpc_csharp_plugin $(prefix)/bin/grpc_csharp_plugin
	$(Q) $(INSTALL) -d $(prefix)/bin
	$(Q) $(INSTALL) $(BINDIR)/$(CONFIG)/grpc_objective_c_plugin $(prefix)/bin/grpc_objective_c_plugin
	$(Q) $(INSTALL) -d $(prefix)/bin
	$(Q) $(INSTALL) $(BINDIR)/$(CONFIG)/grpc_python_plugin $(prefix)/bin/grpc_python_plugin
	$(Q) $(INSTALL) -d $(prefix)/bin
	$(Q) $(INSTALL) $(BINDIR)/$(CONFIG)/grpc_ruby_plugin $(prefix)/bin/grpc_ruby_plugin
endif

install-pkg-config_c: pc_gpr pc_c pc_c_unsecure pc_c_zookeeper
	$(E) "[INSTALL] Installing C pkg-config files"
	$(Q) $(INSTALL) -d $(prefix)/lib/pkgconfig
	$(Q) $(INSTALL) $(LIBDIR)/$(CONFIG)/pkgconfig/gpr.pc $(prefix)/lib/pkgconfig/gpr.pc
	$(Q) $(INSTALL) $(LIBDIR)/$(CONFIG)/pkgconfig/grpc.pc $(prefix)/lib/pkgconfig/grpc.pc
	$(Q) $(INSTALL) $(LIBDIR)/$(CONFIG)/pkgconfig/grpc_unsecure.pc $(prefix)/lib/pkgconfig/grpc_unsecure.pc
ifeq ($(HAS_ZOOKEEPER),true)
	$(Q) $(INSTALL) $(LIBDIR)/$(CONFIG)/pkgconfig/grpc_zookeeper.pc $(prefix)/lib/pkgconfig/grpc_zookeeper.pc
endif

install-pkg-config_cxx: pc_cxx pc_cxx_unsecure
	$(E) "[INSTALL] Installing C++ pkg-config files"
	$(Q) $(INSTALL) -d $(prefix)/lib/pkgconfig
	$(Q) $(INSTALL) $(LIBDIR)/$(CONFIG)/pkgconfig/grpc++.pc $(prefix)/lib/pkgconfig/grpc++.pc
	$(Q) $(INSTALL) $(LIBDIR)/$(CONFIG)/pkgconfig/grpc++_unsecure.pc $(prefix)/lib/pkgconfig/grpc++_unsecure.pc

install-certs: etc/roots.pem
	$(E) "[INSTALL] Installing root certificates"
	$(Q) $(INSTALL) -d $(prefix)/share/grpc
	$(Q) $(INSTALL) etc/roots.pem $(prefix)/share/grpc/roots.pem

verify-install:
ifeq ($(INSTALL_OK),true)
	@echo "Your system looks ready to go."
	@echo
else
	@echo "We couldn't find protoc 3.0.0+ installed on your system. While this"
	@echo "won't prevent grpc from working, you won't be able to compile"
	@echo "and run any meaningful code with it."
	@echo
	@echo
	@echo "Please download and install protobuf 3.0.0+ from:"
	@echo
	@echo "   https://github.com/google/protobuf/releases"
	@echo
	@echo "Once you've done so, or if you think this message is in error,"
	@echo "you can re-run this check by doing:"
	@echo
	@echo "   make verify-install"
endif

clean:
	$(E) "[CLEAN]   Cleaning build directories."
	$(Q) $(RM) -rf $(OBJDIR) $(LIBDIR) $(BINDIR) $(GENDIR) cache.mk


# The various libraries


LIBGPR_SRC = \
    src/core/profiling/basic_timers.c \
    src/core/profiling/stap_timers.c \
    src/core/support/alloc.c \
    src/core/support/avl.c \
    src/core/support/cmdline.c \
    src/core/support/cpu_iphone.c \
    src/core/support/cpu_linux.c \
    src/core/support/cpu_posix.c \
    src/core/support/cpu_windows.c \
    src/core/support/env_linux.c \
    src/core/support/env_posix.c \
    src/core/support/env_win32.c \
    src/core/support/file.c \
    src/core/support/file_posix.c \
    src/core/support/file_win32.c \
    src/core/support/histogram.c \
    src/core/support/host_port.c \
    src/core/support/log.c \
    src/core/support/log_android.c \
    src/core/support/log_linux.c \
    src/core/support/log_posix.c \
    src/core/support/log_win32.c \
    src/core/support/murmur_hash.c \
    src/core/support/slice.c \
    src/core/support/slice_buffer.c \
    src/core/support/stack_lockfree.c \
    src/core/support/string.c \
    src/core/support/string_posix.c \
    src/core/support/string_win32.c \
    src/core/support/subprocess_posix.c \
    src/core/support/sync.c \
    src/core/support/sync_posix.c \
    src/core/support/sync_win32.c \
    src/core/support/thd.c \
    src/core/support/thd_posix.c \
    src/core/support/thd_win32.c \
    src/core/support/time.c \
    src/core/support/time_posix.c \
    src/core/support/time_precise.c \
    src/core/support/time_win32.c \
    src/core/support/tls_pthread.c \

PUBLIC_HEADERS_C += \
    include/grpc/support/alloc.h \
    include/grpc/support/atm.h \
    include/grpc/support/atm_gcc_atomic.h \
    include/grpc/support/atm_gcc_sync.h \
    include/grpc/support/atm_win32.h \
    include/grpc/support/avl.h \
    include/grpc/support/cmdline.h \
    include/grpc/support/cpu.h \
    include/grpc/support/histogram.h \
    include/grpc/support/host_port.h \
    include/grpc/support/log.h \
    include/grpc/support/log_win32.h \
    include/grpc/support/port_platform.h \
    include/grpc/support/slice.h \
    include/grpc/support/slice_buffer.h \
    include/grpc/support/string_util.h \
    include/grpc/support/subprocess.h \
    include/grpc/support/sync.h \
    include/grpc/support/sync_generic.h \
    include/grpc/support/sync_posix.h \
    include/grpc/support/sync_win32.h \
    include/grpc/support/thd.h \
    include/grpc/support/time.h \
    include/grpc/support/tls.h \
    include/grpc/support/tls_gcc.h \
    include/grpc/support/tls_msvc.h \
    include/grpc/support/tls_pthread.h \
    include/grpc/support/useful.h \

LIBGPR_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(LIBGPR_SRC))))


$(LIBDIR)/$(CONFIG)/libgpr.a: $(ZLIB_DEP) $(OPENSSL_DEP)  $(LIBGPR_OBJS)
	$(E) "[AR]      Creating $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) rm -f $(LIBDIR)/$(CONFIG)/libgpr.a
	$(Q) $(AR) rcs $(LIBDIR)/$(CONFIG)/libgpr.a $(LIBGPR_OBJS)
ifeq ($(SYSTEM),Darwin)
	$(Q) ranlib $(LIBDIR)/$(CONFIG)/libgpr.a
endif



ifeq ($(SYSTEM),MINGW32)
$(LIBDIR)/$(CONFIG)/gpr.$(SHARED_EXT): $(LIBGPR_OBJS)  $(ZLIB_DEP)
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) -L$(LIBDIR)/$(CONFIG) -shared -Wl,--output-def=$(LIBDIR)/$(CONFIG)/gpr.def -Wl,--out-implib=$(LIBDIR)/$(CONFIG)/libgpr-imp.a -o $(LIBDIR)/$(CONFIG)/gpr.$(SHARED_EXT) $(LIBGPR_OBJS) $(LDLIBS) $(ZLIB_MERGE_LIBS)
else
$(LIBDIR)/$(CONFIG)/libgpr.$(SHARED_EXT): $(LIBGPR_OBJS)  $(ZLIB_DEP)
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
ifeq ($(SYSTEM),Darwin)
	$(Q) $(LD) $(LDFLAGS) -L$(LIBDIR)/$(CONFIG) -install_name libgpr.$(SHARED_EXT) -dynamiclib -o $(LIBDIR)/$(CONFIG)/libgpr.$(SHARED_EXT) $(LIBGPR_OBJS) $(LDLIBS) $(ZLIB_MERGE_LIBS)
else
	$(Q) $(LD) $(LDFLAGS) -L$(LIBDIR)/$(CONFIG) -shared -Wl,-soname,libgpr.so.0 -o $(LIBDIR)/$(CONFIG)/libgpr.$(SHARED_EXT) $(LIBGPR_OBJS) $(LDLIBS) $(ZLIB_MERGE_LIBS)
	$(Q) ln -sf libgpr.$(SHARED_EXT) $(LIBDIR)/$(CONFIG)/libgpr.so.0
	$(Q) ln -sf libgpr.$(SHARED_EXT) $(LIBDIR)/$(CONFIG)/libgpr.so
endif
endif

ifneq ($(NO_DEPS),true)
-include $(LIBGPR_OBJS:.o=.dep)
endif


LIBGPR_TEST_UTIL_SRC = \
    test/core/util/test_config.c \


LIBGPR_TEST_UTIL_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(LIBGPR_TEST_UTIL_SRC))))


$(LIBDIR)/$(CONFIG)/libgpr_test_util.a: $(ZLIB_DEP) $(OPENSSL_DEP)  $(LIBGPR_TEST_UTIL_OBJS)
	$(E) "[AR]      Creating $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) rm -f $(LIBDIR)/$(CONFIG)/libgpr_test_util.a
	$(Q) $(AR) rcs $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBGPR_TEST_UTIL_OBJS)
ifeq ($(SYSTEM),Darwin)
	$(Q) ranlib $(LIBDIR)/$(CONFIG)/libgpr_test_util.a
endif




ifneq ($(NO_DEPS),true)
-include $(LIBGPR_TEST_UTIL_OBJS:.o=.dep)
endif


LIBGRPC_SRC = \
    src/core/httpcli/httpcli_security_connector.c \
    src/core/security/base64.c \
    src/core/security/client_auth_filter.c \
    src/core/security/credentials.c \
    src/core/security/credentials_metadata.c \
    src/core/security/credentials_posix.c \
    src/core/security/credentials_win32.c \
    src/core/security/google_default_credentials.c \
    src/core/security/handshake.c \
    src/core/security/json_token.c \
    src/core/security/jwt_verifier.c \
    src/core/security/secure_endpoint.c \
    src/core/security/security_connector.c \
    src/core/security/security_context.c \
    src/core/security/server_auth_filter.c \
    src/core/security/server_secure_chttp2.c \
    src/core/surface/init_secure.c \
    src/core/surface/secure_channel_create.c \
    src/core/tsi/fake_transport_security.c \
    src/core/tsi/ssl_transport_security.c \
    src/core/tsi/transport_security.c \
    src/core/census/grpc_context.c \
    src/core/census/grpc_filter.c \
    src/core/channel/channel_args.c \
    src/core/channel/channel_stack.c \
    src/core/channel/client_channel.c \
    src/core/channel/client_uchannel.c \
    src/core/channel/compress_filter.c \
    src/core/channel/connected_channel.c \
    src/core/channel/http_client_filter.c \
    src/core/channel/http_server_filter.c \
    src/core/channel/subchannel_call_holder.c \
    src/core/client_config/client_config.c \
    src/core/client_config/connector.c \
    src/core/client_config/default_initial_connect_string.c \
    src/core/client_config/initial_connect_string.c \
    src/core/client_config/lb_policies/pick_first.c \
    src/core/client_config/lb_policies/round_robin.c \
    src/core/client_config/lb_policy.c \
    src/core/client_config/lb_policy_factory.c \
    src/core/client_config/lb_policy_registry.c \
    src/core/client_config/resolver.c \
    src/core/client_config/resolver_factory.c \
    src/core/client_config/resolver_registry.c \
    src/core/client_config/resolvers/dns_resolver.c \
    src/core/client_config/resolvers/sockaddr_resolver.c \
    src/core/client_config/subchannel.c \
    src/core/client_config/subchannel_factory.c \
    src/core/client_config/uri_parser.c \
    src/core/compression/algorithm.c \
    src/core/compression/message_compress.c \
    src/core/debug/trace.c \
    src/core/httpcli/format_request.c \
    src/core/httpcli/httpcli.c \
    src/core/httpcli/parser.c \
    src/core/iomgr/closure.c \
    src/core/iomgr/endpoint.c \
    src/core/iomgr/endpoint_pair_posix.c \
    src/core/iomgr/endpoint_pair_windows.c \
    src/core/iomgr/exec_ctx.c \
    src/core/iomgr/executor.c \
    src/core/iomgr/fd_posix.c \
    src/core/iomgr/iocp_windows.c \
    src/core/iomgr/iomgr.c \
    src/core/iomgr/iomgr_posix.c \
    src/core/iomgr/iomgr_windows.c \
    src/core/iomgr/pollset_multipoller_with_epoll.c \
    src/core/iomgr/pollset_multipoller_with_poll_posix.c \
    src/core/iomgr/pollset_posix.c \
    src/core/iomgr/pollset_set_posix.c \
    src/core/iomgr/pollset_set_windows.c \
    src/core/iomgr/pollset_windows.c \
    src/core/iomgr/resolve_address_posix.c \
    src/core/iomgr/resolve_address_windows.c \
    src/core/iomgr/sockaddr_utils.c \
    src/core/iomgr/socket_utils_common_posix.c \
    src/core/iomgr/socket_utils_linux.c \
    src/core/iomgr/socket_utils_posix.c \
    src/core/iomgr/socket_windows.c \
    src/core/iomgr/tcp_client_posix.c \
    src/core/iomgr/tcp_client_windows.c \
    src/core/iomgr/tcp_posix.c \
    src/core/iomgr/tcp_server_posix.c \
    src/core/iomgr/tcp_server_windows.c \
    src/core/iomgr/tcp_windows.c \
    src/core/iomgr/time_averaged_stats.c \
    src/core/iomgr/timer.c \
    src/core/iomgr/timer_heap.c \
    src/core/iomgr/udp_server.c \
    src/core/iomgr/wakeup_fd_eventfd.c \
    src/core/iomgr/wakeup_fd_nospecial.c \
    src/core/iomgr/wakeup_fd_pipe.c \
    src/core/iomgr/wakeup_fd_posix.c \
    src/core/iomgr/workqueue_posix.c \
    src/core/iomgr/workqueue_windows.c \
    src/core/json/json.c \
    src/core/json/json_reader.c \
    src/core/json/json_string.c \
    src/core/json/json_writer.c \
    src/core/surface/api_trace.c \
    src/core/surface/byte_buffer.c \
    src/core/surface/byte_buffer_reader.c \
    src/core/surface/call.c \
    src/core/surface/call_details.c \
    src/core/surface/call_log_batch.c \
    src/core/surface/channel.c \
    src/core/surface/channel_connectivity.c \
    src/core/surface/channel_create.c \
    src/core/surface/channel_ping.c \
    src/core/surface/completion_queue.c \
    src/core/surface/event_string.c \
    src/core/surface/init.c \
    src/core/surface/lame_client.c \
    src/core/surface/metadata_array.c \
    src/core/surface/server.c \
    src/core/surface/server_chttp2.c \
    src/core/surface/server_create.c \
    src/core/surface/version.c \
    src/core/transport/byte_stream.c \
    src/core/transport/chttp2/alpn.c \
    src/core/transport/chttp2/bin_encoder.c \
    src/core/transport/chttp2/frame_data.c \
    src/core/transport/chttp2/frame_goaway.c \
    src/core/transport/chttp2/frame_ping.c \
    src/core/transport/chttp2/frame_rst_stream.c \
    src/core/transport/chttp2/frame_settings.c \
    src/core/transport/chttp2/frame_window_update.c \
    src/core/transport/chttp2/hpack_encoder.c \
    src/core/transport/chttp2/hpack_parser.c \
    src/core/transport/chttp2/hpack_table.c \
    src/core/transport/chttp2/huffsyms.c \
    src/core/transport/chttp2/incoming_metadata.c \
    src/core/transport/chttp2/parsing.c \
    src/core/transport/chttp2/status_conversion.c \
    src/core/transport/chttp2/stream_lists.c \
    src/core/transport/chttp2/stream_map.c \
    src/core/transport/chttp2/timeout_encoding.c \
    src/core/transport/chttp2/varint.c \
    src/core/transport/chttp2/writing.c \
    src/core/transport/chttp2_transport.c \
    src/core/transport/connectivity_state.c \
    src/core/transport/metadata.c \
    src/core/transport/metadata_batch.c \
    src/core/transport/static_metadata.c \
    src/core/transport/transport.c \
    src/core/transport/transport_op_string.c \
    src/core/census/context.c \
    src/core/census/initialize.c \
    src/core/census/operation.c \
    src/core/census/tracing.c \

PUBLIC_HEADERS_C += \
    include/grpc/grpc_security.h \
    include/grpc/byte_buffer.h \
    include/grpc/byte_buffer_reader.h \
    include/grpc/compression.h \
    include/grpc/grpc.h \
    include/grpc/status.h \
    include/grpc/census.h \

LIBGRPC_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(LIBGRPC_SRC))))


ifeq ($(NO_SECURE),true)

# You can't build secure libraries if you don't have OpenSSL.

$(LIBDIR)/$(CONFIG)/libgrpc.a: openssl_dep_error

ifeq ($(SYSTEM),MINGW32)
$(LIBDIR)/$(CONFIG)/grpc.$(SHARED_EXT): openssl_dep_error
else
$(LIBDIR)/$(CONFIG)/libgrpc.$(SHARED_EXT): openssl_dep_error
endif

else


$(LIBDIR)/$(CONFIG)/libgrpc.a: $(ZLIB_DEP) $(OPENSSL_DEP) $(LIBGRPC_OBJS)
	$(E) "[AR]      Creating $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) rm -f $(LIBDIR)/$(CONFIG)/libgrpc.a
	$(Q) $(AR) rcs $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBGRPC_OBJS)
	$(Q) rm -rf $(BUILDDIR_ABSOLUTE)/tmp-merge-grpc
	$(Q) ( mkdir -p $(BUILDDIR_ABSOLUTE)/tmp-merge-grpc/grpc ; \
	       cd $(BUILDDIR_ABSOLUTE)/tmp-merge-grpc/grpc ; \
	       $(AR) x $(LIBDIR)/$(CONFIG)/libgrpc.a )
	$(Q) for l in $(ZLIB_MERGE_LIBS) ; do ( \
	       mkdir -p $(BUILDDIR_ABSOLUTE)/tmp-merge-grpc/zlib ; \
	       cd $(BUILDDIR_ABSOLUTE)/tmp-merge-grpc/zlib ; \
	       $(AR) x $${l} ) ; done
	$(Q) for l in $(ZLIB_MERGE_LIBS) ; do ( \
	       mkdir -p $(BUILDDIR_ABSOLUTE)/tmp-merge-grpc/zlib ; \
	       cd $(BUILDDIR_ABSOLUTE)/tmp-merge-grpc/zlib ; \
	       $(AR) x $${l} ) ; done
	$(Q) for l in $(OPENSSL_MERGE_LIBS) ; do ( \
	       mkdir -p $(BUILDDIR_ABSOLUTE)/tmp-merge-grpc/ssl ; \
	       cd $(BUILDDIR_ABSOLUTE)/tmp-merge-grpc/ssl ; \
	       $(AR) x $${l} ) ; done
	$(Q) for l in $(OPENSSL_MERGE_LIBS) ; do ( \
	       mkdir -p $(BUILDDIR_ABSOLUTE)/tmp-merge-grpc/ssl ; \
	       cd $(BUILDDIR_ABSOLUTE)/tmp-merge-grpc/ssl ; \
	       $(AR) x $${l} ) ; done
	$(Q) rm -f $(LIBDIR)/$(CONFIG)/libgrpc.a $(BUILDDIR_ABSOLUTE)/tmp-merge-grpc/*/__.SYMDEF*
	$(Q) ar rcs $(LIBDIR)/$(CONFIG)/libgrpc.a $(BUILDDIR_ABSOLUTE)/tmp-merge-grpc/*/*
	$(Q) rm -rf $(BUILDDIR_ABSOLUTE)/tmp-merge-grpc
ifeq ($(SYSTEM),Darwin)
	$(Q) ranlib $(LIBDIR)/$(CONFIG)/libgrpc.a
endif



ifeq ($(SYSTEM),MINGW32)
$(LIBDIR)/$(CONFIG)/grpc.$(SHARED_EXT): $(LIBGRPC_OBJS)  $(ZLIB_DEP) $(LIBDIR)/$(CONFIG)/gpr.$(SHARED_EXT) $(OPENSSL_DEP)
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) -L$(LIBDIR)/$(CONFIG) -shared -Wl,--output-def=$(LIBDIR)/$(CONFIG)/grpc.def -Wl,--out-implib=$(LIBDIR)/$(CONFIG)/libgrpc-imp.a -o $(LIBDIR)/$(CONFIG)/grpc.$(SHARED_EXT) $(LIBGRPC_OBJS) $(LDLIBS) $(OPENSSL_MERGE_LIBS) $(LDLIBS_SECURE) $(ZLIB_MERGE_LIBS) -lgpr-imp
else
$(LIBDIR)/$(CONFIG)/libgrpc.$(SHARED_EXT): $(LIBGRPC_OBJS)  $(ZLIB_DEP) $(LIBDIR)/$(CONFIG)/libgpr.$(SHARED_EXT) $(OPENSSL_DEP)
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
ifeq ($(SYSTEM),Darwin)
	$(Q) $(LD) $(LDFLAGS) -L$(LIBDIR)/$(CONFIG) -install_name libgrpc.$(SHARED_EXT) -dynamiclib -o $(LIBDIR)/$(CONFIG)/libgrpc.$(SHARED_EXT) $(LIBGRPC_OBJS) $(LDLIBS) $(OPENSSL_MERGE_LIBS) $(LDLIBS_SECURE) $(ZLIB_MERGE_LIBS) -lgpr
else
	$(Q) $(LD) $(LDFLAGS) -L$(LIBDIR)/$(CONFIG) -shared -Wl,-soname,libgrpc.so.0 -o $(LIBDIR)/$(CONFIG)/libgrpc.$(SHARED_EXT) $(LIBGRPC_OBJS) $(LDLIBS) $(OPENSSL_MERGE_LIBS) $(LDLIBS_SECURE) $(ZLIB_MERGE_LIBS) -lgpr
	$(Q) ln -sf libgrpc.$(SHARED_EXT) $(LIBDIR)/$(CONFIG)/libgrpc.so.0
	$(Q) ln -sf libgrpc.$(SHARED_EXT) $(LIBDIR)/$(CONFIG)/libgrpc.so
endif
endif

endif

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(LIBGRPC_OBJS:.o=.dep)
endif
endif


LIBGRPC_TEST_UTIL_SRC = \
    test/core/end2end/data/server1_cert.c \
    test/core/end2end/data/server1_key.c \
    test/core/end2end/data/test_root_cert.c \
    test/core/security/oauth2_utils.c \
    test/core/end2end/cq_verifier.c \
    test/core/end2end/fixtures/proxy.c \
    test/core/iomgr/endpoint_tests.c \
    test/core/util/grpc_profiler.c \
    test/core/util/parse_hexstring.c \
    test/core/util/port_posix.c \
    test/core/util/port_windows.c \
    test/core/util/slice_splitter.c \

PUBLIC_HEADERS_C += \

LIBGRPC_TEST_UTIL_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(LIBGRPC_TEST_UTIL_SRC))))


ifeq ($(NO_SECURE),true)

# You can't build secure libraries if you don't have OpenSSL.

$(LIBDIR)/$(CONFIG)/libgrpc_test_util.a: openssl_dep_error


else


$(LIBDIR)/$(CONFIG)/libgrpc_test_util.a: $(ZLIB_DEP) $(OPENSSL_DEP) $(LIBGRPC_TEST_UTIL_OBJS)
	$(E) "[AR]      Creating $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) rm -f $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a
	$(Q) $(AR) rcs $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBGRPC_TEST_UTIL_OBJS)
ifeq ($(SYSTEM),Darwin)
	$(Q) ranlib $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a
endif




endif

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(LIBGRPC_TEST_UTIL_OBJS:.o=.dep)
endif
endif


LIBGRPC_TEST_UTIL_UNSECURE_SRC = \
    test/core/end2end/cq_verifier.c \
    test/core/end2end/fixtures/proxy.c \
    test/core/iomgr/endpoint_tests.c \
    test/core/util/grpc_profiler.c \
    test/core/util/parse_hexstring.c \
    test/core/util/port_posix.c \
    test/core/util/port_windows.c \
    test/core/util/slice_splitter.c \

PUBLIC_HEADERS_C += \

LIBGRPC_TEST_UTIL_UNSECURE_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(LIBGRPC_TEST_UTIL_UNSECURE_SRC))))


$(LIBDIR)/$(CONFIG)/libgrpc_test_util_unsecure.a: $(ZLIB_DEP) $(OPENSSL_DEP)  $(LIBGRPC_TEST_UTIL_UNSECURE_OBJS)
	$(E) "[AR]      Creating $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) rm -f $(LIBDIR)/$(CONFIG)/libgrpc_test_util_unsecure.a
	$(Q) $(AR) rcs $(LIBDIR)/$(CONFIG)/libgrpc_test_util_unsecure.a $(LIBGRPC_TEST_UTIL_UNSECURE_OBJS)
ifeq ($(SYSTEM),Darwin)
	$(Q) ranlib $(LIBDIR)/$(CONFIG)/libgrpc_test_util_unsecure.a
endif




ifneq ($(NO_DEPS),true)
-include $(LIBGRPC_TEST_UTIL_UNSECURE_OBJS:.o=.dep)
endif


LIBGRPC_UNSECURE_SRC = \
    src/core/surface/init_unsecure.c \
    src/core/census/grpc_context.c \
    src/core/census/grpc_filter.c \
    src/core/channel/channel_args.c \
    src/core/channel/channel_stack.c \
    src/core/channel/client_channel.c \
    src/core/channel/client_uchannel.c \
    src/core/channel/compress_filter.c \
    src/core/channel/connected_channel.c \
    src/core/channel/http_client_filter.c \
    src/core/channel/http_server_filter.c \
    src/core/channel/subchannel_call_holder.c \
    src/core/client_config/client_config.c \
    src/core/client_config/connector.c \
    src/core/client_config/default_initial_connect_string.c \
    src/core/client_config/initial_connect_string.c \
    src/core/client_config/lb_policies/pick_first.c \
    src/core/client_config/lb_policies/round_robin.c \
    src/core/client_config/lb_policy.c \
    src/core/client_config/lb_policy_factory.c \
    src/core/client_config/lb_policy_registry.c \
    src/core/client_config/resolver.c \
    src/core/client_config/resolver_factory.c \
    src/core/client_config/resolver_registry.c \
    src/core/client_config/resolvers/dns_resolver.c \
    src/core/client_config/resolvers/sockaddr_resolver.c \
    src/core/client_config/subchannel.c \
    src/core/client_config/subchannel_factory.c \
    src/core/client_config/uri_parser.c \
    src/core/compression/algorithm.c \
    src/core/compression/message_compress.c \
    src/core/debug/trace.c \
    src/core/httpcli/format_request.c \
    src/core/httpcli/httpcli.c \
    src/core/httpcli/parser.c \
    src/core/iomgr/closure.c \
    src/core/iomgr/endpoint.c \
    src/core/iomgr/endpoint_pair_posix.c \
    src/core/iomgr/endpoint_pair_windows.c \
    src/core/iomgr/exec_ctx.c \
    src/core/iomgr/executor.c \
    src/core/iomgr/fd_posix.c \
    src/core/iomgr/iocp_windows.c \
    src/core/iomgr/iomgr.c \
    src/core/iomgr/iomgr_posix.c \
    src/core/iomgr/iomgr_windows.c \
    src/core/iomgr/pollset_multipoller_with_epoll.c \
    src/core/iomgr/pollset_multipoller_with_poll_posix.c \
    src/core/iomgr/pollset_posix.c \
    src/core/iomgr/pollset_set_posix.c \
    src/core/iomgr/pollset_set_windows.c \
    src/core/iomgr/pollset_windows.c \
    src/core/iomgr/resolve_address_posix.c \
    src/core/iomgr/resolve_address_windows.c \
    src/core/iomgr/sockaddr_utils.c \
    src/core/iomgr/socket_utils_common_posix.c \
    src/core/iomgr/socket_utils_linux.c \
    src/core/iomgr/socket_utils_posix.c \
    src/core/iomgr/socket_windows.c \
    src/core/iomgr/tcp_client_posix.c \
    src/core/iomgr/tcp_client_windows.c \
    src/core/iomgr/tcp_posix.c \
    src/core/iomgr/tcp_server_posix.c \
    src/core/iomgr/tcp_server_windows.c \
    src/core/iomgr/tcp_windows.c \
    src/core/iomgr/time_averaged_stats.c \
    src/core/iomgr/timer.c \
    src/core/iomgr/timer_heap.c \
    src/core/iomgr/udp_server.c \
    src/core/iomgr/wakeup_fd_eventfd.c \
    src/core/iomgr/wakeup_fd_nospecial.c \
    src/core/iomgr/wakeup_fd_pipe.c \
    src/core/iomgr/wakeup_fd_posix.c \
    src/core/iomgr/workqueue_posix.c \
    src/core/iomgr/workqueue_windows.c \
    src/core/json/json.c \
    src/core/json/json_reader.c \
    src/core/json/json_string.c \
    src/core/json/json_writer.c \
    src/core/surface/api_trace.c \
    src/core/surface/byte_buffer.c \
    src/core/surface/byte_buffer_reader.c \
    src/core/surface/call.c \
    src/core/surface/call_details.c \
    src/core/surface/call_log_batch.c \
    src/core/surface/channel.c \
    src/core/surface/channel_connectivity.c \
    src/core/surface/channel_create.c \
    src/core/surface/channel_ping.c \
    src/core/surface/completion_queue.c \
    src/core/surface/event_string.c \
    src/core/surface/init.c \
    src/core/surface/lame_client.c \
    src/core/surface/metadata_array.c \
    src/core/surface/server.c \
    src/core/surface/server_chttp2.c \
    src/core/surface/server_create.c \
    src/core/surface/version.c \
    src/core/transport/byte_stream.c \
    src/core/transport/chttp2/alpn.c \
    src/core/transport/chttp2/bin_encoder.c \
    src/core/transport/chttp2/frame_data.c \
    src/core/transport/chttp2/frame_goaway.c \
    src/core/transport/chttp2/frame_ping.c \
    src/core/transport/chttp2/frame_rst_stream.c \
    src/core/transport/chttp2/frame_settings.c \
    src/core/transport/chttp2/frame_window_update.c \
    src/core/transport/chttp2/hpack_encoder.c \
    src/core/transport/chttp2/hpack_parser.c \
    src/core/transport/chttp2/hpack_table.c \
    src/core/transport/chttp2/huffsyms.c \
    src/core/transport/chttp2/incoming_metadata.c \
    src/core/transport/chttp2/parsing.c \
    src/core/transport/chttp2/status_conversion.c \
    src/core/transport/chttp2/stream_lists.c \
    src/core/transport/chttp2/stream_map.c \
    src/core/transport/chttp2/timeout_encoding.c \
    src/core/transport/chttp2/varint.c \
    src/core/transport/chttp2/writing.c \
    src/core/transport/chttp2_transport.c \
    src/core/transport/connectivity_state.c \
    src/core/transport/metadata.c \
    src/core/transport/metadata_batch.c \
    src/core/transport/static_metadata.c \
    src/core/transport/transport.c \
    src/core/transport/transport_op_string.c \
    src/core/census/context.c \
    src/core/census/initialize.c \
    src/core/census/operation.c \
    src/core/census/tracing.c \

PUBLIC_HEADERS_C += \
    include/grpc/byte_buffer.h \
    include/grpc/byte_buffer_reader.h \
    include/grpc/compression.h \
    include/grpc/grpc.h \
    include/grpc/status.h \
    include/grpc/census.h \

LIBGRPC_UNSECURE_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(LIBGRPC_UNSECURE_SRC))))


$(LIBDIR)/$(CONFIG)/libgrpc_unsecure.a: $(ZLIB_DEP) $(OPENSSL_DEP)  $(LIBGRPC_UNSECURE_OBJS)
	$(E) "[AR]      Creating $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) rm -f $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.a
	$(Q) $(AR) rcs $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.a $(LIBGRPC_UNSECURE_OBJS)
	$(Q) rm -rf $(BUILDDIR_ABSOLUTE)/tmp-merge-grpc_unsecure
	$(Q) ( mkdir -p $(BUILDDIR_ABSOLUTE)/tmp-merge-grpc_unsecure/grpc ; \
	       cd $(BUILDDIR_ABSOLUTE)/tmp-merge-grpc_unsecure/grpc ; \
	       $(AR) x $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.a )
	$(Q) for l in $(ZLIB_MERGE_LIBS) ; do ( \
	       mkdir -p $(BUILDDIR_ABSOLUTE)/tmp-merge-grpc_unsecure/zlib ; \
	       cd $(BUILDDIR_ABSOLUTE)/tmp-merge-grpc_unsecure/zlib ; \
	       $(AR) x $${l} ) ; done
	$(Q) for l in $(ZLIB_MERGE_LIBS) ; do ( \
	       mkdir -p $(BUILDDIR_ABSOLUTE)/tmp-merge-grpc_unsecure/zlib ; \
	       cd $(BUILDDIR_ABSOLUTE)/tmp-merge-grpc_unsecure/zlib ; \
	       $(AR) x $${l} ) ; done
	$(Q) rm -f $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.a $(BUILDDIR_ABSOLUTE)/tmp-merge-grpc_unsecure/*/__.SYMDEF*
	$(Q) ar rcs $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.a $(BUILDDIR_ABSOLUTE)/tmp-merge-grpc_unsecure/*/*
	$(Q) rm -rf $(BUILDDIR_ABSOLUTE)/tmp-merge-grpc_unsecure
ifeq ($(SYSTEM),Darwin)
	$(Q) ranlib $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.a
endif



ifeq ($(SYSTEM),MINGW32)
$(LIBDIR)/$(CONFIG)/grpc_unsecure.$(SHARED_EXT): $(LIBGRPC_UNSECURE_OBJS)  $(ZLIB_DEP) $(LIBDIR)/$(CONFIG)/gpr.$(SHARED_EXT)
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) -L$(LIBDIR)/$(CONFIG) -shared -Wl,--output-def=$(LIBDIR)/$(CONFIG)/grpc_unsecure.def -Wl,--out-implib=$(LIBDIR)/$(CONFIG)/libgrpc_unsecure-imp.a -o $(LIBDIR)/$(CONFIG)/grpc_unsecure.$(SHARED_EXT) $(LIBGRPC_UNSECURE_OBJS) $(LDLIBS) $(ZLIB_MERGE_LIBS) -lgpr-imp
else
$(LIBDIR)/$(CONFIG)/libgrpc_unsecure.$(SHARED_EXT): $(LIBGRPC_UNSECURE_OBJS)  $(ZLIB_DEP) $(LIBDIR)/$(CONFIG)/libgpr.$(SHARED_EXT)
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
ifeq ($(SYSTEM),Darwin)
	$(Q) $(LD) $(LDFLAGS) -L$(LIBDIR)/$(CONFIG) -install_name libgrpc_unsecure.$(SHARED_EXT) -dynamiclib -o $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.$(SHARED_EXT) $(LIBGRPC_UNSECURE_OBJS) $(LDLIBS) $(ZLIB_MERGE_LIBS) -lgpr
else
	$(Q) $(LD) $(LDFLAGS) -L$(LIBDIR)/$(CONFIG) -shared -Wl,-soname,libgrpc_unsecure.so.0 -o $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.$(SHARED_EXT) $(LIBGRPC_UNSECURE_OBJS) $(LDLIBS) $(ZLIB_MERGE_LIBS) -lgpr
	$(Q) ln -sf libgrpc_unsecure.$(SHARED_EXT) $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.so.0
	$(Q) ln -sf libgrpc_unsecure.$(SHARED_EXT) $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.so
endif
endif

ifneq ($(NO_DEPS),true)
-include $(LIBGRPC_UNSECURE_OBJS:.o=.dep)
endif


LIBGRPC_ZOOKEEPER_SRC = \
    src/core/client_config/resolvers/zookeeper_resolver.c \

PUBLIC_HEADERS_C += \
    include/grpc/grpc_zookeeper.h \

LIBGRPC_ZOOKEEPER_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(LIBGRPC_ZOOKEEPER_SRC))))


$(LIBDIR)/$(CONFIG)/libgrpc_zookeeper.a: $(ZLIB_DEP) $(OPENSSL_DEP)  $(LIBGRPC_ZOOKEEPER_OBJS)
	$(E) "[AR]      Creating $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) rm -f $(LIBDIR)/$(CONFIG)/libgrpc_zookeeper.a
	$(Q) $(AR) rcs $(LIBDIR)/$(CONFIG)/libgrpc_zookeeper.a $(LIBGRPC_ZOOKEEPER_OBJS)
ifeq ($(SYSTEM),Darwin)
	$(Q) ranlib $(LIBDIR)/$(CONFIG)/libgrpc_zookeeper.a
endif



ifeq ($(SYSTEM),MINGW32)
$(LIBDIR)/$(CONFIG)/grpc_zookeeper.$(SHARED_EXT): $(LIBGRPC_ZOOKEEPER_OBJS)  $(ZLIB_DEP) $(LIBDIR)/$(CONFIG)/gpr.$(SHARED_EXT) $(LIBDIR)/$(CONFIG)/grpc.$(SHARED_EXT)
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) -L$(LIBDIR)/$(CONFIG) -shared -Wl,--output-def=$(LIBDIR)/$(CONFIG)/grpc_zookeeper.def -Wl,--out-implib=$(LIBDIR)/$(CONFIG)/libgrpc_zookeeper-imp.a -o $(LIBDIR)/$(CONFIG)/grpc_zookeeper.$(SHARED_EXT) $(LIBGRPC_ZOOKEEPER_OBJS) $(LDLIBS) $(ZLIB_MERGE_LIBS) -lgpr-imp -lgrpc-imp
else
$(LIBDIR)/$(CONFIG)/libgrpc_zookeeper.$(SHARED_EXT): $(LIBGRPC_ZOOKEEPER_OBJS)  $(ZLIB_DEP) $(LIBDIR)/$(CONFIG)/libgpr.$(SHARED_EXT) $(LIBDIR)/$(CONFIG)/libgrpc.$(SHARED_EXT)
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
ifeq ($(SYSTEM),Darwin)
	$(Q) $(LD) $(LDFLAGS) -L$(LIBDIR)/$(CONFIG) -install_name libgrpc_zookeeper.$(SHARED_EXT) -dynamiclib -o $(LIBDIR)/$(CONFIG)/libgrpc_zookeeper.$(SHARED_EXT) $(LIBGRPC_ZOOKEEPER_OBJS) $(LDLIBS) $(ZLIB_MERGE_LIBS) -lgpr -lgrpc -lzookeeper_mt
else
	$(Q) $(LD) $(LDFLAGS) -L$(LIBDIR)/$(CONFIG) -shared -Wl,-soname,libgrpc_zookeeper.so.0 -o $(LIBDIR)/$(CONFIG)/libgrpc_zookeeper.$(SHARED_EXT) $(LIBGRPC_ZOOKEEPER_OBJS) $(LDLIBS) $(ZLIB_MERGE_LIBS) -lgpr -lgrpc -lzookeeper_mt
	$(Q) ln -sf libgrpc_zookeeper.$(SHARED_EXT) $(LIBDIR)/$(CONFIG)/libgrpc_zookeeper.so.0
	$(Q) ln -sf libgrpc_zookeeper.$(SHARED_EXT) $(LIBDIR)/$(CONFIG)/libgrpc_zookeeper.so
endif
endif

ifneq ($(NO_DEPS),true)
-include $(LIBGRPC_ZOOKEEPER_OBJS:.o=.dep)
endif


LIBRECONNECT_SERVER_SRC = \
    test/core/util/reconnect_server.c \


LIBRECONNECT_SERVER_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(LIBRECONNECT_SERVER_SRC))))


ifeq ($(NO_SECURE),true)

# You can't build secure libraries if you don't have OpenSSL.

$(LIBDIR)/$(CONFIG)/libreconnect_server.a: openssl_dep_error


else


$(LIBDIR)/$(CONFIG)/libreconnect_server.a: $(ZLIB_DEP) $(OPENSSL_DEP) $(LIBRECONNECT_SERVER_OBJS)
	$(E) "[AR]      Creating $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) rm -f $(LIBDIR)/$(CONFIG)/libreconnect_server.a
	$(Q) $(AR) rcs $(LIBDIR)/$(CONFIG)/libreconnect_server.a $(LIBRECONNECT_SERVER_OBJS)
ifeq ($(SYSTEM),Darwin)
	$(Q) ranlib $(LIBDIR)/$(CONFIG)/libreconnect_server.a
endif




endif

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(LIBRECONNECT_SERVER_OBJS:.o=.dep)
endif
endif


LIBTEST_TCP_SERVER_SRC = \
    test/core/util/test_tcp_server.c \


LIBTEST_TCP_SERVER_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(LIBTEST_TCP_SERVER_SRC))))


ifeq ($(NO_SECURE),true)

# You can't build secure libraries if you don't have OpenSSL.

$(LIBDIR)/$(CONFIG)/libtest_tcp_server.a: openssl_dep_error


else


$(LIBDIR)/$(CONFIG)/libtest_tcp_server.a: $(ZLIB_DEP) $(OPENSSL_DEP) $(LIBTEST_TCP_SERVER_OBJS)
	$(E) "[AR]      Creating $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) rm -f $(LIBDIR)/$(CONFIG)/libtest_tcp_server.a
	$(Q) $(AR) rcs $(LIBDIR)/$(CONFIG)/libtest_tcp_server.a $(LIBTEST_TCP_SERVER_OBJS)
ifeq ($(SYSTEM),Darwin)
	$(Q) ranlib $(LIBDIR)/$(CONFIG)/libtest_tcp_server.a
endif




endif

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(LIBTEST_TCP_SERVER_OBJS:.o=.dep)
endif
endif


LIBGRPC++_SRC = \
    src/cpp/client/secure_credentials.cc \
    src/cpp/common/auth_property_iterator.cc \
    src/cpp/common/secure_auth_context.cc \
    src/cpp/common/secure_channel_arguments.cc \
    src/cpp/common/secure_create_auth_context.cc \
    src/cpp/server/secure_server_credentials.cc \
    src/cpp/client/channel.cc \
    src/cpp/client/client_context.cc \
    src/cpp/client/create_channel.cc \
    src/cpp/client/create_channel_internal.cc \
    src/cpp/client/credentials.cc \
    src/cpp/client/generic_stub.cc \
    src/cpp/client/insecure_credentials.cc \
    src/cpp/common/call.cc \
    src/cpp/common/channel_arguments.cc \
    src/cpp/common/completion_queue.cc \
    src/cpp/common/rpc_method.cc \
    src/cpp/proto/proto_utils.cc \
    src/cpp/server/async_generic_service.cc \
    src/cpp/server/create_default_thread_pool.cc \
    src/cpp/server/dynamic_thread_pool.cc \
    src/cpp/server/fixed_size_thread_pool.cc \
    src/cpp/server/insecure_server_credentials.cc \
    src/cpp/server/server.cc \
    src/cpp/server/server_builder.cc \
    src/cpp/server/server_context.cc \
    src/cpp/server/server_credentials.cc \
    src/cpp/util/byte_buffer.cc \
    src/cpp/util/slice.cc \
    src/cpp/util/status.cc \
    src/cpp/util/string_ref.cc \
    src/cpp/util/time.cc \

PUBLIC_HEADERS_CXX += \
    include/grpc++/channel.h \
    include/grpc++/client_context.h \
    include/grpc++/completion_queue.h \
    include/grpc++/create_channel.h \
    include/grpc++/generic/async_generic_service.h \
    include/grpc++/generic/generic_stub.h \
    include/grpc++/grpc++.h \
    include/grpc++/impl/call.h \
    include/grpc++/impl/client_unary_call.h \
    include/grpc++/impl/grpc_library.h \
    include/grpc++/impl/proto_utils.h \
    include/grpc++/impl/rpc_method.h \
    include/grpc++/impl/rpc_service_method.h \
    include/grpc++/impl/serialization_traits.h \
    include/grpc++/impl/server_builder_option.h \
    include/grpc++/impl/service_type.h \
    include/grpc++/impl/sync.h \
    include/grpc++/impl/sync_cxx11.h \
    include/grpc++/impl/sync_no_cxx11.h \
    include/grpc++/impl/thd.h \
    include/grpc++/impl/thd_cxx11.h \
    include/grpc++/impl/thd_no_cxx11.h \
    include/grpc++/security/auth_context.h \
    include/grpc++/security/auth_metadata_processor.h \
    include/grpc++/security/credentials.h \
    include/grpc++/security/server_credentials.h \
    include/grpc++/server.h \
    include/grpc++/server_builder.h \
    include/grpc++/server_context.h \
    include/grpc++/support/async_stream.h \
    include/grpc++/support/async_unary_call.h \
    include/grpc++/support/byte_buffer.h \
    include/grpc++/support/channel_arguments.h \
    include/grpc++/support/config.h \
    include/grpc++/support/config_protobuf.h \
    include/grpc++/support/slice.h \
    include/grpc++/support/status.h \
    include/grpc++/support/status_code_enum.h \
    include/grpc++/support/string_ref.h \
    include/grpc++/support/stub_options.h \
    include/grpc++/support/sync_stream.h \
    include/grpc++/support/time.h \

LIBGRPC++_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(LIBGRPC++_SRC))))


ifeq ($(NO_SECURE),true)

# You can't build secure libraries if you don't have OpenSSL.

$(LIBDIR)/$(CONFIG)/libgrpc++.a: openssl_dep_error

ifeq ($(SYSTEM),MINGW32)
$(LIBDIR)/$(CONFIG)/grpc++.$(SHARED_EXT): openssl_dep_error
else
$(LIBDIR)/$(CONFIG)/libgrpc++.$(SHARED_EXT): openssl_dep_error
endif

else

ifeq ($(NO_PROTOBUF),true)

# You can't build a C++ library if you don't have protobuf - a bit overreached, but still okay.

$(LIBDIR)/$(CONFIG)/libgrpc++.a: protobuf_dep_error

ifeq ($(SYSTEM),MINGW32)
$(LIBDIR)/$(CONFIG)/grpc++.$(SHARED_EXT): protobuf_dep_error
else
$(LIBDIR)/$(CONFIG)/libgrpc++.$(SHARED_EXT): protobuf_dep_error
endif

else

$(LIBDIR)/$(CONFIG)/libgrpc++.a: $(ZLIB_DEP) $(OPENSSL_DEP) $(PROTOBUF_DEP) $(LIBGRPC++_OBJS)
	$(E) "[AR]      Creating $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) rm -f $(LIBDIR)/$(CONFIG)/libgrpc++.a
	$(Q) $(AR) rcs $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBGRPC++_OBJS)
	$(Q) rm -rf $(BUILDDIR_ABSOLUTE)/tmp-merge-grpc++
	$(Q) ( mkdir -p $(BUILDDIR_ABSOLUTE)/tmp-merge-grpc++/grpc ; \
	       cd $(BUILDDIR_ABSOLUTE)/tmp-merge-grpc++/grpc ; \
	       $(AR) x $(LIBDIR)/$(CONFIG)/libgrpc++.a )
	$(Q) for l in $(ZLIB_MERGE_LIBS) ; do ( \
	       mkdir -p $(BUILDDIR_ABSOLUTE)/tmp-merge-grpc++/zlib ; \
	       cd $(BUILDDIR_ABSOLUTE)/tmp-merge-grpc++/zlib ; \
	       $(AR) x $${l} ) ; done
	$(Q) for l in $(ZLIB_MERGE_LIBS) ; do ( \
	       mkdir -p $(BUILDDIR_ABSOLUTE)/tmp-merge-grpc++/zlib ; \
	       cd $(BUILDDIR_ABSOLUTE)/tmp-merge-grpc++/zlib ; \
	       $(AR) x $${l} ) ; done
	$(Q) rm -f $(LIBDIR)/$(CONFIG)/libgrpc++.a $(BUILDDIR_ABSOLUTE)/tmp-merge-grpc++/*/__.SYMDEF*
	$(Q) ar rcs $(LIBDIR)/$(CONFIG)/libgrpc++.a $(BUILDDIR_ABSOLUTE)/tmp-merge-grpc++/*/*
	$(Q) rm -rf $(BUILDDIR_ABSOLUTE)/tmp-merge-grpc++
ifeq ($(SYSTEM),Darwin)
	$(Q) ranlib $(LIBDIR)/$(CONFIG)/libgrpc++.a
endif



ifeq ($(SYSTEM),MINGW32)
$(LIBDIR)/$(CONFIG)/grpc++.$(SHARED_EXT): $(LIBGRPC++_OBJS)  $(ZLIB_DEP) $(PROTOBUF_DEP) $(LIBDIR)/$(CONFIG)/gpr.$(SHARED_EXT) $(LIBDIR)/$(CONFIG)/grpc.$(SHARED_EXT) $(OPENSSL_DEP)
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS) -L$(LIBDIR)/$(CONFIG) -shared -Wl,--output-def=$(LIBDIR)/$(CONFIG)/grpc++.def -Wl,--out-implib=$(LIBDIR)/$(CONFIG)/libgrpc++-imp.a -o $(LIBDIR)/$(CONFIG)/grpc++.$(SHARED_EXT) $(LIBGRPC++_OBJS) $(LDLIBS) $(ZLIB_MERGE_LIBS) $(LDLIBSXX) $(LDLIBS_PROTOBUF) -lgpr-imp -lgrpc-imp
else
$(LIBDIR)/$(CONFIG)/libgrpc++.$(SHARED_EXT): $(LIBGRPC++_OBJS)  $(ZLIB_DEP) $(PROTOBUF_DEP) $(LIBDIR)/$(CONFIG)/libgpr.$(SHARED_EXT) $(LIBDIR)/$(CONFIG)/libgrpc.$(SHARED_EXT) $(OPENSSL_DEP)
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
ifeq ($(SYSTEM),Darwin)
	$(Q) $(LDXX) $(LDFLAGS) -L$(LIBDIR)/$(CONFIG) -install_name libgrpc++.$(SHARED_EXT) -dynamiclib -o $(LIBDIR)/$(CONFIG)/libgrpc++.$(SHARED_EXT) $(LIBGRPC++_OBJS) $(LDLIBS) $(ZLIB_MERGE_LIBS) $(LDLIBSXX) $(LDLIBS_PROTOBUF) -lgpr -lgrpc
else
	$(Q) $(LDXX) $(LDFLAGS) -L$(LIBDIR)/$(CONFIG) -shared -Wl,-soname,libgrpc++.so.0 -o $(LIBDIR)/$(CONFIG)/libgrpc++.$(SHARED_EXT) $(LIBGRPC++_OBJS) $(LDLIBS) $(ZLIB_MERGE_LIBS) $(LDLIBSXX) $(LDLIBS_PROTOBUF) -lgpr -lgrpc
	$(Q) ln -sf libgrpc++.$(SHARED_EXT) $(LIBDIR)/$(CONFIG)/libgrpc++.so.0
	$(Q) ln -sf libgrpc++.$(SHARED_EXT) $(LIBDIR)/$(CONFIG)/libgrpc++.so
endif
endif

endif

endif

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(LIBGRPC++_OBJS:.o=.dep)
endif
endif


LIBGRPC++_TEST_CONFIG_SRC = \
    test/cpp/util/test_config.cc \


LIBGRPC++_TEST_CONFIG_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(LIBGRPC++_TEST_CONFIG_SRC))))


ifeq ($(NO_SECURE),true)

# You can't build secure libraries if you don't have OpenSSL.

$(LIBDIR)/$(CONFIG)/libgrpc++_test_config.a: openssl_dep_error


else

ifeq ($(NO_PROTOBUF),true)

# You can't build a C++ library if you don't have protobuf - a bit overreached, but still okay.

$(LIBDIR)/$(CONFIG)/libgrpc++_test_config.a: protobuf_dep_error


else

$(LIBDIR)/$(CONFIG)/libgrpc++_test_config.a: $(ZLIB_DEP) $(OPENSSL_DEP) $(PROTOBUF_DEP) $(LIBGRPC++_TEST_CONFIG_OBJS)
	$(E) "[AR]      Creating $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) rm -f $(LIBDIR)/$(CONFIG)/libgrpc++_test_config.a
	$(Q) $(AR) rcs $(LIBDIR)/$(CONFIG)/libgrpc++_test_config.a $(LIBGRPC++_TEST_CONFIG_OBJS)
ifeq ($(SYSTEM),Darwin)
	$(Q) ranlib $(LIBDIR)/$(CONFIG)/libgrpc++_test_config.a
endif




endif

endif

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(LIBGRPC++_TEST_CONFIG_OBJS:.o=.dep)
endif
endif


LIBGRPC++_TEST_UTIL_SRC = \
    $(GENDIR)/test/cpp/util/messages.pb.cc $(GENDIR)/test/cpp/util/messages.grpc.pb.cc \
    $(GENDIR)/test/cpp/util/echo.pb.cc $(GENDIR)/test/cpp/util/echo.grpc.pb.cc \
    $(GENDIR)/test/cpp/util/echo_duplicate.pb.cc $(GENDIR)/test/cpp/util/echo_duplicate.grpc.pb.cc \
    test/cpp/util/cli_call.cc \
    test/cpp/util/create_test_channel.cc \
    test/cpp/util/string_ref_helper.cc \
    test/cpp/util/subprocess.cc \


LIBGRPC++_TEST_UTIL_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(LIBGRPC++_TEST_UTIL_SRC))))


ifeq ($(NO_SECURE),true)

# You can't build secure libraries if you don't have OpenSSL.

$(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a: openssl_dep_error


else

ifeq ($(NO_PROTOBUF),true)

# You can't build a C++ library if you don't have protobuf - a bit overreached, but still okay.

$(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a: protobuf_dep_error


else

$(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a: $(ZLIB_DEP) $(OPENSSL_DEP) $(PROTOBUF_DEP) $(LIBGRPC++_TEST_UTIL_OBJS)
	$(E) "[AR]      Creating $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) rm -f $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a
	$(Q) $(AR) rcs $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBGRPC++_TEST_UTIL_OBJS)
ifeq ($(SYSTEM),Darwin)
	$(Q) ranlib $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a
endif




endif

endif

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(LIBGRPC++_TEST_UTIL_OBJS:.o=.dep)
endif
endif
$(OBJDIR)/$(CONFIG)/test/cpp/util/cli_call.o: $(GENDIR)/test/cpp/util/messages.pb.cc $(GENDIR)/test/cpp/util/messages.grpc.pb.cc $(GENDIR)/test/cpp/util/echo.pb.cc $(GENDIR)/test/cpp/util/echo.grpc.pb.cc $(GENDIR)/test/cpp/util/echo_duplicate.pb.cc $(GENDIR)/test/cpp/util/echo_duplicate.grpc.pb.cc
$(OBJDIR)/$(CONFIG)/test/cpp/util/create_test_channel.o: $(GENDIR)/test/cpp/util/messages.pb.cc $(GENDIR)/test/cpp/util/messages.grpc.pb.cc $(GENDIR)/test/cpp/util/echo.pb.cc $(GENDIR)/test/cpp/util/echo.grpc.pb.cc $(GENDIR)/test/cpp/util/echo_duplicate.pb.cc $(GENDIR)/test/cpp/util/echo_duplicate.grpc.pb.cc
$(OBJDIR)/$(CONFIG)/test/cpp/util/string_ref_helper.o: $(GENDIR)/test/cpp/util/messages.pb.cc $(GENDIR)/test/cpp/util/messages.grpc.pb.cc $(GENDIR)/test/cpp/util/echo.pb.cc $(GENDIR)/test/cpp/util/echo.grpc.pb.cc $(GENDIR)/test/cpp/util/echo_duplicate.pb.cc $(GENDIR)/test/cpp/util/echo_duplicate.grpc.pb.cc
$(OBJDIR)/$(CONFIG)/test/cpp/util/subprocess.o: $(GENDIR)/test/cpp/util/messages.pb.cc $(GENDIR)/test/cpp/util/messages.grpc.pb.cc $(GENDIR)/test/cpp/util/echo.pb.cc $(GENDIR)/test/cpp/util/echo.grpc.pb.cc $(GENDIR)/test/cpp/util/echo_duplicate.pb.cc $(GENDIR)/test/cpp/util/echo_duplicate.grpc.pb.cc


LIBGRPC++_UNSECURE_SRC = \
    src/cpp/common/insecure_create_auth_context.cc \
    src/cpp/client/channel.cc \
    src/cpp/client/client_context.cc \
    src/cpp/client/create_channel.cc \
    src/cpp/client/create_channel_internal.cc \
    src/cpp/client/credentials.cc \
    src/cpp/client/generic_stub.cc \
    src/cpp/client/insecure_credentials.cc \
    src/cpp/common/call.cc \
    src/cpp/common/channel_arguments.cc \
    src/cpp/common/completion_queue.cc \
    src/cpp/common/rpc_method.cc \
    src/cpp/proto/proto_utils.cc \
    src/cpp/server/async_generic_service.cc \
    src/cpp/server/create_default_thread_pool.cc \
    src/cpp/server/dynamic_thread_pool.cc \
    src/cpp/server/fixed_size_thread_pool.cc \
    src/cpp/server/insecure_server_credentials.cc \
    src/cpp/server/server.cc \
    src/cpp/server/server_builder.cc \
    src/cpp/server/server_context.cc \
    src/cpp/server/server_credentials.cc \
    src/cpp/util/byte_buffer.cc \
    src/cpp/util/slice.cc \
    src/cpp/util/status.cc \
    src/cpp/util/string_ref.cc \
    src/cpp/util/time.cc \

PUBLIC_HEADERS_CXX += \
    include/grpc++/channel.h \
    include/grpc++/client_context.h \
    include/grpc++/completion_queue.h \
    include/grpc++/create_channel.h \
    include/grpc++/generic/async_generic_service.h \
    include/grpc++/generic/generic_stub.h \
    include/grpc++/grpc++.h \
    include/grpc++/impl/call.h \
    include/grpc++/impl/client_unary_call.h \
    include/grpc++/impl/grpc_library.h \
    include/grpc++/impl/proto_utils.h \
    include/grpc++/impl/rpc_method.h \
    include/grpc++/impl/rpc_service_method.h \
    include/grpc++/impl/serialization_traits.h \
    include/grpc++/impl/server_builder_option.h \
    include/grpc++/impl/service_type.h \
    include/grpc++/impl/sync.h \
    include/grpc++/impl/sync_cxx11.h \
    include/grpc++/impl/sync_no_cxx11.h \
    include/grpc++/impl/thd.h \
    include/grpc++/impl/thd_cxx11.h \
    include/grpc++/impl/thd_no_cxx11.h \
    include/grpc++/security/auth_context.h \
    include/grpc++/security/auth_metadata_processor.h \
    include/grpc++/security/credentials.h \
    include/grpc++/security/server_credentials.h \
    include/grpc++/server.h \
    include/grpc++/server_builder.h \
    include/grpc++/server_context.h \
    include/grpc++/support/async_stream.h \
    include/grpc++/support/async_unary_call.h \
    include/grpc++/support/byte_buffer.h \
    include/grpc++/support/channel_arguments.h \
    include/grpc++/support/config.h \
    include/grpc++/support/config_protobuf.h \
    include/grpc++/support/slice.h \
    include/grpc++/support/status.h \
    include/grpc++/support/status_code_enum.h \
    include/grpc++/support/string_ref.h \
    include/grpc++/support/stub_options.h \
    include/grpc++/support/sync_stream.h \
    include/grpc++/support/time.h \

LIBGRPC++_UNSECURE_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(LIBGRPC++_UNSECURE_SRC))))


ifeq ($(NO_PROTOBUF),true)

# You can't build a C++ library if you don't have protobuf - a bit overreached, but still okay.

$(LIBDIR)/$(CONFIG)/libgrpc++_unsecure.a: protobuf_dep_error

ifeq ($(SYSTEM),MINGW32)
$(LIBDIR)/$(CONFIG)/grpc++_unsecure.$(SHARED_EXT): protobuf_dep_error
else
$(LIBDIR)/$(CONFIG)/libgrpc++_unsecure.$(SHARED_EXT): protobuf_dep_error
endif

else

$(LIBDIR)/$(CONFIG)/libgrpc++_unsecure.a: $(ZLIB_DEP) $(OPENSSL_DEP)  $(PROTOBUF_DEP) $(LIBGRPC++_UNSECURE_OBJS)
	$(E) "[AR]      Creating $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) rm -f $(LIBDIR)/$(CONFIG)/libgrpc++_unsecure.a
	$(Q) $(AR) rcs $(LIBDIR)/$(CONFIG)/libgrpc++_unsecure.a $(LIBGRPC++_UNSECURE_OBJS)
	$(Q) rm -rf $(BUILDDIR_ABSOLUTE)/tmp-merge-grpc++_unsecure
	$(Q) ( mkdir -p $(BUILDDIR_ABSOLUTE)/tmp-merge-grpc++_unsecure/grpc ; \
	       cd $(BUILDDIR_ABSOLUTE)/tmp-merge-grpc++_unsecure/grpc ; \
	       $(AR) x $(LIBDIR)/$(CONFIG)/libgrpc++_unsecure.a )
	$(Q) for l in $(ZLIB_MERGE_LIBS) ; do ( \
	       mkdir -p $(BUILDDIR_ABSOLUTE)/tmp-merge-grpc++_unsecure/zlib ; \
	       cd $(BUILDDIR_ABSOLUTE)/tmp-merge-grpc++_unsecure/zlib ; \
	       $(AR) x $${l} ) ; done
	$(Q) for l in $(ZLIB_MERGE_LIBS) ; do ( \
	       mkdir -p $(BUILDDIR_ABSOLUTE)/tmp-merge-grpc++_unsecure/zlib ; \
	       cd $(BUILDDIR_ABSOLUTE)/tmp-merge-grpc++_unsecure/zlib ; \
	       $(AR) x $${l} ) ; done
	$(Q) rm -f $(LIBDIR)/$(CONFIG)/libgrpc++_unsecure.a $(BUILDDIR_ABSOLUTE)/tmp-merge-grpc++_unsecure/*/__.SYMDEF*
	$(Q) ar rcs $(LIBDIR)/$(CONFIG)/libgrpc++_unsecure.a $(BUILDDIR_ABSOLUTE)/tmp-merge-grpc++_unsecure/*/*
	$(Q) rm -rf $(BUILDDIR_ABSOLUTE)/tmp-merge-grpc++_unsecure
ifeq ($(SYSTEM),Darwin)
	$(Q) ranlib $(LIBDIR)/$(CONFIG)/libgrpc++_unsecure.a
endif



ifeq ($(SYSTEM),MINGW32)
$(LIBDIR)/$(CONFIG)/grpc++_unsecure.$(SHARED_EXT): $(LIBGRPC++_UNSECURE_OBJS)  $(ZLIB_DEP) $(PROTOBUF_DEP) $(LIBDIR)/$(CONFIG)/gpr.$(SHARED_EXT) $(LIBDIR)/$(CONFIG)/grpc_unsecure.$(SHARED_EXT)
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS) -L$(LIBDIR)/$(CONFIG) -shared -Wl,--output-def=$(LIBDIR)/$(CONFIG)/grpc++_unsecure.def -Wl,--out-implib=$(LIBDIR)/$(CONFIG)/libgrpc++_unsecure-imp.a -o $(LIBDIR)/$(CONFIG)/grpc++_unsecure.$(SHARED_EXT) $(LIBGRPC++_UNSECURE_OBJS) $(LDLIBS) $(ZLIB_MERGE_LIBS) $(LDLIBSXX) $(LDLIBS_PROTOBUF) -lgpr-imp -lgrpc_unsecure-imp
else
$(LIBDIR)/$(CONFIG)/libgrpc++_unsecure.$(SHARED_EXT): $(LIBGRPC++_UNSECURE_OBJS)  $(ZLIB_DEP) $(PROTOBUF_DEP) $(LIBDIR)/$(CONFIG)/libgpr.$(SHARED_EXT) $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.$(SHARED_EXT)
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
ifeq ($(SYSTEM),Darwin)
	$(Q) $(LDXX) $(LDFLAGS) -L$(LIBDIR)/$(CONFIG) -install_name libgrpc++_unsecure.$(SHARED_EXT) -dynamiclib -o $(LIBDIR)/$(CONFIG)/libgrpc++_unsecure.$(SHARED_EXT) $(LIBGRPC++_UNSECURE_OBJS) $(LDLIBS) $(ZLIB_MERGE_LIBS) $(LDLIBSXX) $(LDLIBS_PROTOBUF) -lgpr -lgrpc_unsecure
else
	$(Q) $(LDXX) $(LDFLAGS) -L$(LIBDIR)/$(CONFIG) -shared -Wl,-soname,libgrpc++_unsecure.so.0 -o $(LIBDIR)/$(CONFIG)/libgrpc++_unsecure.$(SHARED_EXT) $(LIBGRPC++_UNSECURE_OBJS) $(LDLIBS) $(ZLIB_MERGE_LIBS) $(LDLIBSXX) $(LDLIBS_PROTOBUF) -lgpr -lgrpc_unsecure
	$(Q) ln -sf libgrpc++_unsecure.$(SHARED_EXT) $(LIBDIR)/$(CONFIG)/libgrpc++_unsecure.so.0
	$(Q) ln -sf libgrpc++_unsecure.$(SHARED_EXT) $(LIBDIR)/$(CONFIG)/libgrpc++_unsecure.so
endif
endif

endif

ifneq ($(NO_DEPS),true)
-include $(LIBGRPC++_UNSECURE_OBJS:.o=.dep)
endif


LIBGRPC_PLUGIN_SUPPORT_SRC = \
    src/compiler/cpp_generator.cc \
    src/compiler/csharp_generator.cc \
    src/compiler/objective_c_generator.cc \
    src/compiler/python_generator.cc \
    src/compiler/ruby_generator.cc \


LIBGRPC_PLUGIN_SUPPORT_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(LIBGRPC_PLUGIN_SUPPORT_SRC))))


ifeq ($(NO_PROTOBUF),true)

# You can't build a C++ library if you don't have protobuf - a bit overreached, but still okay.

$(LIBDIR)/$(CONFIG)/libgrpc_plugin_support.a: protobuf_dep_error


else

$(LIBDIR)/$(CONFIG)/libgrpc_plugin_support.a: $(ZLIB_DEP) $(OPENSSL_DEP)  $(PROTOBUF_DEP) $(LIBGRPC_PLUGIN_SUPPORT_OBJS)
	$(E) "[AR]      Creating $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) rm -f $(LIBDIR)/$(CONFIG)/libgrpc_plugin_support.a
	$(Q) $(AR) rcs $(LIBDIR)/$(CONFIG)/libgrpc_plugin_support.a $(LIBGRPC_PLUGIN_SUPPORT_OBJS)
ifeq ($(SYSTEM),Darwin)
	$(Q) ranlib $(LIBDIR)/$(CONFIG)/libgrpc_plugin_support.a
endif




endif

ifneq ($(NO_DEPS),true)
-include $(LIBGRPC_PLUGIN_SUPPORT_OBJS:.o=.dep)
endif


LIBINTEROP_CLIENT_HELPER_SRC = \
    $(GENDIR)/test/proto/messages.pb.cc $(GENDIR)/test/proto/messages.grpc.pb.cc \
    test/cpp/interop/client_helper.cc \


LIBINTEROP_CLIENT_HELPER_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(LIBINTEROP_CLIENT_HELPER_SRC))))


ifeq ($(NO_SECURE),true)

# You can't build secure libraries if you don't have OpenSSL.

$(LIBDIR)/$(CONFIG)/libinterop_client_helper.a: openssl_dep_error


else

ifeq ($(NO_PROTOBUF),true)

# You can't build a C++ library if you don't have protobuf - a bit overreached, but still okay.

$(LIBDIR)/$(CONFIG)/libinterop_client_helper.a: protobuf_dep_error


else

$(LIBDIR)/$(CONFIG)/libinterop_client_helper.a: $(ZLIB_DEP) $(OPENSSL_DEP) $(PROTOBUF_DEP) $(LIBINTEROP_CLIENT_HELPER_OBJS)
	$(E) "[AR]      Creating $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) rm -f $(LIBDIR)/$(CONFIG)/libinterop_client_helper.a
	$(Q) $(AR) rcs $(LIBDIR)/$(CONFIG)/libinterop_client_helper.a $(LIBINTEROP_CLIENT_HELPER_OBJS)
ifeq ($(SYSTEM),Darwin)
	$(Q) ranlib $(LIBDIR)/$(CONFIG)/libinterop_client_helper.a
endif




endif

endif

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(LIBINTEROP_CLIENT_HELPER_OBJS:.o=.dep)
endif
endif
$(OBJDIR)/$(CONFIG)/test/cpp/interop/client_helper.o: $(GENDIR)/test/proto/messages.pb.cc $(GENDIR)/test/proto/messages.grpc.pb.cc


LIBINTEROP_CLIENT_MAIN_SRC = \
    $(GENDIR)/test/proto/empty.pb.cc $(GENDIR)/test/proto/empty.grpc.pb.cc \
    $(GENDIR)/test/proto/messages.pb.cc $(GENDIR)/test/proto/messages.grpc.pb.cc \
    $(GENDIR)/test/proto/test.pb.cc $(GENDIR)/test/proto/test.grpc.pb.cc \
    test/cpp/interop/client.cc \
    test/cpp/interop/interop_client.cc \


LIBINTEROP_CLIENT_MAIN_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(LIBINTEROP_CLIENT_MAIN_SRC))))


ifeq ($(NO_SECURE),true)

# You can't build secure libraries if you don't have OpenSSL.

$(LIBDIR)/$(CONFIG)/libinterop_client_main.a: openssl_dep_error


else

ifeq ($(NO_PROTOBUF),true)

# You can't build a C++ library if you don't have protobuf - a bit overreached, but still okay.

$(LIBDIR)/$(CONFIG)/libinterop_client_main.a: protobuf_dep_error


else

$(LIBDIR)/$(CONFIG)/libinterop_client_main.a: $(ZLIB_DEP) $(OPENSSL_DEP) $(PROTOBUF_DEP) $(LIBINTEROP_CLIENT_MAIN_OBJS)
	$(E) "[AR]      Creating $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) rm -f $(LIBDIR)/$(CONFIG)/libinterop_client_main.a
	$(Q) $(AR) rcs $(LIBDIR)/$(CONFIG)/libinterop_client_main.a $(LIBINTEROP_CLIENT_MAIN_OBJS)
ifeq ($(SYSTEM),Darwin)
	$(Q) ranlib $(LIBDIR)/$(CONFIG)/libinterop_client_main.a
endif




endif

endif

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(LIBINTEROP_CLIENT_MAIN_OBJS:.o=.dep)
endif
endif
$(OBJDIR)/$(CONFIG)/test/cpp/interop/client.o: $(GENDIR)/test/proto/empty.pb.cc $(GENDIR)/test/proto/empty.grpc.pb.cc $(GENDIR)/test/proto/messages.pb.cc $(GENDIR)/test/proto/messages.grpc.pb.cc $(GENDIR)/test/proto/test.pb.cc $(GENDIR)/test/proto/test.grpc.pb.cc
$(OBJDIR)/$(CONFIG)/test/cpp/interop/interop_client.o: $(GENDIR)/test/proto/empty.pb.cc $(GENDIR)/test/proto/empty.grpc.pb.cc $(GENDIR)/test/proto/messages.pb.cc $(GENDIR)/test/proto/messages.grpc.pb.cc $(GENDIR)/test/proto/test.pb.cc $(GENDIR)/test/proto/test.grpc.pb.cc


LIBINTEROP_SERVER_HELPER_SRC = \
    test/cpp/interop/server_helper.cc \


LIBINTEROP_SERVER_HELPER_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(LIBINTEROP_SERVER_HELPER_SRC))))


ifeq ($(NO_SECURE),true)

# You can't build secure libraries if you don't have OpenSSL.

$(LIBDIR)/$(CONFIG)/libinterop_server_helper.a: openssl_dep_error


else

ifeq ($(NO_PROTOBUF),true)

# You can't build a C++ library if you don't have protobuf - a bit overreached, but still okay.

$(LIBDIR)/$(CONFIG)/libinterop_server_helper.a: protobuf_dep_error


else

$(LIBDIR)/$(CONFIG)/libinterop_server_helper.a: $(ZLIB_DEP) $(OPENSSL_DEP) $(PROTOBUF_DEP) $(LIBINTEROP_SERVER_HELPER_OBJS)
	$(E) "[AR]      Creating $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) rm -f $(LIBDIR)/$(CONFIG)/libinterop_server_helper.a
	$(Q) $(AR) rcs $(LIBDIR)/$(CONFIG)/libinterop_server_helper.a $(LIBINTEROP_SERVER_HELPER_OBJS)
ifeq ($(SYSTEM),Darwin)
	$(Q) ranlib $(LIBDIR)/$(CONFIG)/libinterop_server_helper.a
endif




endif

endif

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(LIBINTEROP_SERVER_HELPER_OBJS:.o=.dep)
endif
endif


LIBINTEROP_SERVER_MAIN_SRC = \
    $(GENDIR)/test/proto/empty.pb.cc $(GENDIR)/test/proto/empty.grpc.pb.cc \
    $(GENDIR)/test/proto/messages.pb.cc $(GENDIR)/test/proto/messages.grpc.pb.cc \
    $(GENDIR)/test/proto/test.pb.cc $(GENDIR)/test/proto/test.grpc.pb.cc \
    test/cpp/interop/server.cc \


LIBINTEROP_SERVER_MAIN_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(LIBINTEROP_SERVER_MAIN_SRC))))


ifeq ($(NO_SECURE),true)

# You can't build secure libraries if you don't have OpenSSL.

$(LIBDIR)/$(CONFIG)/libinterop_server_main.a: openssl_dep_error


else

ifeq ($(NO_PROTOBUF),true)

# You can't build a C++ library if you don't have protobuf - a bit overreached, but still okay.

$(LIBDIR)/$(CONFIG)/libinterop_server_main.a: protobuf_dep_error


else

$(LIBDIR)/$(CONFIG)/libinterop_server_main.a: $(ZLIB_DEP) $(OPENSSL_DEP) $(PROTOBUF_DEP) $(LIBINTEROP_SERVER_MAIN_OBJS)
	$(E) "[AR]      Creating $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) rm -f $(LIBDIR)/$(CONFIG)/libinterop_server_main.a
	$(Q) $(AR) rcs $(LIBDIR)/$(CONFIG)/libinterop_server_main.a $(LIBINTEROP_SERVER_MAIN_OBJS)
ifeq ($(SYSTEM),Darwin)
	$(Q) ranlib $(LIBDIR)/$(CONFIG)/libinterop_server_main.a
endif




endif

endif

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(LIBINTEROP_SERVER_MAIN_OBJS:.o=.dep)
endif
endif
$(OBJDIR)/$(CONFIG)/test/cpp/interop/server.o: $(GENDIR)/test/proto/empty.pb.cc $(GENDIR)/test/proto/empty.grpc.pb.cc $(GENDIR)/test/proto/messages.pb.cc $(GENDIR)/test/proto/messages.grpc.pb.cc $(GENDIR)/test/proto/test.pb.cc $(GENDIR)/test/proto/test.grpc.pb.cc


LIBQPS_SRC = \
    $(GENDIR)/test/proto/messages.pb.cc $(GENDIR)/test/proto/messages.grpc.pb.cc \
    $(GENDIR)/test/proto/benchmarks/control.pb.cc $(GENDIR)/test/proto/benchmarks/control.grpc.pb.cc \
    $(GENDIR)/test/proto/benchmarks/payloads.pb.cc $(GENDIR)/test/proto/benchmarks/payloads.grpc.pb.cc \
    $(GENDIR)/test/proto/benchmarks/services.pb.cc $(GENDIR)/test/proto/benchmarks/services.grpc.pb.cc \
    $(GENDIR)/test/proto/benchmarks/stats.pb.cc $(GENDIR)/test/proto/benchmarks/stats.grpc.pb.cc \
    $(GENDIR)/test/cpp/qps/perf_db.pb.cc $(GENDIR)/test/cpp/qps/perf_db.grpc.pb.cc \
    test/cpp/qps/client_async.cc \
    test/cpp/qps/client_sync.cc \
    test/cpp/qps/driver.cc \
    test/cpp/qps/perf_db_client.cc \
    test/cpp/qps/qps_worker.cc \
    test/cpp/qps/report.cc \
    test/cpp/qps/server_async.cc \
    test/cpp/qps/server_sync.cc \
    test/cpp/qps/timer.cc \
    test/cpp/util/benchmark_config.cc \


LIBQPS_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(LIBQPS_SRC))))


ifeq ($(NO_SECURE),true)

# You can't build secure libraries if you don't have OpenSSL.

$(LIBDIR)/$(CONFIG)/libqps.a: openssl_dep_error


else

ifeq ($(NO_PROTOBUF),true)

# You can't build a C++ library if you don't have protobuf - a bit overreached, but still okay.

$(LIBDIR)/$(CONFIG)/libqps.a: protobuf_dep_error


else

$(LIBDIR)/$(CONFIG)/libqps.a: $(ZLIB_DEP) $(OPENSSL_DEP) $(PROTOBUF_DEP) $(LIBQPS_OBJS)
	$(E) "[AR]      Creating $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) rm -f $(LIBDIR)/$(CONFIG)/libqps.a
	$(Q) $(AR) rcs $(LIBDIR)/$(CONFIG)/libqps.a $(LIBQPS_OBJS)
ifeq ($(SYSTEM),Darwin)
	$(Q) ranlib $(LIBDIR)/$(CONFIG)/libqps.a
endif




endif

endif

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(LIBQPS_OBJS:.o=.dep)
endif
endif
$(OBJDIR)/$(CONFIG)/test/cpp/qps/client_async.o: $(GENDIR)/test/proto/messages.pb.cc $(GENDIR)/test/proto/messages.grpc.pb.cc $(GENDIR)/test/proto/benchmarks/control.pb.cc $(GENDIR)/test/proto/benchmarks/control.grpc.pb.cc $(GENDIR)/test/proto/benchmarks/payloads.pb.cc $(GENDIR)/test/proto/benchmarks/payloads.grpc.pb.cc $(GENDIR)/test/proto/benchmarks/services.pb.cc $(GENDIR)/test/proto/benchmarks/services.grpc.pb.cc $(GENDIR)/test/proto/benchmarks/stats.pb.cc $(GENDIR)/test/proto/benchmarks/stats.grpc.pb.cc $(GENDIR)/test/cpp/qps/perf_db.pb.cc $(GENDIR)/test/cpp/qps/perf_db.grpc.pb.cc
$(OBJDIR)/$(CONFIG)/test/cpp/qps/client_sync.o: $(GENDIR)/test/proto/messages.pb.cc $(GENDIR)/test/proto/messages.grpc.pb.cc $(GENDIR)/test/proto/benchmarks/control.pb.cc $(GENDIR)/test/proto/benchmarks/control.grpc.pb.cc $(GENDIR)/test/proto/benchmarks/payloads.pb.cc $(GENDIR)/test/proto/benchmarks/payloads.grpc.pb.cc $(GENDIR)/test/proto/benchmarks/services.pb.cc $(GENDIR)/test/proto/benchmarks/services.grpc.pb.cc $(GENDIR)/test/proto/benchmarks/stats.pb.cc $(GENDIR)/test/proto/benchmarks/stats.grpc.pb.cc $(GENDIR)/test/cpp/qps/perf_db.pb.cc $(GENDIR)/test/cpp/qps/perf_db.grpc.pb.cc
$(OBJDIR)/$(CONFIG)/test/cpp/qps/driver.o: $(GENDIR)/test/proto/messages.pb.cc $(GENDIR)/test/proto/messages.grpc.pb.cc $(GENDIR)/test/proto/benchmarks/control.pb.cc $(GENDIR)/test/proto/benchmarks/control.grpc.pb.cc $(GENDIR)/test/proto/benchmarks/payloads.pb.cc $(GENDIR)/test/proto/benchmarks/payloads.grpc.pb.cc $(GENDIR)/test/proto/benchmarks/services.pb.cc $(GENDIR)/test/proto/benchmarks/services.grpc.pb.cc $(GENDIR)/test/proto/benchmarks/stats.pb.cc $(GENDIR)/test/proto/benchmarks/stats.grpc.pb.cc $(GENDIR)/test/cpp/qps/perf_db.pb.cc $(GENDIR)/test/cpp/qps/perf_db.grpc.pb.cc
$(OBJDIR)/$(CONFIG)/test/cpp/qps/perf_db_client.o: $(GENDIR)/test/proto/messages.pb.cc $(GENDIR)/test/proto/messages.grpc.pb.cc $(GENDIR)/test/proto/benchmarks/control.pb.cc $(GENDIR)/test/proto/benchmarks/control.grpc.pb.cc $(GENDIR)/test/proto/benchmarks/payloads.pb.cc $(GENDIR)/test/proto/benchmarks/payloads.grpc.pb.cc $(GENDIR)/test/proto/benchmarks/services.pb.cc $(GENDIR)/test/proto/benchmarks/services.grpc.pb.cc $(GENDIR)/test/proto/benchmarks/stats.pb.cc $(GENDIR)/test/proto/benchmarks/stats.grpc.pb.cc $(GENDIR)/test/cpp/qps/perf_db.pb.cc $(GENDIR)/test/cpp/qps/perf_db.grpc.pb.cc
$(OBJDIR)/$(CONFIG)/test/cpp/qps/qps_worker.o: $(GENDIR)/test/proto/messages.pb.cc $(GENDIR)/test/proto/messages.grpc.pb.cc $(GENDIR)/test/proto/benchmarks/control.pb.cc $(GENDIR)/test/proto/benchmarks/control.grpc.pb.cc $(GENDIR)/test/proto/benchmarks/payloads.pb.cc $(GENDIR)/test/proto/benchmarks/payloads.grpc.pb.cc $(GENDIR)/test/proto/benchmarks/services.pb.cc $(GENDIR)/test/proto/benchmarks/services.grpc.pb.cc $(GENDIR)/test/proto/benchmarks/stats.pb.cc $(GENDIR)/test/proto/benchmarks/stats.grpc.pb.cc $(GENDIR)/test/cpp/qps/perf_db.pb.cc $(GENDIR)/test/cpp/qps/perf_db.grpc.pb.cc
$(OBJDIR)/$(CONFIG)/test/cpp/qps/report.o: $(GENDIR)/test/proto/messages.pb.cc $(GENDIR)/test/proto/messages.grpc.pb.cc $(GENDIR)/test/proto/benchmarks/control.pb.cc $(GENDIR)/test/proto/benchmarks/control.grpc.pb.cc $(GENDIR)/test/proto/benchmarks/payloads.pb.cc $(GENDIR)/test/proto/benchmarks/payloads.grpc.pb.cc $(GENDIR)/test/proto/benchmarks/services.pb.cc $(GENDIR)/test/proto/benchmarks/services.grpc.pb.cc $(GENDIR)/test/proto/benchmarks/stats.pb.cc $(GENDIR)/test/proto/benchmarks/stats.grpc.pb.cc $(GENDIR)/test/cpp/qps/perf_db.pb.cc $(GENDIR)/test/cpp/qps/perf_db.grpc.pb.cc
$(OBJDIR)/$(CONFIG)/test/cpp/qps/server_async.o: $(GENDIR)/test/proto/messages.pb.cc $(GENDIR)/test/proto/messages.grpc.pb.cc $(GENDIR)/test/proto/benchmarks/control.pb.cc $(GENDIR)/test/proto/benchmarks/control.grpc.pb.cc $(GENDIR)/test/proto/benchmarks/payloads.pb.cc $(GENDIR)/test/proto/benchmarks/payloads.grpc.pb.cc $(GENDIR)/test/proto/benchmarks/services.pb.cc $(GENDIR)/test/proto/benchmarks/services.grpc.pb.cc $(GENDIR)/test/proto/benchmarks/stats.pb.cc $(GENDIR)/test/proto/benchmarks/stats.grpc.pb.cc $(GENDIR)/test/cpp/qps/perf_db.pb.cc $(GENDIR)/test/cpp/qps/perf_db.grpc.pb.cc
$(OBJDIR)/$(CONFIG)/test/cpp/qps/server_sync.o: $(GENDIR)/test/proto/messages.pb.cc $(GENDIR)/test/proto/messages.grpc.pb.cc $(GENDIR)/test/proto/benchmarks/control.pb.cc $(GENDIR)/test/proto/benchmarks/control.grpc.pb.cc $(GENDIR)/test/proto/benchmarks/payloads.pb.cc $(GENDIR)/test/proto/benchmarks/payloads.grpc.pb.cc $(GENDIR)/test/proto/benchmarks/services.pb.cc $(GENDIR)/test/proto/benchmarks/services.grpc.pb.cc $(GENDIR)/test/proto/benchmarks/stats.pb.cc $(GENDIR)/test/proto/benchmarks/stats.grpc.pb.cc $(GENDIR)/test/cpp/qps/perf_db.pb.cc $(GENDIR)/test/cpp/qps/perf_db.grpc.pb.cc
$(OBJDIR)/$(CONFIG)/test/cpp/qps/timer.o: $(GENDIR)/test/proto/messages.pb.cc $(GENDIR)/test/proto/messages.grpc.pb.cc $(GENDIR)/test/proto/benchmarks/control.pb.cc $(GENDIR)/test/proto/benchmarks/control.grpc.pb.cc $(GENDIR)/test/proto/benchmarks/payloads.pb.cc $(GENDIR)/test/proto/benchmarks/payloads.grpc.pb.cc $(GENDIR)/test/proto/benchmarks/services.pb.cc $(GENDIR)/test/proto/benchmarks/services.grpc.pb.cc $(GENDIR)/test/proto/benchmarks/stats.pb.cc $(GENDIR)/test/proto/benchmarks/stats.grpc.pb.cc $(GENDIR)/test/cpp/qps/perf_db.pb.cc $(GENDIR)/test/cpp/qps/perf_db.grpc.pb.cc
$(OBJDIR)/$(CONFIG)/test/cpp/util/benchmark_config.o: $(GENDIR)/test/proto/messages.pb.cc $(GENDIR)/test/proto/messages.grpc.pb.cc $(GENDIR)/test/proto/benchmarks/control.pb.cc $(GENDIR)/test/proto/benchmarks/control.grpc.pb.cc $(GENDIR)/test/proto/benchmarks/payloads.pb.cc $(GENDIR)/test/proto/benchmarks/payloads.grpc.pb.cc $(GENDIR)/test/proto/benchmarks/services.pb.cc $(GENDIR)/test/proto/benchmarks/services.grpc.pb.cc $(GENDIR)/test/proto/benchmarks/stats.pb.cc $(GENDIR)/test/proto/benchmarks/stats.grpc.pb.cc $(GENDIR)/test/cpp/qps/perf_db.pb.cc $(GENDIR)/test/cpp/qps/perf_db.grpc.pb.cc


LIBGRPC_CSHARP_EXT_SRC = \
    src/csharp/ext/grpc_csharp_ext.c \


LIBGRPC_CSHARP_EXT_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(LIBGRPC_CSHARP_EXT_SRC))))


ifeq ($(NO_SECURE),true)

# You can't build secure libraries if you don't have OpenSSL.

$(LIBDIR)/$(CONFIG)/libgrpc_csharp_ext.a: openssl_dep_error

ifeq ($(SYSTEM),MINGW32)
$(LIBDIR)/$(CONFIG)/grpc_csharp_ext.$(SHARED_EXT): openssl_dep_error
else
$(LIBDIR)/$(CONFIG)/libgrpc_csharp_ext.$(SHARED_EXT): openssl_dep_error
endif

else


$(LIBDIR)/$(CONFIG)/libgrpc_csharp_ext.a: $(ZLIB_DEP) $(OPENSSL_DEP) $(LIBGRPC_CSHARP_EXT_OBJS)
	$(E) "[AR]      Creating $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) rm -f $(LIBDIR)/$(CONFIG)/libgrpc_csharp_ext.a
	$(Q) $(AR) rcs $(LIBDIR)/$(CONFIG)/libgrpc_csharp_ext.a $(LIBGRPC_CSHARP_EXT_OBJS)
ifeq ($(SYSTEM),Darwin)
	$(Q) ranlib $(LIBDIR)/$(CONFIG)/libgrpc_csharp_ext.a
endif



ifeq ($(SYSTEM),MINGW32)
$(LIBDIR)/$(CONFIG)/grpc_csharp_ext.$(SHARED_EXT): $(LIBGRPC_CSHARP_EXT_OBJS)  $(ZLIB_DEP) $(LIBDIR)/$(CONFIG)/gpr.$(SHARED_EXT) $(LIBDIR)/$(CONFIG)/grpc.$(SHARED_EXT) $(OPENSSL_DEP)
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) -L$(LIBDIR)/$(CONFIG) -shared -Wl,--output-def=$(LIBDIR)/$(CONFIG)/grpc_csharp_ext.def -Wl,--out-implib=$(LIBDIR)/$(CONFIG)/libgrpc_csharp_ext-imp.a -o $(LIBDIR)/$(CONFIG)/grpc_csharp_ext.$(SHARED_EXT) $(LIBGRPC_CSHARP_EXT_OBJS) $(LDLIBS) $(ZLIB_MERGE_LIBS) -lgpr-imp -lgrpc-imp
else
$(LIBDIR)/$(CONFIG)/libgrpc_csharp_ext.$(SHARED_EXT): $(LIBGRPC_CSHARP_EXT_OBJS)  $(ZLIB_DEP) $(LIBDIR)/$(CONFIG)/libgpr.$(SHARED_EXT) $(LIBDIR)/$(CONFIG)/libgrpc.$(SHARED_EXT) $(OPENSSL_DEP)
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
ifeq ($(SYSTEM),Darwin)
	$(Q) $(LD) $(LDFLAGS) -L$(LIBDIR)/$(CONFIG) -install_name libgrpc_csharp_ext.$(SHARED_EXT) -dynamiclib -o $(LIBDIR)/$(CONFIG)/libgrpc_csharp_ext.$(SHARED_EXT) $(LIBGRPC_CSHARP_EXT_OBJS) $(LDLIBS) $(ZLIB_MERGE_LIBS) -lgpr -lgrpc
else
	$(Q) $(LD) $(LDFLAGS) -L$(LIBDIR)/$(CONFIG) -shared -Wl,-soname,libgrpc_csharp_ext.so.0 -o $(LIBDIR)/$(CONFIG)/libgrpc_csharp_ext.$(SHARED_EXT) $(LIBGRPC_CSHARP_EXT_OBJS) $(LDLIBS) $(ZLIB_MERGE_LIBS) -lgpr -lgrpc
	$(Q) ln -sf libgrpc_csharp_ext.$(SHARED_EXT) $(LIBDIR)/$(CONFIG)/libgrpc_csharp_ext.so.0
	$(Q) ln -sf libgrpc_csharp_ext.$(SHARED_EXT) $(LIBDIR)/$(CONFIG)/libgrpc_csharp_ext.so
endif
endif

endif

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(LIBGRPC_CSHARP_EXT_OBJS:.o=.dep)
endif
endif


LIBBORINGSSL_SRC = \
    src/boringssl/err_data.c \
    third_party/boringssl/crypto/aes/aes.c \
    third_party/boringssl/crypto/aes/mode_wrappers.c \
    third_party/boringssl/crypto/asn1/a_bitstr.c \
    third_party/boringssl/crypto/asn1/a_bool.c \
    third_party/boringssl/crypto/asn1/a_bytes.c \
    third_party/boringssl/crypto/asn1/a_d2i_fp.c \
    third_party/boringssl/crypto/asn1/a_dup.c \
    third_party/boringssl/crypto/asn1/a_enum.c \
    third_party/boringssl/crypto/asn1/a_gentm.c \
    third_party/boringssl/crypto/asn1/a_i2d_fp.c \
    third_party/boringssl/crypto/asn1/a_int.c \
    third_party/boringssl/crypto/asn1/a_mbstr.c \
    third_party/boringssl/crypto/asn1/a_object.c \
    third_party/boringssl/crypto/asn1/a_octet.c \
    third_party/boringssl/crypto/asn1/a_print.c \
    third_party/boringssl/crypto/asn1/a_strnid.c \
    third_party/boringssl/crypto/asn1/a_time.c \
    third_party/boringssl/crypto/asn1/a_type.c \
    third_party/boringssl/crypto/asn1/a_utctm.c \
    third_party/boringssl/crypto/asn1/a_utf8.c \
    third_party/boringssl/crypto/asn1/asn1_lib.c \
    third_party/boringssl/crypto/asn1/asn1_par.c \
    third_party/boringssl/crypto/asn1/asn_pack.c \
    third_party/boringssl/crypto/asn1/bio_asn1.c \
    third_party/boringssl/crypto/asn1/bio_ndef.c \
    third_party/boringssl/crypto/asn1/f_enum.c \
    third_party/boringssl/crypto/asn1/f_int.c \
    third_party/boringssl/crypto/asn1/f_string.c \
    third_party/boringssl/crypto/asn1/t_bitst.c \
    third_party/boringssl/crypto/asn1/t_pkey.c \
    third_party/boringssl/crypto/asn1/tasn_dec.c \
    third_party/boringssl/crypto/asn1/tasn_enc.c \
    third_party/boringssl/crypto/asn1/tasn_fre.c \
    third_party/boringssl/crypto/asn1/tasn_new.c \
    third_party/boringssl/crypto/asn1/tasn_prn.c \
    third_party/boringssl/crypto/asn1/tasn_typ.c \
    third_party/boringssl/crypto/asn1/tasn_utl.c \
    third_party/boringssl/crypto/asn1/x_bignum.c \
    third_party/boringssl/crypto/asn1/x_long.c \
    third_party/boringssl/crypto/base64/base64.c \
    third_party/boringssl/crypto/bio/bio.c \
    third_party/boringssl/crypto/bio/bio_mem.c \
    third_party/boringssl/crypto/bio/buffer.c \
    third_party/boringssl/crypto/bio/connect.c \
    third_party/boringssl/crypto/bio/fd.c \
    third_party/boringssl/crypto/bio/file.c \
    third_party/boringssl/crypto/bio/hexdump.c \
    third_party/boringssl/crypto/bio/pair.c \
    third_party/boringssl/crypto/bio/printf.c \
    third_party/boringssl/crypto/bio/socket.c \
    third_party/boringssl/crypto/bio/socket_helper.c \
    third_party/boringssl/crypto/bn/add.c \
    third_party/boringssl/crypto/bn/asm/x86_64-gcc.c \
    third_party/boringssl/crypto/bn/bn.c \
    third_party/boringssl/crypto/bn/bn_asn1.c \
    third_party/boringssl/crypto/bn/cmp.c \
    third_party/boringssl/crypto/bn/convert.c \
    third_party/boringssl/crypto/bn/ctx.c \
    third_party/boringssl/crypto/bn/div.c \
    third_party/boringssl/crypto/bn/exponentiation.c \
    third_party/boringssl/crypto/bn/gcd.c \
    third_party/boringssl/crypto/bn/generic.c \
    third_party/boringssl/crypto/bn/kronecker.c \
    third_party/boringssl/crypto/bn/montgomery.c \
    third_party/boringssl/crypto/bn/mul.c \
    third_party/boringssl/crypto/bn/prime.c \
    third_party/boringssl/crypto/bn/random.c \
    third_party/boringssl/crypto/bn/rsaz_exp.c \
    third_party/boringssl/crypto/bn/shift.c \
    third_party/boringssl/crypto/bn/sqrt.c \
    third_party/boringssl/crypto/buf/buf.c \
    third_party/boringssl/crypto/bytestring/ber.c \
    third_party/boringssl/crypto/bytestring/cbb.c \
    third_party/boringssl/crypto/bytestring/cbs.c \
    third_party/boringssl/crypto/chacha/chacha_generic.c \
    third_party/boringssl/crypto/chacha/chacha_vec.c \
    third_party/boringssl/crypto/cipher/aead.c \
    third_party/boringssl/crypto/cipher/cipher.c \
    third_party/boringssl/crypto/cipher/derive_key.c \
    third_party/boringssl/crypto/cipher/e_aes.c \
    third_party/boringssl/crypto/cipher/e_chacha20poly1305.c \
    third_party/boringssl/crypto/cipher/e_des.c \
    third_party/boringssl/crypto/cipher/e_null.c \
    third_party/boringssl/crypto/cipher/e_rc2.c \
    third_party/boringssl/crypto/cipher/e_rc4.c \
    third_party/boringssl/crypto/cipher/e_ssl3.c \
    third_party/boringssl/crypto/cipher/e_tls.c \
    third_party/boringssl/crypto/cipher/tls_cbc.c \
    third_party/boringssl/crypto/cmac/cmac.c \
    third_party/boringssl/crypto/conf/conf.c \
    third_party/boringssl/crypto/cpu-arm.c \
    third_party/boringssl/crypto/cpu-intel.c \
    third_party/boringssl/crypto/crypto.c \
    third_party/boringssl/crypto/curve25519/curve25519.c \
    third_party/boringssl/crypto/des/des.c \
    third_party/boringssl/crypto/dh/check.c \
    third_party/boringssl/crypto/dh/dh.c \
    third_party/boringssl/crypto/dh/dh_asn1.c \
    third_party/boringssl/crypto/dh/params.c \
    third_party/boringssl/crypto/digest/digest.c \
    third_party/boringssl/crypto/digest/digests.c \
    third_party/boringssl/crypto/directory_posix.c \
    third_party/boringssl/crypto/directory_win.c \
    third_party/boringssl/crypto/dsa/dsa.c \
    third_party/boringssl/crypto/dsa/dsa_asn1.c \
    third_party/boringssl/crypto/ec/ec.c \
    third_party/boringssl/crypto/ec/ec_asn1.c \
    third_party/boringssl/crypto/ec/ec_key.c \
    third_party/boringssl/crypto/ec/ec_montgomery.c \
    third_party/boringssl/crypto/ec/oct.c \
    third_party/boringssl/crypto/ec/p224-64.c \
    third_party/boringssl/crypto/ec/p256-64.c \
    third_party/boringssl/crypto/ec/p256-x86_64.c \
    third_party/boringssl/crypto/ec/simple.c \
    third_party/boringssl/crypto/ec/util-64.c \
    third_party/boringssl/crypto/ec/wnaf.c \
    third_party/boringssl/crypto/ecdh/ecdh.c \
    third_party/boringssl/crypto/ecdsa/ecdsa.c \
    third_party/boringssl/crypto/ecdsa/ecdsa_asn1.c \
    third_party/boringssl/crypto/engine/engine.c \
    third_party/boringssl/crypto/err/err.c \
    third_party/boringssl/crypto/evp/algorithm.c \
    third_party/boringssl/crypto/evp/digestsign.c \
    third_party/boringssl/crypto/evp/evp.c \
    third_party/boringssl/crypto/evp/evp_asn1.c \
    third_party/boringssl/crypto/evp/evp_ctx.c \
    third_party/boringssl/crypto/evp/p_dsa_asn1.c \
    third_party/boringssl/crypto/evp/p_ec.c \
    third_party/boringssl/crypto/evp/p_ec_asn1.c \
    third_party/boringssl/crypto/evp/p_rsa.c \
    third_party/boringssl/crypto/evp/p_rsa_asn1.c \
    third_party/boringssl/crypto/evp/pbkdf.c \
    third_party/boringssl/crypto/evp/sign.c \
    third_party/boringssl/crypto/ex_data.c \
    third_party/boringssl/crypto/hkdf/hkdf.c \
    third_party/boringssl/crypto/hmac/hmac.c \
    third_party/boringssl/crypto/lhash/lhash.c \
    third_party/boringssl/crypto/md4/md4.c \
    third_party/boringssl/crypto/md5/md5.c \
    third_party/boringssl/crypto/mem.c \
    third_party/boringssl/crypto/modes/cbc.c \
    third_party/boringssl/crypto/modes/cfb.c \
    third_party/boringssl/crypto/modes/ctr.c \
    third_party/boringssl/crypto/modes/gcm.c \
    third_party/boringssl/crypto/modes/ofb.c \
    third_party/boringssl/crypto/obj/obj.c \
    third_party/boringssl/crypto/obj/obj_xref.c \
    third_party/boringssl/crypto/pem/pem_all.c \
    third_party/boringssl/crypto/pem/pem_info.c \
    third_party/boringssl/crypto/pem/pem_lib.c \
    third_party/boringssl/crypto/pem/pem_oth.c \
    third_party/boringssl/crypto/pem/pem_pk8.c \
    third_party/boringssl/crypto/pem/pem_pkey.c \
    third_party/boringssl/crypto/pem/pem_x509.c \
    third_party/boringssl/crypto/pem/pem_xaux.c \
    third_party/boringssl/crypto/pkcs8/p5_pbe.c \
    third_party/boringssl/crypto/pkcs8/p5_pbev2.c \
    third_party/boringssl/crypto/pkcs8/p8_pkey.c \
    third_party/boringssl/crypto/pkcs8/pkcs8.c \
    third_party/boringssl/crypto/poly1305/poly1305.c \
    third_party/boringssl/crypto/poly1305/poly1305_arm.c \
    third_party/boringssl/crypto/poly1305/poly1305_vec.c \
    third_party/boringssl/crypto/rand/rand.c \
    third_party/boringssl/crypto/rand/urandom.c \
    third_party/boringssl/crypto/rand/windows.c \
    third_party/boringssl/crypto/rc4/rc4.c \
    third_party/boringssl/crypto/refcount_c11.c \
    third_party/boringssl/crypto/refcount_lock.c \
    third_party/boringssl/crypto/rsa/blinding.c \
    third_party/boringssl/crypto/rsa/padding.c \
    third_party/boringssl/crypto/rsa/rsa.c \
    third_party/boringssl/crypto/rsa/rsa_asn1.c \
    third_party/boringssl/crypto/rsa/rsa_impl.c \
    third_party/boringssl/crypto/sha/sha1.c \
    third_party/boringssl/crypto/sha/sha256.c \
    third_party/boringssl/crypto/sha/sha512.c \
    third_party/boringssl/crypto/stack/stack.c \
    third_party/boringssl/crypto/thread.c \
    third_party/boringssl/crypto/thread_none.c \
    third_party/boringssl/crypto/thread_pthread.c \
    third_party/boringssl/crypto/thread_win.c \
    third_party/boringssl/crypto/time_support.c \
    third_party/boringssl/crypto/x509/a_digest.c \
    third_party/boringssl/crypto/x509/a_sign.c \
    third_party/boringssl/crypto/x509/a_strex.c \
    third_party/boringssl/crypto/x509/a_verify.c \
    third_party/boringssl/crypto/x509/asn1_gen.c \
    third_party/boringssl/crypto/x509/by_dir.c \
    third_party/boringssl/crypto/x509/by_file.c \
    third_party/boringssl/crypto/x509/i2d_pr.c \
    third_party/boringssl/crypto/x509/pkcs7.c \
    third_party/boringssl/crypto/x509/t_crl.c \
    third_party/boringssl/crypto/x509/t_req.c \
    third_party/boringssl/crypto/x509/t_x509.c \
    third_party/boringssl/crypto/x509/t_x509a.c \
    third_party/boringssl/crypto/x509/x509.c \
    third_party/boringssl/crypto/x509/x509_att.c \
    third_party/boringssl/crypto/x509/x509_cmp.c \
    third_party/boringssl/crypto/x509/x509_d2.c \
    third_party/boringssl/crypto/x509/x509_def.c \
    third_party/boringssl/crypto/x509/x509_ext.c \
    third_party/boringssl/crypto/x509/x509_lu.c \
    third_party/boringssl/crypto/x509/x509_obj.c \
    third_party/boringssl/crypto/x509/x509_r2x.c \
    third_party/boringssl/crypto/x509/x509_req.c \
    third_party/boringssl/crypto/x509/x509_set.c \
    third_party/boringssl/crypto/x509/x509_trs.c \
    third_party/boringssl/crypto/x509/x509_txt.c \
    third_party/boringssl/crypto/x509/x509_v3.c \
    third_party/boringssl/crypto/x509/x509_vfy.c \
    third_party/boringssl/crypto/x509/x509_vpm.c \
    third_party/boringssl/crypto/x509/x509cset.c \
    third_party/boringssl/crypto/x509/x509name.c \
    third_party/boringssl/crypto/x509/x509rset.c \
    third_party/boringssl/crypto/x509/x509spki.c \
    third_party/boringssl/crypto/x509/x509type.c \
    third_party/boringssl/crypto/x509/x_algor.c \
    third_party/boringssl/crypto/x509/x_all.c \
    third_party/boringssl/crypto/x509/x_attrib.c \
    third_party/boringssl/crypto/x509/x_crl.c \
    third_party/boringssl/crypto/x509/x_exten.c \
    third_party/boringssl/crypto/x509/x_info.c \
    third_party/boringssl/crypto/x509/x_name.c \
    third_party/boringssl/crypto/x509/x_pkey.c \
    third_party/boringssl/crypto/x509/x_pubkey.c \
    third_party/boringssl/crypto/x509/x_req.c \
    third_party/boringssl/crypto/x509/x_sig.c \
    third_party/boringssl/crypto/x509/x_spki.c \
    third_party/boringssl/crypto/x509/x_val.c \
    third_party/boringssl/crypto/x509/x_x509.c \
    third_party/boringssl/crypto/x509/x_x509a.c \
    third_party/boringssl/crypto/x509v3/pcy_cache.c \
    third_party/boringssl/crypto/x509v3/pcy_data.c \
    third_party/boringssl/crypto/x509v3/pcy_lib.c \
    third_party/boringssl/crypto/x509v3/pcy_map.c \
    third_party/boringssl/crypto/x509v3/pcy_node.c \
    third_party/boringssl/crypto/x509v3/pcy_tree.c \
    third_party/boringssl/crypto/x509v3/v3_akey.c \
    third_party/boringssl/crypto/x509v3/v3_akeya.c \
    third_party/boringssl/crypto/x509v3/v3_alt.c \
    third_party/boringssl/crypto/x509v3/v3_bcons.c \
    third_party/boringssl/crypto/x509v3/v3_bitst.c \
    third_party/boringssl/crypto/x509v3/v3_conf.c \
    third_party/boringssl/crypto/x509v3/v3_cpols.c \
    third_party/boringssl/crypto/x509v3/v3_crld.c \
    third_party/boringssl/crypto/x509v3/v3_enum.c \
    third_party/boringssl/crypto/x509v3/v3_extku.c \
    third_party/boringssl/crypto/x509v3/v3_genn.c \
    third_party/boringssl/crypto/x509v3/v3_ia5.c \
    third_party/boringssl/crypto/x509v3/v3_info.c \
    third_party/boringssl/crypto/x509v3/v3_int.c \
    third_party/boringssl/crypto/x509v3/v3_lib.c \
    third_party/boringssl/crypto/x509v3/v3_ncons.c \
    third_party/boringssl/crypto/x509v3/v3_pci.c \
    third_party/boringssl/crypto/x509v3/v3_pcia.c \
    third_party/boringssl/crypto/x509v3/v3_pcons.c \
    third_party/boringssl/crypto/x509v3/v3_pku.c \
    third_party/boringssl/crypto/x509v3/v3_pmaps.c \
    third_party/boringssl/crypto/x509v3/v3_prn.c \
    third_party/boringssl/crypto/x509v3/v3_purp.c \
    third_party/boringssl/crypto/x509v3/v3_skey.c \
    third_party/boringssl/crypto/x509v3/v3_sxnet.c \
    third_party/boringssl/crypto/x509v3/v3_utl.c \
    third_party/boringssl/ssl/custom_extensions.c \
    third_party/boringssl/ssl/d1_both.c \
    third_party/boringssl/ssl/d1_clnt.c \
    third_party/boringssl/ssl/d1_lib.c \
    third_party/boringssl/ssl/d1_meth.c \
    third_party/boringssl/ssl/d1_pkt.c \
    third_party/boringssl/ssl/d1_srtp.c \
    third_party/boringssl/ssl/d1_srvr.c \
    third_party/boringssl/ssl/dtls_record.c \
    third_party/boringssl/ssl/pqueue/pqueue.c \
    third_party/boringssl/ssl/s3_both.c \
    third_party/boringssl/ssl/s3_clnt.c \
    third_party/boringssl/ssl/s3_enc.c \
    third_party/boringssl/ssl/s3_lib.c \
    third_party/boringssl/ssl/s3_meth.c \
    third_party/boringssl/ssl/s3_pkt.c \
    third_party/boringssl/ssl/s3_srvr.c \
    third_party/boringssl/ssl/ssl_aead_ctx.c \
    third_party/boringssl/ssl/ssl_asn1.c \
    third_party/boringssl/ssl/ssl_buffer.c \
    third_party/boringssl/ssl/ssl_cert.c \
    third_party/boringssl/ssl/ssl_cipher.c \
    third_party/boringssl/ssl/ssl_file.c \
    third_party/boringssl/ssl/ssl_lib.c \
    third_party/boringssl/ssl/ssl_rsa.c \
    third_party/boringssl/ssl/ssl_session.c \
    third_party/boringssl/ssl/ssl_stat.c \
    third_party/boringssl/ssl/t1_enc.c \
    third_party/boringssl/ssl/t1_lib.c \
    third_party/boringssl/ssl/tls_record.c \


LIBBORINGSSL_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(LIBBORINGSSL_SRC))))

# boringssl needs an override to ensure that it does not include
# system openssl headers regardless of other configuration
# we do so here with a target specific variable assignment
$(LIBBORINGSSL_OBJS): CFLAGS := -Ithird_party/boringssl/include $(CFLAGS) -Wno-sign-conversion -Wno-conversion -Wno-unused-value -fvisibility=hidden
$(LIBBORINGSSL_OBJS): CXXFLAGS := -Ithird_party/boringssl/include $(CXXFLAGS) -fvisibility=hidden
$(LIBBORINGSSL_OBJS): CPPFLAGS += -DOPENSSL_NO_ASM -D_GNU_SOURCE

$(LIBDIR)/$(CONFIG)/libboringssl.a: $(ZLIB_DEP) $(OPENSSL_DEP)  $(LIBBORINGSSL_OBJS)
	$(E) "[AR]      Creating $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) rm -f $(LIBDIR)/$(CONFIG)/libboringssl.a
	$(Q) $(AR) rcs $(LIBDIR)/$(CONFIG)/libboringssl.a $(LIBBORINGSSL_OBJS)
ifeq ($(SYSTEM),Darwin)
	$(Q) ranlib $(LIBDIR)/$(CONFIG)/libboringssl.a
endif




ifneq ($(NO_DEPS),true)
-include $(LIBBORINGSSL_OBJS:.o=.dep)
endif


LIBBORINGSSL_TEST_UTIL_SRC = \
    third_party/boringssl/crypto/test/file_test.cc \
    third_party/boringssl/crypto/test/malloc.cc \
    third_party/boringssl/crypto/test/test_util.cc \


LIBBORINGSSL_TEST_UTIL_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(LIBBORINGSSL_TEST_UTIL_SRC))))

# boringssl needs an override to ensure that it does not include
# system openssl headers regardless of other configuration
# we do so here with a target specific variable assignment
$(LIBBORINGSSL_TEST_UTIL_OBJS): CFLAGS := -Ithird_party/boringssl/include $(CFLAGS) -Wno-sign-conversion -Wno-conversion -Wno-unused-value -fvisibility=hidden
$(LIBBORINGSSL_TEST_UTIL_OBJS): CXXFLAGS := -Ithird_party/boringssl/include $(CXXFLAGS) -fvisibility=hidden
$(LIBBORINGSSL_TEST_UTIL_OBJS): CPPFLAGS += -DOPENSSL_NO_ASM -D_GNU_SOURCE

ifeq ($(NO_PROTOBUF),true)

# You can't build a C++ library if you don't have protobuf - a bit overreached, but still okay.

$(LIBDIR)/$(CONFIG)/libboringssl_test_util.a: protobuf_dep_error


else

$(LIBDIR)/$(CONFIG)/libboringssl_test_util.a: $(ZLIB_DEP) $(OPENSSL_DEP)  $(PROTOBUF_DEP) $(LIBBORINGSSL_TEST_UTIL_OBJS)
	$(E) "[AR]      Creating $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) rm -f $(LIBDIR)/$(CONFIG)/libboringssl_test_util.a
	$(Q) $(AR) rcs $(LIBDIR)/$(CONFIG)/libboringssl_test_util.a $(LIBBORINGSSL_TEST_UTIL_OBJS)
ifeq ($(SYSTEM),Darwin)
	$(Q) ranlib $(LIBDIR)/$(CONFIG)/libboringssl_test_util.a
endif




endif

ifneq ($(NO_DEPS),true)
-include $(LIBBORINGSSL_TEST_UTIL_OBJS:.o=.dep)
endif


LIBBORINGSSL_AES_TEST_LIB_SRC = \
    third_party/boringssl/crypto/aes/aes_test.cc \


LIBBORINGSSL_AES_TEST_LIB_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(LIBBORINGSSL_AES_TEST_LIB_SRC))))

# boringssl needs an override to ensure that it does not include
# system openssl headers regardless of other configuration
# we do so here with a target specific variable assignment
$(LIBBORINGSSL_AES_TEST_LIB_OBJS): CFLAGS := -Ithird_party/boringssl/include $(CFLAGS) -Wno-sign-conversion -Wno-conversion -Wno-unused-value -fvisibility=hidden
$(LIBBORINGSSL_AES_TEST_LIB_OBJS): CXXFLAGS := -Ithird_party/boringssl/include $(CXXFLAGS) -fvisibility=hidden
$(LIBBORINGSSL_AES_TEST_LIB_OBJS): CPPFLAGS += -DOPENSSL_NO_ASM -D_GNU_SOURCE

ifeq ($(NO_PROTOBUF),true)

# You can't build a C++ library if you don't have protobuf - a bit overreached, but still okay.

$(LIBDIR)/$(CONFIG)/libboringssl_aes_test_lib.a: protobuf_dep_error


else

$(LIBDIR)/$(CONFIG)/libboringssl_aes_test_lib.a: $(ZLIB_DEP) $(OPENSSL_DEP)  $(PROTOBUF_DEP) $(LIBBORINGSSL_AES_TEST_LIB_OBJS)
	$(E) "[AR]      Creating $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) rm -f $(LIBDIR)/$(CONFIG)/libboringssl_aes_test_lib.a
	$(Q) $(AR) rcs $(LIBDIR)/$(CONFIG)/libboringssl_aes_test_lib.a $(LIBBORINGSSL_AES_TEST_LIB_OBJS)
ifeq ($(SYSTEM),Darwin)
	$(Q) ranlib $(LIBDIR)/$(CONFIG)/libboringssl_aes_test_lib.a
endif




endif

ifneq ($(NO_DEPS),true)
-include $(LIBBORINGSSL_AES_TEST_LIB_OBJS:.o=.dep)
endif


LIBBORINGSSL_BASE64_TEST_LIB_SRC = \
    third_party/boringssl/crypto/base64/base64_test.cc \


LIBBORINGSSL_BASE64_TEST_LIB_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(LIBBORINGSSL_BASE64_TEST_LIB_SRC))))

# boringssl needs an override to ensure that it does not include
# system openssl headers regardless of other configuration
# we do so here with a target specific variable assignment
$(LIBBORINGSSL_BASE64_TEST_LIB_OBJS): CFLAGS := -Ithird_party/boringssl/include $(CFLAGS) -Wno-sign-conversion -Wno-conversion -Wno-unused-value -fvisibility=hidden
$(LIBBORINGSSL_BASE64_TEST_LIB_OBJS): CXXFLAGS := -Ithird_party/boringssl/include $(CXXFLAGS) -fvisibility=hidden
$(LIBBORINGSSL_BASE64_TEST_LIB_OBJS): CPPFLAGS += -DOPENSSL_NO_ASM -D_GNU_SOURCE

ifeq ($(NO_PROTOBUF),true)

# You can't build a C++ library if you don't have protobuf - a bit overreached, but still okay.

$(LIBDIR)/$(CONFIG)/libboringssl_base64_test_lib.a: protobuf_dep_error


else

$(LIBDIR)/$(CONFIG)/libboringssl_base64_test_lib.a: $(ZLIB_DEP) $(OPENSSL_DEP)  $(PROTOBUF_DEP) $(LIBBORINGSSL_BASE64_TEST_LIB_OBJS)
	$(E) "[AR]      Creating $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) rm -f $(LIBDIR)/$(CONFIG)/libboringssl_base64_test_lib.a
	$(Q) $(AR) rcs $(LIBDIR)/$(CONFIG)/libboringssl_base64_test_lib.a $(LIBBORINGSSL_BASE64_TEST_LIB_OBJS)
ifeq ($(SYSTEM),Darwin)
	$(Q) ranlib $(LIBDIR)/$(CONFIG)/libboringssl_base64_test_lib.a
endif




endif

ifneq ($(NO_DEPS),true)
-include $(LIBBORINGSSL_BASE64_TEST_LIB_OBJS:.o=.dep)
endif


LIBBORINGSSL_BIO_TEST_LIB_SRC = \
    third_party/boringssl/crypto/bio/bio_test.cc \


LIBBORINGSSL_BIO_TEST_LIB_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(LIBBORINGSSL_BIO_TEST_LIB_SRC))))

# boringssl needs an override to ensure that it does not include
# system openssl headers regardless of other configuration
# we do so here with a target specific variable assignment
$(LIBBORINGSSL_BIO_TEST_LIB_OBJS): CFLAGS := -Ithird_party/boringssl/include $(CFLAGS) -Wno-sign-conversion -Wno-conversion -Wno-unused-value -fvisibility=hidden
$(LIBBORINGSSL_BIO_TEST_LIB_OBJS): CXXFLAGS := -Ithird_party/boringssl/include $(CXXFLAGS) -fvisibility=hidden
$(LIBBORINGSSL_BIO_TEST_LIB_OBJS): CPPFLAGS += -DOPENSSL_NO_ASM -D_GNU_SOURCE

ifeq ($(NO_PROTOBUF),true)

# You can't build a C++ library if you don't have protobuf - a bit overreached, but still okay.

$(LIBDIR)/$(CONFIG)/libboringssl_bio_test_lib.a: protobuf_dep_error


else

$(LIBDIR)/$(CONFIG)/libboringssl_bio_test_lib.a: $(ZLIB_DEP) $(OPENSSL_DEP)  $(PROTOBUF_DEP) $(LIBBORINGSSL_BIO_TEST_LIB_OBJS)
	$(E) "[AR]      Creating $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) rm -f $(LIBDIR)/$(CONFIG)/libboringssl_bio_test_lib.a
	$(Q) $(AR) rcs $(LIBDIR)/$(CONFIG)/libboringssl_bio_test_lib.a $(LIBBORINGSSL_BIO_TEST_LIB_OBJS)
ifeq ($(SYSTEM),Darwin)
	$(Q) ranlib $(LIBDIR)/$(CONFIG)/libboringssl_bio_test_lib.a
endif




endif

ifneq ($(NO_DEPS),true)
-include $(LIBBORINGSSL_BIO_TEST_LIB_OBJS:.o=.dep)
endif


LIBBORINGSSL_BN_TEST_LIB_SRC = \
    third_party/boringssl/crypto/bn/bn_test.cc \


LIBBORINGSSL_BN_TEST_LIB_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(LIBBORINGSSL_BN_TEST_LIB_SRC))))

# boringssl needs an override to ensure that it does not include
# system openssl headers regardless of other configuration
# we do so here with a target specific variable assignment
$(LIBBORINGSSL_BN_TEST_LIB_OBJS): CFLAGS := -Ithird_party/boringssl/include $(CFLAGS) -Wno-sign-conversion -Wno-conversion -Wno-unused-value -fvisibility=hidden
$(LIBBORINGSSL_BN_TEST_LIB_OBJS): CXXFLAGS := -Ithird_party/boringssl/include $(CXXFLAGS) -fvisibility=hidden
$(LIBBORINGSSL_BN_TEST_LIB_OBJS): CPPFLAGS += -DOPENSSL_NO_ASM -D_GNU_SOURCE

ifeq ($(NO_PROTOBUF),true)

# You can't build a C++ library if you don't have protobuf - a bit overreached, but still okay.

$(LIBDIR)/$(CONFIG)/libboringssl_bn_test_lib.a: protobuf_dep_error


else

$(LIBDIR)/$(CONFIG)/libboringssl_bn_test_lib.a: $(ZLIB_DEP) $(OPENSSL_DEP)  $(PROTOBUF_DEP) $(LIBBORINGSSL_BN_TEST_LIB_OBJS)
	$(E) "[AR]      Creating $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) rm -f $(LIBDIR)/$(CONFIG)/libboringssl_bn_test_lib.a
	$(Q) $(AR) rcs $(LIBDIR)/$(CONFIG)/libboringssl_bn_test_lib.a $(LIBBORINGSSL_BN_TEST_LIB_OBJS)
ifeq ($(SYSTEM),Darwin)
	$(Q) ranlib $(LIBDIR)/$(CONFIG)/libboringssl_bn_test_lib.a
endif




endif

ifneq ($(NO_DEPS),true)
-include $(LIBBORINGSSL_BN_TEST_LIB_OBJS:.o=.dep)
endif


LIBBORINGSSL_BYTESTRING_TEST_LIB_SRC = \
    third_party/boringssl/crypto/bytestring/bytestring_test.cc \


LIBBORINGSSL_BYTESTRING_TEST_LIB_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(LIBBORINGSSL_BYTESTRING_TEST_LIB_SRC))))

# boringssl needs an override to ensure that it does not include
# system openssl headers regardless of other configuration
# we do so here with a target specific variable assignment
$(LIBBORINGSSL_BYTESTRING_TEST_LIB_OBJS): CFLAGS := -Ithird_party/boringssl/include $(CFLAGS) -Wno-sign-conversion -Wno-conversion -Wno-unused-value -fvisibility=hidden
$(LIBBORINGSSL_BYTESTRING_TEST_LIB_OBJS): CXXFLAGS := -Ithird_party/boringssl/include $(CXXFLAGS) -fvisibility=hidden
$(LIBBORINGSSL_BYTESTRING_TEST_LIB_OBJS): CPPFLAGS += -DOPENSSL_NO_ASM -D_GNU_SOURCE

ifeq ($(NO_PROTOBUF),true)

# You can't build a C++ library if you don't have protobuf - a bit overreached, but still okay.

$(LIBDIR)/$(CONFIG)/libboringssl_bytestring_test_lib.a: protobuf_dep_error


else

$(LIBDIR)/$(CONFIG)/libboringssl_bytestring_test_lib.a: $(ZLIB_DEP) $(OPENSSL_DEP)  $(PROTOBUF_DEP) $(LIBBORINGSSL_BYTESTRING_TEST_LIB_OBJS)
	$(E) "[AR]      Creating $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) rm -f $(LIBDIR)/$(CONFIG)/libboringssl_bytestring_test_lib.a
	$(Q) $(AR) rcs $(LIBDIR)/$(CONFIG)/libboringssl_bytestring_test_lib.a $(LIBBORINGSSL_BYTESTRING_TEST_LIB_OBJS)
ifeq ($(SYSTEM),Darwin)
	$(Q) ranlib $(LIBDIR)/$(CONFIG)/libboringssl_bytestring_test_lib.a
endif




endif

ifneq ($(NO_DEPS),true)
-include $(LIBBORINGSSL_BYTESTRING_TEST_LIB_OBJS:.o=.dep)
endif


LIBBORINGSSL_AEAD_TEST_LIB_SRC = \
    third_party/boringssl/crypto/cipher/aead_test.cc \


LIBBORINGSSL_AEAD_TEST_LIB_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(LIBBORINGSSL_AEAD_TEST_LIB_SRC))))

# boringssl needs an override to ensure that it does not include
# system openssl headers regardless of other configuration
# we do so here with a target specific variable assignment
$(LIBBORINGSSL_AEAD_TEST_LIB_OBJS): CFLAGS := -Ithird_party/boringssl/include $(CFLAGS) -Wno-sign-conversion -Wno-conversion -Wno-unused-value -fvisibility=hidden
$(LIBBORINGSSL_AEAD_TEST_LIB_OBJS): CXXFLAGS := -Ithird_party/boringssl/include $(CXXFLAGS) -fvisibility=hidden
$(LIBBORINGSSL_AEAD_TEST_LIB_OBJS): CPPFLAGS += -DOPENSSL_NO_ASM -D_GNU_SOURCE

ifeq ($(NO_PROTOBUF),true)

# You can't build a C++ library if you don't have protobuf - a bit overreached, but still okay.

$(LIBDIR)/$(CONFIG)/libboringssl_aead_test_lib.a: protobuf_dep_error


else

$(LIBDIR)/$(CONFIG)/libboringssl_aead_test_lib.a: $(ZLIB_DEP) $(OPENSSL_DEP)  $(PROTOBUF_DEP) $(LIBBORINGSSL_AEAD_TEST_LIB_OBJS)
	$(E) "[AR]      Creating $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) rm -f $(LIBDIR)/$(CONFIG)/libboringssl_aead_test_lib.a
	$(Q) $(AR) rcs $(LIBDIR)/$(CONFIG)/libboringssl_aead_test_lib.a $(LIBBORINGSSL_AEAD_TEST_LIB_OBJS)
ifeq ($(SYSTEM),Darwin)
	$(Q) ranlib $(LIBDIR)/$(CONFIG)/libboringssl_aead_test_lib.a
endif




endif

ifneq ($(NO_DEPS),true)
-include $(LIBBORINGSSL_AEAD_TEST_LIB_OBJS:.o=.dep)
endif


LIBBORINGSSL_CIPHER_TEST_LIB_SRC = \
    third_party/boringssl/crypto/cipher/cipher_test.cc \


LIBBORINGSSL_CIPHER_TEST_LIB_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(LIBBORINGSSL_CIPHER_TEST_LIB_SRC))))

# boringssl needs an override to ensure that it does not include
# system openssl headers regardless of other configuration
# we do so here with a target specific variable assignment
$(LIBBORINGSSL_CIPHER_TEST_LIB_OBJS): CFLAGS := -Ithird_party/boringssl/include $(CFLAGS) -Wno-sign-conversion -Wno-conversion -Wno-unused-value -fvisibility=hidden
$(LIBBORINGSSL_CIPHER_TEST_LIB_OBJS): CXXFLAGS := -Ithird_party/boringssl/include $(CXXFLAGS) -fvisibility=hidden
$(LIBBORINGSSL_CIPHER_TEST_LIB_OBJS): CPPFLAGS += -DOPENSSL_NO_ASM -D_GNU_SOURCE

ifeq ($(NO_PROTOBUF),true)

# You can't build a C++ library if you don't have protobuf - a bit overreached, but still okay.

$(LIBDIR)/$(CONFIG)/libboringssl_cipher_test_lib.a: protobuf_dep_error


else

$(LIBDIR)/$(CONFIG)/libboringssl_cipher_test_lib.a: $(ZLIB_DEP) $(OPENSSL_DEP)  $(PROTOBUF_DEP) $(LIBBORINGSSL_CIPHER_TEST_LIB_OBJS)
	$(E) "[AR]      Creating $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) rm -f $(LIBDIR)/$(CONFIG)/libboringssl_cipher_test_lib.a
	$(Q) $(AR) rcs $(LIBDIR)/$(CONFIG)/libboringssl_cipher_test_lib.a $(LIBBORINGSSL_CIPHER_TEST_LIB_OBJS)
ifeq ($(SYSTEM),Darwin)
	$(Q) ranlib $(LIBDIR)/$(CONFIG)/libboringssl_cipher_test_lib.a
endif




endif

ifneq ($(NO_DEPS),true)
-include $(LIBBORINGSSL_CIPHER_TEST_LIB_OBJS:.o=.dep)
endif


LIBBORINGSSL_CMAC_TEST_LIB_SRC = \
    third_party/boringssl/crypto/cmac/cmac_test.cc \


LIBBORINGSSL_CMAC_TEST_LIB_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(LIBBORINGSSL_CMAC_TEST_LIB_SRC))))

# boringssl needs an override to ensure that it does not include
# system openssl headers regardless of other configuration
# we do so here with a target specific variable assignment
$(LIBBORINGSSL_CMAC_TEST_LIB_OBJS): CFLAGS := -Ithird_party/boringssl/include $(CFLAGS) -Wno-sign-conversion -Wno-conversion -Wno-unused-value -fvisibility=hidden
$(LIBBORINGSSL_CMAC_TEST_LIB_OBJS): CXXFLAGS := -Ithird_party/boringssl/include $(CXXFLAGS) -fvisibility=hidden
$(LIBBORINGSSL_CMAC_TEST_LIB_OBJS): CPPFLAGS += -DOPENSSL_NO_ASM -D_GNU_SOURCE

ifeq ($(NO_PROTOBUF),true)

# You can't build a C++ library if you don't have protobuf - a bit overreached, but still okay.

$(LIBDIR)/$(CONFIG)/libboringssl_cmac_test_lib.a: protobuf_dep_error


else

$(LIBDIR)/$(CONFIG)/libboringssl_cmac_test_lib.a: $(ZLIB_DEP) $(OPENSSL_DEP)  $(PROTOBUF_DEP) $(LIBBORINGSSL_CMAC_TEST_LIB_OBJS)
	$(E) "[AR]      Creating $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) rm -f $(LIBDIR)/$(CONFIG)/libboringssl_cmac_test_lib.a
	$(Q) $(AR) rcs $(LIBDIR)/$(CONFIG)/libboringssl_cmac_test_lib.a $(LIBBORINGSSL_CMAC_TEST_LIB_OBJS)
ifeq ($(SYSTEM),Darwin)
	$(Q) ranlib $(LIBDIR)/$(CONFIG)/libboringssl_cmac_test_lib.a
endif




endif

ifneq ($(NO_DEPS),true)
-include $(LIBBORINGSSL_CMAC_TEST_LIB_OBJS:.o=.dep)
endif


LIBBORINGSSL_CONSTANT_TIME_TEST_LIB_SRC = \
    third_party/boringssl/crypto/constant_time_test.c \


LIBBORINGSSL_CONSTANT_TIME_TEST_LIB_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(LIBBORINGSSL_CONSTANT_TIME_TEST_LIB_SRC))))

# boringssl needs an override to ensure that it does not include
# system openssl headers regardless of other configuration
# we do so here with a target specific variable assignment
$(LIBBORINGSSL_CONSTANT_TIME_TEST_LIB_OBJS): CFLAGS := -Ithird_party/boringssl/include $(CFLAGS) -Wno-sign-conversion -Wno-conversion -Wno-unused-value -fvisibility=hidden
$(LIBBORINGSSL_CONSTANT_TIME_TEST_LIB_OBJS): CXXFLAGS := -Ithird_party/boringssl/include $(CXXFLAGS) -fvisibility=hidden
$(LIBBORINGSSL_CONSTANT_TIME_TEST_LIB_OBJS): CPPFLAGS += -DOPENSSL_NO_ASM -D_GNU_SOURCE

$(LIBDIR)/$(CONFIG)/libboringssl_constant_time_test_lib.a: $(ZLIB_DEP) $(OPENSSL_DEP)  $(LIBBORINGSSL_CONSTANT_TIME_TEST_LIB_OBJS)
	$(E) "[AR]      Creating $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) rm -f $(LIBDIR)/$(CONFIG)/libboringssl_constant_time_test_lib.a
	$(Q) $(AR) rcs $(LIBDIR)/$(CONFIG)/libboringssl_constant_time_test_lib.a $(LIBBORINGSSL_CONSTANT_TIME_TEST_LIB_OBJS)
ifeq ($(SYSTEM),Darwin)
	$(Q) ranlib $(LIBDIR)/$(CONFIG)/libboringssl_constant_time_test_lib.a
endif




ifneq ($(NO_DEPS),true)
-include $(LIBBORINGSSL_CONSTANT_TIME_TEST_LIB_OBJS:.o=.dep)
endif


LIBBORINGSSL_ED25519_TEST_LIB_SRC = \
    third_party/boringssl/crypto/curve25519/ed25519_test.cc \


LIBBORINGSSL_ED25519_TEST_LIB_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(LIBBORINGSSL_ED25519_TEST_LIB_SRC))))

# boringssl needs an override to ensure that it does not include
# system openssl headers regardless of other configuration
# we do so here with a target specific variable assignment
$(LIBBORINGSSL_ED25519_TEST_LIB_OBJS): CFLAGS := -Ithird_party/boringssl/include $(CFLAGS) -Wno-sign-conversion -Wno-conversion -Wno-unused-value -fvisibility=hidden
$(LIBBORINGSSL_ED25519_TEST_LIB_OBJS): CXXFLAGS := -Ithird_party/boringssl/include $(CXXFLAGS) -fvisibility=hidden
$(LIBBORINGSSL_ED25519_TEST_LIB_OBJS): CPPFLAGS += -DOPENSSL_NO_ASM -D_GNU_SOURCE

ifeq ($(NO_PROTOBUF),true)

# You can't build a C++ library if you don't have protobuf - a bit overreached, but still okay.

$(LIBDIR)/$(CONFIG)/libboringssl_ed25519_test_lib.a: protobuf_dep_error


else

$(LIBDIR)/$(CONFIG)/libboringssl_ed25519_test_lib.a: $(ZLIB_DEP) $(OPENSSL_DEP)  $(PROTOBUF_DEP) $(LIBBORINGSSL_ED25519_TEST_LIB_OBJS)
	$(E) "[AR]      Creating $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) rm -f $(LIBDIR)/$(CONFIG)/libboringssl_ed25519_test_lib.a
	$(Q) $(AR) rcs $(LIBDIR)/$(CONFIG)/libboringssl_ed25519_test_lib.a $(LIBBORINGSSL_ED25519_TEST_LIB_OBJS)
ifeq ($(SYSTEM),Darwin)
	$(Q) ranlib $(LIBDIR)/$(CONFIG)/libboringssl_ed25519_test_lib.a
endif




endif

ifneq ($(NO_DEPS),true)
-include $(LIBBORINGSSL_ED25519_TEST_LIB_OBJS:.o=.dep)
endif


LIBBORINGSSL_X25519_TEST_LIB_SRC = \
    third_party/boringssl/crypto/curve25519/x25519_test.cc \


LIBBORINGSSL_X25519_TEST_LIB_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(LIBBORINGSSL_X25519_TEST_LIB_SRC))))

# boringssl needs an override to ensure that it does not include
# system openssl headers regardless of other configuration
# we do so here with a target specific variable assignment
$(LIBBORINGSSL_X25519_TEST_LIB_OBJS): CFLAGS := -Ithird_party/boringssl/include $(CFLAGS) -Wno-sign-conversion -Wno-conversion -Wno-unused-value -fvisibility=hidden
$(LIBBORINGSSL_X25519_TEST_LIB_OBJS): CXXFLAGS := -Ithird_party/boringssl/include $(CXXFLAGS) -fvisibility=hidden
$(LIBBORINGSSL_X25519_TEST_LIB_OBJS): CPPFLAGS += -DOPENSSL_NO_ASM -D_GNU_SOURCE

ifeq ($(NO_PROTOBUF),true)

# You can't build a C++ library if you don't have protobuf - a bit overreached, but still okay.

$(LIBDIR)/$(CONFIG)/libboringssl_x25519_test_lib.a: protobuf_dep_error


else

$(LIBDIR)/$(CONFIG)/libboringssl_x25519_test_lib.a: $(ZLIB_DEP) $(OPENSSL_DEP)  $(PROTOBUF_DEP) $(LIBBORINGSSL_X25519_TEST_LIB_OBJS)
	$(E) "[AR]      Creating $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) rm -f $(LIBDIR)/$(CONFIG)/libboringssl_x25519_test_lib.a
	$(Q) $(AR) rcs $(LIBDIR)/$(CONFIG)/libboringssl_x25519_test_lib.a $(LIBBORINGSSL_X25519_TEST_LIB_OBJS)
ifeq ($(SYSTEM),Darwin)
	$(Q) ranlib $(LIBDIR)/$(CONFIG)/libboringssl_x25519_test_lib.a
endif




endif

ifneq ($(NO_DEPS),true)
-include $(LIBBORINGSSL_X25519_TEST_LIB_OBJS:.o=.dep)
endif


LIBBORINGSSL_DH_TEST_LIB_SRC = \
    third_party/boringssl/crypto/dh/dh_test.cc \


LIBBORINGSSL_DH_TEST_LIB_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(LIBBORINGSSL_DH_TEST_LIB_SRC))))

# boringssl needs an override to ensure that it does not include
# system openssl headers regardless of other configuration
# we do so here with a target specific variable assignment
$(LIBBORINGSSL_DH_TEST_LIB_OBJS): CFLAGS := -Ithird_party/boringssl/include $(CFLAGS) -Wno-sign-conversion -Wno-conversion -Wno-unused-value -fvisibility=hidden
$(LIBBORINGSSL_DH_TEST_LIB_OBJS): CXXFLAGS := -Ithird_party/boringssl/include $(CXXFLAGS) -fvisibility=hidden
$(LIBBORINGSSL_DH_TEST_LIB_OBJS): CPPFLAGS += -DOPENSSL_NO_ASM -D_GNU_SOURCE

ifeq ($(NO_PROTOBUF),true)

# You can't build a C++ library if you don't have protobuf - a bit overreached, but still okay.

$(LIBDIR)/$(CONFIG)/libboringssl_dh_test_lib.a: protobuf_dep_error


else

$(LIBDIR)/$(CONFIG)/libboringssl_dh_test_lib.a: $(ZLIB_DEP) $(OPENSSL_DEP)  $(PROTOBUF_DEP) $(LIBBORINGSSL_DH_TEST_LIB_OBJS)
	$(E) "[AR]      Creating $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) rm -f $(LIBDIR)/$(CONFIG)/libboringssl_dh_test_lib.a
	$(Q) $(AR) rcs $(LIBDIR)/$(CONFIG)/libboringssl_dh_test_lib.a $(LIBBORINGSSL_DH_TEST_LIB_OBJS)
ifeq ($(SYSTEM),Darwin)
	$(Q) ranlib $(LIBDIR)/$(CONFIG)/libboringssl_dh_test_lib.a
endif




endif

ifneq ($(NO_DEPS),true)
-include $(LIBBORINGSSL_DH_TEST_LIB_OBJS:.o=.dep)
endif


LIBBORINGSSL_DIGEST_TEST_LIB_SRC = \
    third_party/boringssl/crypto/digest/digest_test.cc \


LIBBORINGSSL_DIGEST_TEST_LIB_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(LIBBORINGSSL_DIGEST_TEST_LIB_SRC))))

# boringssl needs an override to ensure that it does not include
# system openssl headers regardless of other configuration
# we do so here with a target specific variable assignment
$(LIBBORINGSSL_DIGEST_TEST_LIB_OBJS): CFLAGS := -Ithird_party/boringssl/include $(CFLAGS) -Wno-sign-conversion -Wno-conversion -Wno-unused-value -fvisibility=hidden
$(LIBBORINGSSL_DIGEST_TEST_LIB_OBJS): CXXFLAGS := -Ithird_party/boringssl/include $(CXXFLAGS) -fvisibility=hidden
$(LIBBORINGSSL_DIGEST_TEST_LIB_OBJS): CPPFLAGS += -DOPENSSL_NO_ASM -D_GNU_SOURCE

ifeq ($(NO_PROTOBUF),true)

# You can't build a C++ library if you don't have protobuf - a bit overreached, but still okay.

$(LIBDIR)/$(CONFIG)/libboringssl_digest_test_lib.a: protobuf_dep_error


else

$(LIBDIR)/$(CONFIG)/libboringssl_digest_test_lib.a: $(ZLIB_DEP) $(OPENSSL_DEP)  $(PROTOBUF_DEP) $(LIBBORINGSSL_DIGEST_TEST_LIB_OBJS)
	$(E) "[AR]      Creating $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) rm -f $(LIBDIR)/$(CONFIG)/libboringssl_digest_test_lib.a
	$(Q) $(AR) rcs $(LIBDIR)/$(CONFIG)/libboringssl_digest_test_lib.a $(LIBBORINGSSL_DIGEST_TEST_LIB_OBJS)
ifeq ($(SYSTEM),Darwin)
	$(Q) ranlib $(LIBDIR)/$(CONFIG)/libboringssl_digest_test_lib.a
endif




endif

ifneq ($(NO_DEPS),true)
-include $(LIBBORINGSSL_DIGEST_TEST_LIB_OBJS:.o=.dep)
endif


LIBBORINGSSL_DSA_TEST_LIB_SRC = \
    third_party/boringssl/crypto/dsa/dsa_test.c \


LIBBORINGSSL_DSA_TEST_LIB_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(LIBBORINGSSL_DSA_TEST_LIB_SRC))))

# boringssl needs an override to ensure that it does not include
# system openssl headers regardless of other configuration
# we do so here with a target specific variable assignment
$(LIBBORINGSSL_DSA_TEST_LIB_OBJS): CFLAGS := -Ithird_party/boringssl/include $(CFLAGS) -Wno-sign-conversion -Wno-conversion -Wno-unused-value -fvisibility=hidden
$(LIBBORINGSSL_DSA_TEST_LIB_OBJS): CXXFLAGS := -Ithird_party/boringssl/include $(CXXFLAGS) -fvisibility=hidden
$(LIBBORINGSSL_DSA_TEST_LIB_OBJS): CPPFLAGS += -DOPENSSL_NO_ASM -D_GNU_SOURCE

$(LIBDIR)/$(CONFIG)/libboringssl_dsa_test_lib.a: $(ZLIB_DEP) $(OPENSSL_DEP)  $(LIBBORINGSSL_DSA_TEST_LIB_OBJS)
	$(E) "[AR]      Creating $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) rm -f $(LIBDIR)/$(CONFIG)/libboringssl_dsa_test_lib.a
	$(Q) $(AR) rcs $(LIBDIR)/$(CONFIG)/libboringssl_dsa_test_lib.a $(LIBBORINGSSL_DSA_TEST_LIB_OBJS)
ifeq ($(SYSTEM),Darwin)
	$(Q) ranlib $(LIBDIR)/$(CONFIG)/libboringssl_dsa_test_lib.a
endif




ifneq ($(NO_DEPS),true)
-include $(LIBBORINGSSL_DSA_TEST_LIB_OBJS:.o=.dep)
endif


LIBBORINGSSL_EC_TEST_LIB_SRC = \
    third_party/boringssl/crypto/ec/ec_test.cc \


LIBBORINGSSL_EC_TEST_LIB_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(LIBBORINGSSL_EC_TEST_LIB_SRC))))

# boringssl needs an override to ensure that it does not include
# system openssl headers regardless of other configuration
# we do so here with a target specific variable assignment
$(LIBBORINGSSL_EC_TEST_LIB_OBJS): CFLAGS := -Ithird_party/boringssl/include $(CFLAGS) -Wno-sign-conversion -Wno-conversion -Wno-unused-value -fvisibility=hidden
$(LIBBORINGSSL_EC_TEST_LIB_OBJS): CXXFLAGS := -Ithird_party/boringssl/include $(CXXFLAGS) -fvisibility=hidden
$(LIBBORINGSSL_EC_TEST_LIB_OBJS): CPPFLAGS += -DOPENSSL_NO_ASM -D_GNU_SOURCE

ifeq ($(NO_PROTOBUF),true)

# You can't build a C++ library if you don't have protobuf - a bit overreached, but still okay.

$(LIBDIR)/$(CONFIG)/libboringssl_ec_test_lib.a: protobuf_dep_error


else

$(LIBDIR)/$(CONFIG)/libboringssl_ec_test_lib.a: $(ZLIB_DEP) $(OPENSSL_DEP)  $(PROTOBUF_DEP) $(LIBBORINGSSL_EC_TEST_LIB_OBJS)
	$(E) "[AR]      Creating $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) rm -f $(LIBDIR)/$(CONFIG)/libboringssl_ec_test_lib.a
	$(Q) $(AR) rcs $(LIBDIR)/$(CONFIG)/libboringssl_ec_test_lib.a $(LIBBORINGSSL_EC_TEST_LIB_OBJS)
ifeq ($(SYSTEM),Darwin)
	$(Q) ranlib $(LIBDIR)/$(CONFIG)/libboringssl_ec_test_lib.a
endif




endif

ifneq ($(NO_DEPS),true)
-include $(LIBBORINGSSL_EC_TEST_LIB_OBJS:.o=.dep)
endif


LIBBORINGSSL_EXAMPLE_MUL_LIB_SRC = \
    third_party/boringssl/crypto/ec/example_mul.c \


LIBBORINGSSL_EXAMPLE_MUL_LIB_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(LIBBORINGSSL_EXAMPLE_MUL_LIB_SRC))))

# boringssl needs an override to ensure that it does not include
# system openssl headers regardless of other configuration
# we do so here with a target specific variable assignment
$(LIBBORINGSSL_EXAMPLE_MUL_LIB_OBJS): CFLAGS := -Ithird_party/boringssl/include $(CFLAGS) -Wno-sign-conversion -Wno-conversion -Wno-unused-value -fvisibility=hidden
$(LIBBORINGSSL_EXAMPLE_MUL_LIB_OBJS): CXXFLAGS := -Ithird_party/boringssl/include $(CXXFLAGS) -fvisibility=hidden
$(LIBBORINGSSL_EXAMPLE_MUL_LIB_OBJS): CPPFLAGS += -DOPENSSL_NO_ASM -D_GNU_SOURCE

$(LIBDIR)/$(CONFIG)/libboringssl_example_mul_lib.a: $(ZLIB_DEP) $(OPENSSL_DEP)  $(LIBBORINGSSL_EXAMPLE_MUL_LIB_OBJS)
	$(E) "[AR]      Creating $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) rm -f $(LIBDIR)/$(CONFIG)/libboringssl_example_mul_lib.a
	$(Q) $(AR) rcs $(LIBDIR)/$(CONFIG)/libboringssl_example_mul_lib.a $(LIBBORINGSSL_EXAMPLE_MUL_LIB_OBJS)
ifeq ($(SYSTEM),Darwin)
	$(Q) ranlib $(LIBDIR)/$(CONFIG)/libboringssl_example_mul_lib.a
endif




ifneq ($(NO_DEPS),true)
-include $(LIBBORINGSSL_EXAMPLE_MUL_LIB_OBJS:.o=.dep)
endif


LIBBORINGSSL_ECDSA_TEST_LIB_SRC = \
    third_party/boringssl/crypto/ecdsa/ecdsa_test.cc \


LIBBORINGSSL_ECDSA_TEST_LIB_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(LIBBORINGSSL_ECDSA_TEST_LIB_SRC))))

# boringssl needs an override to ensure that it does not include
# system openssl headers regardless of other configuration
# we do so here with a target specific variable assignment
$(LIBBORINGSSL_ECDSA_TEST_LIB_OBJS): CFLAGS := -Ithird_party/boringssl/include $(CFLAGS) -Wno-sign-conversion -Wno-conversion -Wno-unused-value -fvisibility=hidden
$(LIBBORINGSSL_ECDSA_TEST_LIB_OBJS): CXXFLAGS := -Ithird_party/boringssl/include $(CXXFLAGS) -fvisibility=hidden
$(LIBBORINGSSL_ECDSA_TEST_LIB_OBJS): CPPFLAGS += -DOPENSSL_NO_ASM -D_GNU_SOURCE

ifeq ($(NO_PROTOBUF),true)

# You can't build a C++ library if you don't have protobuf - a bit overreached, but still okay.

$(LIBDIR)/$(CONFIG)/libboringssl_ecdsa_test_lib.a: protobuf_dep_error


else

$(LIBDIR)/$(CONFIG)/libboringssl_ecdsa_test_lib.a: $(ZLIB_DEP) $(OPENSSL_DEP)  $(PROTOBUF_DEP) $(LIBBORINGSSL_ECDSA_TEST_LIB_OBJS)
	$(E) "[AR]      Creating $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) rm -f $(LIBDIR)/$(CONFIG)/libboringssl_ecdsa_test_lib.a
	$(Q) $(AR) rcs $(LIBDIR)/$(CONFIG)/libboringssl_ecdsa_test_lib.a $(LIBBORINGSSL_ECDSA_TEST_LIB_OBJS)
ifeq ($(SYSTEM),Darwin)
	$(Q) ranlib $(LIBDIR)/$(CONFIG)/libboringssl_ecdsa_test_lib.a
endif




endif

ifneq ($(NO_DEPS),true)
-include $(LIBBORINGSSL_ECDSA_TEST_LIB_OBJS:.o=.dep)
endif


LIBBORINGSSL_ERR_TEST_LIB_SRC = \
    third_party/boringssl/crypto/err/err_test.cc \


LIBBORINGSSL_ERR_TEST_LIB_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(LIBBORINGSSL_ERR_TEST_LIB_SRC))))

# boringssl needs an override to ensure that it does not include
# system openssl headers regardless of other configuration
# we do so here with a target specific variable assignment
$(LIBBORINGSSL_ERR_TEST_LIB_OBJS): CFLAGS := -Ithird_party/boringssl/include $(CFLAGS) -Wno-sign-conversion -Wno-conversion -Wno-unused-value -fvisibility=hidden
$(LIBBORINGSSL_ERR_TEST_LIB_OBJS): CXXFLAGS := -Ithird_party/boringssl/include $(CXXFLAGS) -fvisibility=hidden
$(LIBBORINGSSL_ERR_TEST_LIB_OBJS): CPPFLAGS += -DOPENSSL_NO_ASM -D_GNU_SOURCE

ifeq ($(NO_PROTOBUF),true)

# You can't build a C++ library if you don't have protobuf - a bit overreached, but still okay.

$(LIBDIR)/$(CONFIG)/libboringssl_err_test_lib.a: protobuf_dep_error


else

$(LIBDIR)/$(CONFIG)/libboringssl_err_test_lib.a: $(ZLIB_DEP) $(OPENSSL_DEP)  $(PROTOBUF_DEP) $(LIBBORINGSSL_ERR_TEST_LIB_OBJS)
	$(E) "[AR]      Creating $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) rm -f $(LIBDIR)/$(CONFIG)/libboringssl_err_test_lib.a
	$(Q) $(AR) rcs $(LIBDIR)/$(CONFIG)/libboringssl_err_test_lib.a $(LIBBORINGSSL_ERR_TEST_LIB_OBJS)
ifeq ($(SYSTEM),Darwin)
	$(Q) ranlib $(LIBDIR)/$(CONFIG)/libboringssl_err_test_lib.a
endif




endif

ifneq ($(NO_DEPS),true)
-include $(LIBBORINGSSL_ERR_TEST_LIB_OBJS:.o=.dep)
endif


LIBBORINGSSL_EVP_EXTRA_TEST_LIB_SRC = \
    third_party/boringssl/crypto/evp/evp_extra_test.cc \


LIBBORINGSSL_EVP_EXTRA_TEST_LIB_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(LIBBORINGSSL_EVP_EXTRA_TEST_LIB_SRC))))

# boringssl needs an override to ensure that it does not include
# system openssl headers regardless of other configuration
# we do so here with a target specific variable assignment
$(LIBBORINGSSL_EVP_EXTRA_TEST_LIB_OBJS): CFLAGS := -Ithird_party/boringssl/include $(CFLAGS) -Wno-sign-conversion -Wno-conversion -Wno-unused-value -fvisibility=hidden
$(LIBBORINGSSL_EVP_EXTRA_TEST_LIB_OBJS): CXXFLAGS := -Ithird_party/boringssl/include $(CXXFLAGS) -fvisibility=hidden
$(LIBBORINGSSL_EVP_EXTRA_TEST_LIB_OBJS): CPPFLAGS += -DOPENSSL_NO_ASM -D_GNU_SOURCE

ifeq ($(NO_PROTOBUF),true)

# You can't build a C++ library if you don't have protobuf - a bit overreached, but still okay.

$(LIBDIR)/$(CONFIG)/libboringssl_evp_extra_test_lib.a: protobuf_dep_error


else

$(LIBDIR)/$(CONFIG)/libboringssl_evp_extra_test_lib.a: $(ZLIB_DEP) $(OPENSSL_DEP)  $(PROTOBUF_DEP) $(LIBBORINGSSL_EVP_EXTRA_TEST_LIB_OBJS)
	$(E) "[AR]      Creating $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) rm -f $(LIBDIR)/$(CONFIG)/libboringssl_evp_extra_test_lib.a
	$(Q) $(AR) rcs $(LIBDIR)/$(CONFIG)/libboringssl_evp_extra_test_lib.a $(LIBBORINGSSL_EVP_EXTRA_TEST_LIB_OBJS)
ifeq ($(SYSTEM),Darwin)
	$(Q) ranlib $(LIBDIR)/$(CONFIG)/libboringssl_evp_extra_test_lib.a
endif




endif

ifneq ($(NO_DEPS),true)
-include $(LIBBORINGSSL_EVP_EXTRA_TEST_LIB_OBJS:.o=.dep)
endif


LIBBORINGSSL_EVP_TEST_LIB_SRC = \
    third_party/boringssl/crypto/evp/evp_test.cc \


LIBBORINGSSL_EVP_TEST_LIB_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(LIBBORINGSSL_EVP_TEST_LIB_SRC))))

# boringssl needs an override to ensure that it does not include
# system openssl headers regardless of other configuration
# we do so here with a target specific variable assignment
$(LIBBORINGSSL_EVP_TEST_LIB_OBJS): CFLAGS := -Ithird_party/boringssl/include $(CFLAGS) -Wno-sign-conversion -Wno-conversion -Wno-unused-value -fvisibility=hidden
$(LIBBORINGSSL_EVP_TEST_LIB_OBJS): CXXFLAGS := -Ithird_party/boringssl/include $(CXXFLAGS) -fvisibility=hidden
$(LIBBORINGSSL_EVP_TEST_LIB_OBJS): CPPFLAGS += -DOPENSSL_NO_ASM -D_GNU_SOURCE

ifeq ($(NO_PROTOBUF),true)

# You can't build a C++ library if you don't have protobuf - a bit overreached, but still okay.

$(LIBDIR)/$(CONFIG)/libboringssl_evp_test_lib.a: protobuf_dep_error


else

$(LIBDIR)/$(CONFIG)/libboringssl_evp_test_lib.a: $(ZLIB_DEP) $(OPENSSL_DEP)  $(PROTOBUF_DEP) $(LIBBORINGSSL_EVP_TEST_LIB_OBJS)
	$(E) "[AR]      Creating $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) rm -f $(LIBDIR)/$(CONFIG)/libboringssl_evp_test_lib.a
	$(Q) $(AR) rcs $(LIBDIR)/$(CONFIG)/libboringssl_evp_test_lib.a $(LIBBORINGSSL_EVP_TEST_LIB_OBJS)
ifeq ($(SYSTEM),Darwin)
	$(Q) ranlib $(LIBDIR)/$(CONFIG)/libboringssl_evp_test_lib.a
endif




endif

ifneq ($(NO_DEPS),true)
-include $(LIBBORINGSSL_EVP_TEST_LIB_OBJS:.o=.dep)
endif


LIBBORINGSSL_PBKDF_TEST_LIB_SRC = \
    third_party/boringssl/crypto/evp/pbkdf_test.cc \


LIBBORINGSSL_PBKDF_TEST_LIB_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(LIBBORINGSSL_PBKDF_TEST_LIB_SRC))))

# boringssl needs an override to ensure that it does not include
# system openssl headers regardless of other configuration
# we do so here with a target specific variable assignment
$(LIBBORINGSSL_PBKDF_TEST_LIB_OBJS): CFLAGS := -Ithird_party/boringssl/include $(CFLAGS) -Wno-sign-conversion -Wno-conversion -Wno-unused-value -fvisibility=hidden
$(LIBBORINGSSL_PBKDF_TEST_LIB_OBJS): CXXFLAGS := -Ithird_party/boringssl/include $(CXXFLAGS) -fvisibility=hidden
$(LIBBORINGSSL_PBKDF_TEST_LIB_OBJS): CPPFLAGS += -DOPENSSL_NO_ASM -D_GNU_SOURCE

ifeq ($(NO_PROTOBUF),true)

# You can't build a C++ library if you don't have protobuf - a bit overreached, but still okay.

$(LIBDIR)/$(CONFIG)/libboringssl_pbkdf_test_lib.a: protobuf_dep_error


else

$(LIBDIR)/$(CONFIG)/libboringssl_pbkdf_test_lib.a: $(ZLIB_DEP) $(OPENSSL_DEP)  $(PROTOBUF_DEP) $(LIBBORINGSSL_PBKDF_TEST_LIB_OBJS)
	$(E) "[AR]      Creating $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) rm -f $(LIBDIR)/$(CONFIG)/libboringssl_pbkdf_test_lib.a
	$(Q) $(AR) rcs $(LIBDIR)/$(CONFIG)/libboringssl_pbkdf_test_lib.a $(LIBBORINGSSL_PBKDF_TEST_LIB_OBJS)
ifeq ($(SYSTEM),Darwin)
	$(Q) ranlib $(LIBDIR)/$(CONFIG)/libboringssl_pbkdf_test_lib.a
endif




endif

ifneq ($(NO_DEPS),true)
-include $(LIBBORINGSSL_PBKDF_TEST_LIB_OBJS:.o=.dep)
endif


LIBBORINGSSL_HKDF_TEST_LIB_SRC = \
    third_party/boringssl/crypto/hkdf/hkdf_test.c \


LIBBORINGSSL_HKDF_TEST_LIB_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(LIBBORINGSSL_HKDF_TEST_LIB_SRC))))

# boringssl needs an override to ensure that it does not include
# system openssl headers regardless of other configuration
# we do so here with a target specific variable assignment
$(LIBBORINGSSL_HKDF_TEST_LIB_OBJS): CFLAGS := -Ithird_party/boringssl/include $(CFLAGS) -Wno-sign-conversion -Wno-conversion -Wno-unused-value -fvisibility=hidden
$(LIBBORINGSSL_HKDF_TEST_LIB_OBJS): CXXFLAGS := -Ithird_party/boringssl/include $(CXXFLAGS) -fvisibility=hidden
$(LIBBORINGSSL_HKDF_TEST_LIB_OBJS): CPPFLAGS += -DOPENSSL_NO_ASM -D_GNU_SOURCE

$(LIBDIR)/$(CONFIG)/libboringssl_hkdf_test_lib.a: $(ZLIB_DEP) $(OPENSSL_DEP)  $(LIBBORINGSSL_HKDF_TEST_LIB_OBJS)
	$(E) "[AR]      Creating $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) rm -f $(LIBDIR)/$(CONFIG)/libboringssl_hkdf_test_lib.a
	$(Q) $(AR) rcs $(LIBDIR)/$(CONFIG)/libboringssl_hkdf_test_lib.a $(LIBBORINGSSL_HKDF_TEST_LIB_OBJS)
ifeq ($(SYSTEM),Darwin)
	$(Q) ranlib $(LIBDIR)/$(CONFIG)/libboringssl_hkdf_test_lib.a
endif




ifneq ($(NO_DEPS),true)
-include $(LIBBORINGSSL_HKDF_TEST_LIB_OBJS:.o=.dep)
endif


LIBBORINGSSL_HMAC_TEST_LIB_SRC = \
    third_party/boringssl/crypto/hmac/hmac_test.cc \


LIBBORINGSSL_HMAC_TEST_LIB_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(LIBBORINGSSL_HMAC_TEST_LIB_SRC))))

# boringssl needs an override to ensure that it does not include
# system openssl headers regardless of other configuration
# we do so here with a target specific variable assignment
$(LIBBORINGSSL_HMAC_TEST_LIB_OBJS): CFLAGS := -Ithird_party/boringssl/include $(CFLAGS) -Wno-sign-conversion -Wno-conversion -Wno-unused-value -fvisibility=hidden
$(LIBBORINGSSL_HMAC_TEST_LIB_OBJS): CXXFLAGS := -Ithird_party/boringssl/include $(CXXFLAGS) -fvisibility=hidden
$(LIBBORINGSSL_HMAC_TEST_LIB_OBJS): CPPFLAGS += -DOPENSSL_NO_ASM -D_GNU_SOURCE

ifeq ($(NO_PROTOBUF),true)

# You can't build a C++ library if you don't have protobuf - a bit overreached, but still okay.

$(LIBDIR)/$(CONFIG)/libboringssl_hmac_test_lib.a: protobuf_dep_error


else

$(LIBDIR)/$(CONFIG)/libboringssl_hmac_test_lib.a: $(ZLIB_DEP) $(OPENSSL_DEP)  $(PROTOBUF_DEP) $(LIBBORINGSSL_HMAC_TEST_LIB_OBJS)
	$(E) "[AR]      Creating $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) rm -f $(LIBDIR)/$(CONFIG)/libboringssl_hmac_test_lib.a
	$(Q) $(AR) rcs $(LIBDIR)/$(CONFIG)/libboringssl_hmac_test_lib.a $(LIBBORINGSSL_HMAC_TEST_LIB_OBJS)
ifeq ($(SYSTEM),Darwin)
	$(Q) ranlib $(LIBDIR)/$(CONFIG)/libboringssl_hmac_test_lib.a
endif




endif

ifneq ($(NO_DEPS),true)
-include $(LIBBORINGSSL_HMAC_TEST_LIB_OBJS:.o=.dep)
endif


LIBBORINGSSL_LHASH_TEST_LIB_SRC = \
    third_party/boringssl/crypto/lhash/lhash_test.c \


LIBBORINGSSL_LHASH_TEST_LIB_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(LIBBORINGSSL_LHASH_TEST_LIB_SRC))))

# boringssl needs an override to ensure that it does not include
# system openssl headers regardless of other configuration
# we do so here with a target specific variable assignment
$(LIBBORINGSSL_LHASH_TEST_LIB_OBJS): CFLAGS := -Ithird_party/boringssl/include $(CFLAGS) -Wno-sign-conversion -Wno-conversion -Wno-unused-value -fvisibility=hidden
$(LIBBORINGSSL_LHASH_TEST_LIB_OBJS): CXXFLAGS := -Ithird_party/boringssl/include $(CXXFLAGS) -fvisibility=hidden
$(LIBBORINGSSL_LHASH_TEST_LIB_OBJS): CPPFLAGS += -DOPENSSL_NO_ASM -D_GNU_SOURCE

$(LIBDIR)/$(CONFIG)/libboringssl_lhash_test_lib.a: $(ZLIB_DEP) $(OPENSSL_DEP)  $(LIBBORINGSSL_LHASH_TEST_LIB_OBJS)
	$(E) "[AR]      Creating $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) rm -f $(LIBDIR)/$(CONFIG)/libboringssl_lhash_test_lib.a
	$(Q) $(AR) rcs $(LIBDIR)/$(CONFIG)/libboringssl_lhash_test_lib.a $(LIBBORINGSSL_LHASH_TEST_LIB_OBJS)
ifeq ($(SYSTEM),Darwin)
	$(Q) ranlib $(LIBDIR)/$(CONFIG)/libboringssl_lhash_test_lib.a
endif




ifneq ($(NO_DEPS),true)
-include $(LIBBORINGSSL_LHASH_TEST_LIB_OBJS:.o=.dep)
endif


LIBBORINGSSL_GCM_TEST_LIB_SRC = \
    third_party/boringssl/crypto/modes/gcm_test.c \


LIBBORINGSSL_GCM_TEST_LIB_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(LIBBORINGSSL_GCM_TEST_LIB_SRC))))

# boringssl needs an override to ensure that it does not include
# system openssl headers regardless of other configuration
# we do so here with a target specific variable assignment
$(LIBBORINGSSL_GCM_TEST_LIB_OBJS): CFLAGS := -Ithird_party/boringssl/include $(CFLAGS) -Wno-sign-conversion -Wno-conversion -Wno-unused-value -fvisibility=hidden
$(LIBBORINGSSL_GCM_TEST_LIB_OBJS): CXXFLAGS := -Ithird_party/boringssl/include $(CXXFLAGS) -fvisibility=hidden
$(LIBBORINGSSL_GCM_TEST_LIB_OBJS): CPPFLAGS += -DOPENSSL_NO_ASM -D_GNU_SOURCE

$(LIBDIR)/$(CONFIG)/libboringssl_gcm_test_lib.a: $(ZLIB_DEP) $(OPENSSL_DEP)  $(LIBBORINGSSL_GCM_TEST_LIB_OBJS)
	$(E) "[AR]      Creating $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) rm -f $(LIBDIR)/$(CONFIG)/libboringssl_gcm_test_lib.a
	$(Q) $(AR) rcs $(LIBDIR)/$(CONFIG)/libboringssl_gcm_test_lib.a $(LIBBORINGSSL_GCM_TEST_LIB_OBJS)
ifeq ($(SYSTEM),Darwin)
	$(Q) ranlib $(LIBDIR)/$(CONFIG)/libboringssl_gcm_test_lib.a
endif




ifneq ($(NO_DEPS),true)
-include $(LIBBORINGSSL_GCM_TEST_LIB_OBJS:.o=.dep)
endif


LIBBORINGSSL_PKCS12_TEST_LIB_SRC = \
    third_party/boringssl/crypto/pkcs8/pkcs12_test.cc \


LIBBORINGSSL_PKCS12_TEST_LIB_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(LIBBORINGSSL_PKCS12_TEST_LIB_SRC))))

# boringssl needs an override to ensure that it does not include
# system openssl headers regardless of other configuration
# we do so here with a target specific variable assignment
$(LIBBORINGSSL_PKCS12_TEST_LIB_OBJS): CFLAGS := -Ithird_party/boringssl/include $(CFLAGS) -Wno-sign-conversion -Wno-conversion -Wno-unused-value -fvisibility=hidden
$(LIBBORINGSSL_PKCS12_TEST_LIB_OBJS): CXXFLAGS := -Ithird_party/boringssl/include $(CXXFLAGS) -fvisibility=hidden
$(LIBBORINGSSL_PKCS12_TEST_LIB_OBJS): CPPFLAGS += -DOPENSSL_NO_ASM -D_GNU_SOURCE

ifeq ($(NO_PROTOBUF),true)

# You can't build a C++ library if you don't have protobuf - a bit overreached, but still okay.

$(LIBDIR)/$(CONFIG)/libboringssl_pkcs12_test_lib.a: protobuf_dep_error


else

$(LIBDIR)/$(CONFIG)/libboringssl_pkcs12_test_lib.a: $(ZLIB_DEP) $(OPENSSL_DEP)  $(PROTOBUF_DEP) $(LIBBORINGSSL_PKCS12_TEST_LIB_OBJS)
	$(E) "[AR]      Creating $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) rm -f $(LIBDIR)/$(CONFIG)/libboringssl_pkcs12_test_lib.a
	$(Q) $(AR) rcs $(LIBDIR)/$(CONFIG)/libboringssl_pkcs12_test_lib.a $(LIBBORINGSSL_PKCS12_TEST_LIB_OBJS)
ifeq ($(SYSTEM),Darwin)
	$(Q) ranlib $(LIBDIR)/$(CONFIG)/libboringssl_pkcs12_test_lib.a
endif




endif

ifneq ($(NO_DEPS),true)
-include $(LIBBORINGSSL_PKCS12_TEST_LIB_OBJS:.o=.dep)
endif


LIBBORINGSSL_PKCS8_TEST_LIB_SRC = \
    third_party/boringssl/crypto/pkcs8/pkcs8_test.cc \


LIBBORINGSSL_PKCS8_TEST_LIB_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(LIBBORINGSSL_PKCS8_TEST_LIB_SRC))))

# boringssl needs an override to ensure that it does not include
# system openssl headers regardless of other configuration
# we do so here with a target specific variable assignment
$(LIBBORINGSSL_PKCS8_TEST_LIB_OBJS): CFLAGS := -Ithird_party/boringssl/include $(CFLAGS) -Wno-sign-conversion -Wno-conversion -Wno-unused-value -fvisibility=hidden
$(LIBBORINGSSL_PKCS8_TEST_LIB_OBJS): CXXFLAGS := -Ithird_party/boringssl/include $(CXXFLAGS) -fvisibility=hidden
$(LIBBORINGSSL_PKCS8_TEST_LIB_OBJS): CPPFLAGS += -DOPENSSL_NO_ASM -D_GNU_SOURCE

ifeq ($(NO_PROTOBUF),true)

# You can't build a C++ library if you don't have protobuf - a bit overreached, but still okay.

$(LIBDIR)/$(CONFIG)/libboringssl_pkcs8_test_lib.a: protobuf_dep_error


else

$(LIBDIR)/$(CONFIG)/libboringssl_pkcs8_test_lib.a: $(ZLIB_DEP) $(OPENSSL_DEP)  $(PROTOBUF_DEP) $(LIBBORINGSSL_PKCS8_TEST_LIB_OBJS)
	$(E) "[AR]      Creating $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) rm -f $(LIBDIR)/$(CONFIG)/libboringssl_pkcs8_test_lib.a
	$(Q) $(AR) rcs $(LIBDIR)/$(CONFIG)/libboringssl_pkcs8_test_lib.a $(LIBBORINGSSL_PKCS8_TEST_LIB_OBJS)
ifeq ($(SYSTEM),Darwin)
	$(Q) ranlib $(LIBDIR)/$(CONFIG)/libboringssl_pkcs8_test_lib.a
endif




endif

ifneq ($(NO_DEPS),true)
-include $(LIBBORINGSSL_PKCS8_TEST_LIB_OBJS:.o=.dep)
endif


LIBBORINGSSL_POLY1305_TEST_LIB_SRC = \
    third_party/boringssl/crypto/poly1305/poly1305_test.cc \


LIBBORINGSSL_POLY1305_TEST_LIB_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(LIBBORINGSSL_POLY1305_TEST_LIB_SRC))))

# boringssl needs an override to ensure that it does not include
# system openssl headers regardless of other configuration
# we do so here with a target specific variable assignment
$(LIBBORINGSSL_POLY1305_TEST_LIB_OBJS): CFLAGS := -Ithird_party/boringssl/include $(CFLAGS) -Wno-sign-conversion -Wno-conversion -Wno-unused-value -fvisibility=hidden
$(LIBBORINGSSL_POLY1305_TEST_LIB_OBJS): CXXFLAGS := -Ithird_party/boringssl/include $(CXXFLAGS) -fvisibility=hidden
$(LIBBORINGSSL_POLY1305_TEST_LIB_OBJS): CPPFLAGS += -DOPENSSL_NO_ASM -D_GNU_SOURCE

ifeq ($(NO_PROTOBUF),true)

# You can't build a C++ library if you don't have protobuf - a bit overreached, but still okay.

$(LIBDIR)/$(CONFIG)/libboringssl_poly1305_test_lib.a: protobuf_dep_error


else

$(LIBDIR)/$(CONFIG)/libboringssl_poly1305_test_lib.a: $(ZLIB_DEP) $(OPENSSL_DEP)  $(PROTOBUF_DEP) $(LIBBORINGSSL_POLY1305_TEST_LIB_OBJS)
	$(E) "[AR]      Creating $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) rm -f $(LIBDIR)/$(CONFIG)/libboringssl_poly1305_test_lib.a
	$(Q) $(AR) rcs $(LIBDIR)/$(CONFIG)/libboringssl_poly1305_test_lib.a $(LIBBORINGSSL_POLY1305_TEST_LIB_OBJS)
ifeq ($(SYSTEM),Darwin)
	$(Q) ranlib $(LIBDIR)/$(CONFIG)/libboringssl_poly1305_test_lib.a
endif




endif

ifneq ($(NO_DEPS),true)
-include $(LIBBORINGSSL_POLY1305_TEST_LIB_OBJS:.o=.dep)
endif


LIBBORINGSSL_REFCOUNT_TEST_LIB_SRC = \
    third_party/boringssl/crypto/refcount_test.c \


LIBBORINGSSL_REFCOUNT_TEST_LIB_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(LIBBORINGSSL_REFCOUNT_TEST_LIB_SRC))))

# boringssl needs an override to ensure that it does not include
# system openssl headers regardless of other configuration
# we do so here with a target specific variable assignment
$(LIBBORINGSSL_REFCOUNT_TEST_LIB_OBJS): CFLAGS := -Ithird_party/boringssl/include $(CFLAGS) -Wno-sign-conversion -Wno-conversion -Wno-unused-value -fvisibility=hidden
$(LIBBORINGSSL_REFCOUNT_TEST_LIB_OBJS): CXXFLAGS := -Ithird_party/boringssl/include $(CXXFLAGS) -fvisibility=hidden
$(LIBBORINGSSL_REFCOUNT_TEST_LIB_OBJS): CPPFLAGS += -DOPENSSL_NO_ASM -D_GNU_SOURCE

$(LIBDIR)/$(CONFIG)/libboringssl_refcount_test_lib.a: $(ZLIB_DEP) $(OPENSSL_DEP)  $(LIBBORINGSSL_REFCOUNT_TEST_LIB_OBJS)
	$(E) "[AR]      Creating $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) rm -f $(LIBDIR)/$(CONFIG)/libboringssl_refcount_test_lib.a
	$(Q) $(AR) rcs $(LIBDIR)/$(CONFIG)/libboringssl_refcount_test_lib.a $(LIBBORINGSSL_REFCOUNT_TEST_LIB_OBJS)
ifeq ($(SYSTEM),Darwin)
	$(Q) ranlib $(LIBDIR)/$(CONFIG)/libboringssl_refcount_test_lib.a
endif




ifneq ($(NO_DEPS),true)
-include $(LIBBORINGSSL_REFCOUNT_TEST_LIB_OBJS:.o=.dep)
endif


LIBBORINGSSL_RSA_TEST_LIB_SRC = \
    third_party/boringssl/crypto/rsa/rsa_test.cc \


LIBBORINGSSL_RSA_TEST_LIB_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(LIBBORINGSSL_RSA_TEST_LIB_SRC))))

# boringssl needs an override to ensure that it does not include
# system openssl headers regardless of other configuration
# we do so here with a target specific variable assignment
$(LIBBORINGSSL_RSA_TEST_LIB_OBJS): CFLAGS := -Ithird_party/boringssl/include $(CFLAGS) -Wno-sign-conversion -Wno-conversion -Wno-unused-value -fvisibility=hidden
$(LIBBORINGSSL_RSA_TEST_LIB_OBJS): CXXFLAGS := -Ithird_party/boringssl/include $(CXXFLAGS) -fvisibility=hidden
$(LIBBORINGSSL_RSA_TEST_LIB_OBJS): CPPFLAGS += -DOPENSSL_NO_ASM -D_GNU_SOURCE

ifeq ($(NO_PROTOBUF),true)

# You can't build a C++ library if you don't have protobuf - a bit overreached, but still okay.

$(LIBDIR)/$(CONFIG)/libboringssl_rsa_test_lib.a: protobuf_dep_error


else

$(LIBDIR)/$(CONFIG)/libboringssl_rsa_test_lib.a: $(ZLIB_DEP) $(OPENSSL_DEP)  $(PROTOBUF_DEP) $(LIBBORINGSSL_RSA_TEST_LIB_OBJS)
	$(E) "[AR]      Creating $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) rm -f $(LIBDIR)/$(CONFIG)/libboringssl_rsa_test_lib.a
	$(Q) $(AR) rcs $(LIBDIR)/$(CONFIG)/libboringssl_rsa_test_lib.a $(LIBBORINGSSL_RSA_TEST_LIB_OBJS)
ifeq ($(SYSTEM),Darwin)
	$(Q) ranlib $(LIBDIR)/$(CONFIG)/libboringssl_rsa_test_lib.a
endif




endif

ifneq ($(NO_DEPS),true)
-include $(LIBBORINGSSL_RSA_TEST_LIB_OBJS:.o=.dep)
endif


LIBBORINGSSL_THREAD_TEST_LIB_SRC = \
    third_party/boringssl/crypto/thread_test.c \


LIBBORINGSSL_THREAD_TEST_LIB_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(LIBBORINGSSL_THREAD_TEST_LIB_SRC))))

# boringssl needs an override to ensure that it does not include
# system openssl headers regardless of other configuration
# we do so here with a target specific variable assignment
$(LIBBORINGSSL_THREAD_TEST_LIB_OBJS): CFLAGS := -Ithird_party/boringssl/include $(CFLAGS) -Wno-sign-conversion -Wno-conversion -Wno-unused-value -fvisibility=hidden
$(LIBBORINGSSL_THREAD_TEST_LIB_OBJS): CXXFLAGS := -Ithird_party/boringssl/include $(CXXFLAGS) -fvisibility=hidden
$(LIBBORINGSSL_THREAD_TEST_LIB_OBJS): CPPFLAGS += -DOPENSSL_NO_ASM -D_GNU_SOURCE

$(LIBDIR)/$(CONFIG)/libboringssl_thread_test_lib.a: $(ZLIB_DEP) $(OPENSSL_DEP)  $(LIBBORINGSSL_THREAD_TEST_LIB_OBJS)
	$(E) "[AR]      Creating $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) rm -f $(LIBDIR)/$(CONFIG)/libboringssl_thread_test_lib.a
	$(Q) $(AR) rcs $(LIBDIR)/$(CONFIG)/libboringssl_thread_test_lib.a $(LIBBORINGSSL_THREAD_TEST_LIB_OBJS)
ifeq ($(SYSTEM),Darwin)
	$(Q) ranlib $(LIBDIR)/$(CONFIG)/libboringssl_thread_test_lib.a
endif




ifneq ($(NO_DEPS),true)
-include $(LIBBORINGSSL_THREAD_TEST_LIB_OBJS:.o=.dep)
endif


LIBBORINGSSL_PKCS7_TEST_LIB_SRC = \
    third_party/boringssl/crypto/x509/pkcs7_test.c \


LIBBORINGSSL_PKCS7_TEST_LIB_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(LIBBORINGSSL_PKCS7_TEST_LIB_SRC))))

# boringssl needs an override to ensure that it does not include
# system openssl headers regardless of other configuration
# we do so here with a target specific variable assignment
$(LIBBORINGSSL_PKCS7_TEST_LIB_OBJS): CFLAGS := -Ithird_party/boringssl/include $(CFLAGS) -Wno-sign-conversion -Wno-conversion -Wno-unused-value -fvisibility=hidden
$(LIBBORINGSSL_PKCS7_TEST_LIB_OBJS): CXXFLAGS := -Ithird_party/boringssl/include $(CXXFLAGS) -fvisibility=hidden
$(LIBBORINGSSL_PKCS7_TEST_LIB_OBJS): CPPFLAGS += -DOPENSSL_NO_ASM -D_GNU_SOURCE

$(LIBDIR)/$(CONFIG)/libboringssl_pkcs7_test_lib.a: $(ZLIB_DEP) $(OPENSSL_DEP)  $(LIBBORINGSSL_PKCS7_TEST_LIB_OBJS)
	$(E) "[AR]      Creating $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) rm -f $(LIBDIR)/$(CONFIG)/libboringssl_pkcs7_test_lib.a
	$(Q) $(AR) rcs $(LIBDIR)/$(CONFIG)/libboringssl_pkcs7_test_lib.a $(LIBBORINGSSL_PKCS7_TEST_LIB_OBJS)
ifeq ($(SYSTEM),Darwin)
	$(Q) ranlib $(LIBDIR)/$(CONFIG)/libboringssl_pkcs7_test_lib.a
endif




ifneq ($(NO_DEPS),true)
-include $(LIBBORINGSSL_PKCS7_TEST_LIB_OBJS:.o=.dep)
endif


LIBBORINGSSL_TAB_TEST_LIB_SRC = \
    third_party/boringssl/crypto/x509v3/tab_test.c \


LIBBORINGSSL_TAB_TEST_LIB_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(LIBBORINGSSL_TAB_TEST_LIB_SRC))))

# boringssl needs an override to ensure that it does not include
# system openssl headers regardless of other configuration
# we do so here with a target specific variable assignment
$(LIBBORINGSSL_TAB_TEST_LIB_OBJS): CFLAGS := -Ithird_party/boringssl/include $(CFLAGS) -Wno-sign-conversion -Wno-conversion -Wno-unused-value -fvisibility=hidden
$(LIBBORINGSSL_TAB_TEST_LIB_OBJS): CXXFLAGS := -Ithird_party/boringssl/include $(CXXFLAGS) -fvisibility=hidden
$(LIBBORINGSSL_TAB_TEST_LIB_OBJS): CPPFLAGS += -DOPENSSL_NO_ASM -D_GNU_SOURCE

$(LIBDIR)/$(CONFIG)/libboringssl_tab_test_lib.a: $(ZLIB_DEP) $(OPENSSL_DEP)  $(LIBBORINGSSL_TAB_TEST_LIB_OBJS)
	$(E) "[AR]      Creating $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) rm -f $(LIBDIR)/$(CONFIG)/libboringssl_tab_test_lib.a
	$(Q) $(AR) rcs $(LIBDIR)/$(CONFIG)/libboringssl_tab_test_lib.a $(LIBBORINGSSL_TAB_TEST_LIB_OBJS)
ifeq ($(SYSTEM),Darwin)
	$(Q) ranlib $(LIBDIR)/$(CONFIG)/libboringssl_tab_test_lib.a
endif




ifneq ($(NO_DEPS),true)
-include $(LIBBORINGSSL_TAB_TEST_LIB_OBJS:.o=.dep)
endif


LIBBORINGSSL_V3NAME_TEST_LIB_SRC = \
    third_party/boringssl/crypto/x509v3/v3name_test.c \


LIBBORINGSSL_V3NAME_TEST_LIB_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(LIBBORINGSSL_V3NAME_TEST_LIB_SRC))))

# boringssl needs an override to ensure that it does not include
# system openssl headers regardless of other configuration
# we do so here with a target specific variable assignment
$(LIBBORINGSSL_V3NAME_TEST_LIB_OBJS): CFLAGS := -Ithird_party/boringssl/include $(CFLAGS) -Wno-sign-conversion -Wno-conversion -Wno-unused-value -fvisibility=hidden
$(LIBBORINGSSL_V3NAME_TEST_LIB_OBJS): CXXFLAGS := -Ithird_party/boringssl/include $(CXXFLAGS) -fvisibility=hidden
$(LIBBORINGSSL_V3NAME_TEST_LIB_OBJS): CPPFLAGS += -DOPENSSL_NO_ASM -D_GNU_SOURCE

$(LIBDIR)/$(CONFIG)/libboringssl_v3name_test_lib.a: $(ZLIB_DEP) $(OPENSSL_DEP)  $(LIBBORINGSSL_V3NAME_TEST_LIB_OBJS)
	$(E) "[AR]      Creating $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) rm -f $(LIBDIR)/$(CONFIG)/libboringssl_v3name_test_lib.a
	$(Q) $(AR) rcs $(LIBDIR)/$(CONFIG)/libboringssl_v3name_test_lib.a $(LIBBORINGSSL_V3NAME_TEST_LIB_OBJS)
ifeq ($(SYSTEM),Darwin)
	$(Q) ranlib $(LIBDIR)/$(CONFIG)/libboringssl_v3name_test_lib.a
endif




ifneq ($(NO_DEPS),true)
-include $(LIBBORINGSSL_V3NAME_TEST_LIB_OBJS:.o=.dep)
endif


LIBBORINGSSL_PQUEUE_TEST_LIB_SRC = \
    third_party/boringssl/ssl/pqueue/pqueue_test.c \


LIBBORINGSSL_PQUEUE_TEST_LIB_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(LIBBORINGSSL_PQUEUE_TEST_LIB_SRC))))

# boringssl needs an override to ensure that it does not include
# system openssl headers regardless of other configuration
# we do so here with a target specific variable assignment
$(LIBBORINGSSL_PQUEUE_TEST_LIB_OBJS): CFLAGS := -Ithird_party/boringssl/include $(CFLAGS) -Wno-sign-conversion -Wno-conversion -Wno-unused-value -fvisibility=hidden
$(LIBBORINGSSL_PQUEUE_TEST_LIB_OBJS): CXXFLAGS := -Ithird_party/boringssl/include $(CXXFLAGS) -fvisibility=hidden
$(LIBBORINGSSL_PQUEUE_TEST_LIB_OBJS): CPPFLAGS += -DOPENSSL_NO_ASM -D_GNU_SOURCE

$(LIBDIR)/$(CONFIG)/libboringssl_pqueue_test_lib.a: $(ZLIB_DEP) $(OPENSSL_DEP)  $(LIBBORINGSSL_PQUEUE_TEST_LIB_OBJS)
	$(E) "[AR]      Creating $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) rm -f $(LIBDIR)/$(CONFIG)/libboringssl_pqueue_test_lib.a
	$(Q) $(AR) rcs $(LIBDIR)/$(CONFIG)/libboringssl_pqueue_test_lib.a $(LIBBORINGSSL_PQUEUE_TEST_LIB_OBJS)
ifeq ($(SYSTEM),Darwin)
	$(Q) ranlib $(LIBDIR)/$(CONFIG)/libboringssl_pqueue_test_lib.a
endif




ifneq ($(NO_DEPS),true)
-include $(LIBBORINGSSL_PQUEUE_TEST_LIB_OBJS:.o=.dep)
endif


LIBBORINGSSL_SSL_TEST_LIB_SRC = \
    third_party/boringssl/ssl/ssl_test.cc \


LIBBORINGSSL_SSL_TEST_LIB_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(LIBBORINGSSL_SSL_TEST_LIB_SRC))))

# boringssl needs an override to ensure that it does not include
# system openssl headers regardless of other configuration
# we do so here with a target specific variable assignment
$(LIBBORINGSSL_SSL_TEST_LIB_OBJS): CFLAGS := -Ithird_party/boringssl/include $(CFLAGS) -Wno-sign-conversion -Wno-conversion -Wno-unused-value -fvisibility=hidden
$(LIBBORINGSSL_SSL_TEST_LIB_OBJS): CXXFLAGS := -Ithird_party/boringssl/include $(CXXFLAGS) -fvisibility=hidden
$(LIBBORINGSSL_SSL_TEST_LIB_OBJS): CPPFLAGS += -DOPENSSL_NO_ASM -D_GNU_SOURCE

ifeq ($(NO_PROTOBUF),true)

# You can't build a C++ library if you don't have protobuf - a bit overreached, but still okay.

$(LIBDIR)/$(CONFIG)/libboringssl_ssl_test_lib.a: protobuf_dep_error


else

$(LIBDIR)/$(CONFIG)/libboringssl_ssl_test_lib.a: $(ZLIB_DEP) $(OPENSSL_DEP)  $(PROTOBUF_DEP) $(LIBBORINGSSL_SSL_TEST_LIB_OBJS)
	$(E) "[AR]      Creating $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) rm -f $(LIBDIR)/$(CONFIG)/libboringssl_ssl_test_lib.a
	$(Q) $(AR) rcs $(LIBDIR)/$(CONFIG)/libboringssl_ssl_test_lib.a $(LIBBORINGSSL_SSL_TEST_LIB_OBJS)
ifeq ($(SYSTEM),Darwin)
	$(Q) ranlib $(LIBDIR)/$(CONFIG)/libboringssl_ssl_test_lib.a
endif




endif

ifneq ($(NO_DEPS),true)
-include $(LIBBORINGSSL_SSL_TEST_LIB_OBJS:.o=.dep)
endif


LIBEND2END_TESTS_SRC = \
    test/core/end2end/end2end_tests.c \
    test/core/end2end/tests/bad_hostname.c \
    test/core/end2end/tests/binary_metadata.c \
    test/core/end2end/tests/call_creds.c \
    test/core/end2end/tests/cancel_after_accept.c \
    test/core/end2end/tests/cancel_after_client_done.c \
    test/core/end2end/tests/cancel_after_invoke.c \
    test/core/end2end/tests/cancel_before_invoke.c \
    test/core/end2end/tests/cancel_in_a_vacuum.c \
    test/core/end2end/tests/cancel_with_status.c \
    test/core/end2end/tests/channel_connectivity.c \
    test/core/end2end/tests/channel_ping.c \
    test/core/end2end/tests/compressed_payload.c \
    test/core/end2end/tests/default_host.c \
    test/core/end2end/tests/disappearing_server.c \
    test/core/end2end/tests/empty_batch.c \
    test/core/end2end/tests/graceful_server_shutdown.c \
    test/core/end2end/tests/high_initial_seqno.c \
    test/core/end2end/tests/hpack_size.c \
    test/core/end2end/tests/invoke_large_request.c \
    test/core/end2end/tests/large_metadata.c \
    test/core/end2end/tests/max_concurrent_streams.c \
    test/core/end2end/tests/max_message_length.c \
    test/core/end2end/tests/metadata.c \
    test/core/end2end/tests/negative_deadline.c \
    test/core/end2end/tests/no_op.c \
    test/core/end2end/tests/payload.c \
    test/core/end2end/tests/ping_pong_streaming.c \
    test/core/end2end/tests/registered_call.c \
    test/core/end2end/tests/request_with_flags.c \
    test/core/end2end/tests/request_with_payload.c \
    test/core/end2end/tests/server_finishes_request.c \
    test/core/end2end/tests/shutdown_finishes_calls.c \
    test/core/end2end/tests/shutdown_finishes_tags.c \
    test/core/end2end/tests/simple_delayed_request.c \
    test/core/end2end/tests/simple_request.c \
    test/core/end2end/tests/trailing_metadata.c \


LIBEND2END_TESTS_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(LIBEND2END_TESTS_SRC))))


ifeq ($(NO_SECURE),true)

# You can't build secure libraries if you don't have OpenSSL.

$(LIBDIR)/$(CONFIG)/libend2end_tests.a: openssl_dep_error


else


$(LIBDIR)/$(CONFIG)/libend2end_tests.a: $(ZLIB_DEP) $(OPENSSL_DEP) $(LIBEND2END_TESTS_OBJS)
	$(E) "[AR]      Creating $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) rm -f $(LIBDIR)/$(CONFIG)/libend2end_tests.a
	$(Q) $(AR) rcs $(LIBDIR)/$(CONFIG)/libend2end_tests.a $(LIBEND2END_TESTS_OBJS)
ifeq ($(SYSTEM),Darwin)
	$(Q) ranlib $(LIBDIR)/$(CONFIG)/libend2end_tests.a
endif




endif

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(LIBEND2END_TESTS_OBJS:.o=.dep)
endif
endif


LIBEND2END_NOSEC_TESTS_SRC = \
    test/core/end2end/end2end_nosec_tests.c \
    test/core/end2end/tests/bad_hostname.c \
    test/core/end2end/tests/binary_metadata.c \
    test/core/end2end/tests/cancel_after_accept.c \
    test/core/end2end/tests/cancel_after_client_done.c \
    test/core/end2end/tests/cancel_after_invoke.c \
    test/core/end2end/tests/cancel_before_invoke.c \
    test/core/end2end/tests/cancel_in_a_vacuum.c \
    test/core/end2end/tests/cancel_with_status.c \
    test/core/end2end/tests/channel_connectivity.c \
    test/core/end2end/tests/channel_ping.c \
    test/core/end2end/tests/compressed_payload.c \
    test/core/end2end/tests/default_host.c \
    test/core/end2end/tests/disappearing_server.c \
    test/core/end2end/tests/empty_batch.c \
    test/core/end2end/tests/graceful_server_shutdown.c \
    test/core/end2end/tests/high_initial_seqno.c \
    test/core/end2end/tests/hpack_size.c \
    test/core/end2end/tests/invoke_large_request.c \
    test/core/end2end/tests/large_metadata.c \
    test/core/end2end/tests/max_concurrent_streams.c \
    test/core/end2end/tests/max_message_length.c \
    test/core/end2end/tests/metadata.c \
    test/core/end2end/tests/negative_deadline.c \
    test/core/end2end/tests/no_op.c \
    test/core/end2end/tests/payload.c \
    test/core/end2end/tests/ping_pong_streaming.c \
    test/core/end2end/tests/registered_call.c \
    test/core/end2end/tests/request_with_flags.c \
    test/core/end2end/tests/request_with_payload.c \
    test/core/end2end/tests/server_finishes_request.c \
    test/core/end2end/tests/shutdown_finishes_calls.c \
    test/core/end2end/tests/shutdown_finishes_tags.c \
    test/core/end2end/tests/simple_delayed_request.c \
    test/core/end2end/tests/simple_request.c \
    test/core/end2end/tests/trailing_metadata.c \


LIBEND2END_NOSEC_TESTS_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(LIBEND2END_NOSEC_TESTS_SRC))))


$(LIBDIR)/$(CONFIG)/libend2end_nosec_tests.a: $(ZLIB_DEP) $(OPENSSL_DEP)  $(LIBEND2END_NOSEC_TESTS_OBJS)
	$(E) "[AR]      Creating $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) rm -f $(LIBDIR)/$(CONFIG)/libend2end_nosec_tests.a
	$(Q) $(AR) rcs $(LIBDIR)/$(CONFIG)/libend2end_nosec_tests.a $(LIBEND2END_NOSEC_TESTS_OBJS)
ifeq ($(SYSTEM),Darwin)
	$(Q) ranlib $(LIBDIR)/$(CONFIG)/libend2end_nosec_tests.a
endif




ifneq ($(NO_DEPS),true)
-include $(LIBEND2END_NOSEC_TESTS_OBJS:.o=.dep)
endif


LIBEND2END_CERTS_SRC = \
    test/core/end2end/data/test_root_cert.c \
    test/core/end2end/data/server1_cert.c \
    test/core/end2end/data/server1_key.c \


LIBEND2END_CERTS_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(LIBEND2END_CERTS_SRC))))


ifeq ($(NO_SECURE),true)

# You can't build secure libraries if you don't have OpenSSL.

$(LIBDIR)/$(CONFIG)/libend2end_certs.a: openssl_dep_error


else


$(LIBDIR)/$(CONFIG)/libend2end_certs.a: $(ZLIB_DEP) $(OPENSSL_DEP) $(LIBEND2END_CERTS_OBJS)
	$(E) "[AR]      Creating $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) rm -f $(LIBDIR)/$(CONFIG)/libend2end_certs.a
	$(Q) $(AR) rcs $(LIBDIR)/$(CONFIG)/libend2end_certs.a $(LIBEND2END_CERTS_OBJS)
ifeq ($(SYSTEM),Darwin)
	$(Q) ranlib $(LIBDIR)/$(CONFIG)/libend2end_certs.a
endif




endif

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(LIBEND2END_CERTS_OBJS:.o=.dep)
endif
endif


LIBBAD_CLIENT_TEST_SRC = \
    test/core/bad_client/bad_client.c \


LIBBAD_CLIENT_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(LIBBAD_CLIENT_TEST_SRC))))


ifeq ($(NO_SECURE),true)

# You can't build secure libraries if you don't have OpenSSL.

$(LIBDIR)/$(CONFIG)/libbad_client_test.a: openssl_dep_error


else


$(LIBDIR)/$(CONFIG)/libbad_client_test.a: $(ZLIB_DEP) $(OPENSSL_DEP) $(LIBBAD_CLIENT_TEST_OBJS)
	$(E) "[AR]      Creating $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) rm -f $(LIBDIR)/$(CONFIG)/libbad_client_test.a
	$(Q) $(AR) rcs $(LIBDIR)/$(CONFIG)/libbad_client_test.a $(LIBBAD_CLIENT_TEST_OBJS)
ifeq ($(SYSTEM),Darwin)
	$(Q) ranlib $(LIBDIR)/$(CONFIG)/libbad_client_test.a
endif




endif

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(LIBBAD_CLIENT_TEST_OBJS:.o=.dep)
endif
endif


LIBBAD_SSL_TEST_SERVER_SRC = \
    test/core/bad_ssl/server.c \


LIBBAD_SSL_TEST_SERVER_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(LIBBAD_SSL_TEST_SERVER_SRC))))


ifeq ($(NO_SECURE),true)

# You can't build secure libraries if you don't have OpenSSL.

$(LIBDIR)/$(CONFIG)/libbad_ssl_test_server.a: openssl_dep_error


else


$(LIBDIR)/$(CONFIG)/libbad_ssl_test_server.a: $(ZLIB_DEP) $(OPENSSL_DEP) $(LIBBAD_SSL_TEST_SERVER_OBJS)
	$(E) "[AR]      Creating $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) rm -f $(LIBDIR)/$(CONFIG)/libbad_ssl_test_server.a
	$(Q) $(AR) rcs $(LIBDIR)/$(CONFIG)/libbad_ssl_test_server.a $(LIBBAD_SSL_TEST_SERVER_OBJS)
ifeq ($(SYSTEM),Darwin)
	$(Q) ranlib $(LIBDIR)/$(CONFIG)/libbad_ssl_test_server.a
endif




endif

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(LIBBAD_SSL_TEST_SERVER_OBJS:.o=.dep)
endif
endif



# All of the test targets, and protoc plugins


ALGORITHM_TEST_SRC = \
    test/core/compression/algorithm_test.c \

ALGORITHM_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(ALGORITHM_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/algorithm_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/algorithm_test: $(ALGORITHM_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(ALGORITHM_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/algorithm_test

endif

$(OBJDIR)/$(CONFIG)/test/core/compression/algorithm_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_algorithm_test: $(ALGORITHM_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(ALGORITHM_TEST_OBJS:.o=.dep)
endif
endif


ALLOC_TEST_SRC = \
    test/core/support/alloc_test.c \

ALLOC_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(ALLOC_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/alloc_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/alloc_test: $(ALLOC_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(ALLOC_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/alloc_test

endif

$(OBJDIR)/$(CONFIG)/test/core/support/alloc_test.o:  $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_alloc_test: $(ALLOC_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(ALLOC_TEST_OBJS:.o=.dep)
endif
endif


ALPN_TEST_SRC = \
    test/core/transport/chttp2/alpn_test.c \

ALPN_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(ALPN_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/alpn_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/alpn_test: $(ALPN_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(ALPN_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/alpn_test

endif

$(OBJDIR)/$(CONFIG)/test/core/transport/chttp2/alpn_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_alpn_test: $(ALPN_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(ALPN_TEST_OBJS:.o=.dep)
endif
endif


BIN_ENCODER_TEST_SRC = \
    test/core/transport/chttp2/bin_encoder_test.c \

BIN_ENCODER_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(BIN_ENCODER_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/bin_encoder_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/bin_encoder_test: $(BIN_ENCODER_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(BIN_ENCODER_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/bin_encoder_test

endif

$(OBJDIR)/$(CONFIG)/test/core/transport/chttp2/bin_encoder_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_bin_encoder_test: $(BIN_ENCODER_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(BIN_ENCODER_TEST_OBJS:.o=.dep)
endif
endif


CHANNEL_CREATE_TEST_SRC = \
    test/core/surface/channel_create_test.c \

CHANNEL_CREATE_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(CHANNEL_CREATE_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/channel_create_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/channel_create_test: $(CHANNEL_CREATE_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(CHANNEL_CREATE_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/channel_create_test

endif

$(OBJDIR)/$(CONFIG)/test/core/surface/channel_create_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_channel_create_test: $(CHANNEL_CREATE_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(CHANNEL_CREATE_TEST_OBJS:.o=.dep)
endif
endif


CHTTP2_HPACK_ENCODER_TEST_SRC = \
    test/core/transport/chttp2/hpack_encoder_test.c \

CHTTP2_HPACK_ENCODER_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(CHTTP2_HPACK_ENCODER_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/chttp2_hpack_encoder_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/chttp2_hpack_encoder_test: $(CHTTP2_HPACK_ENCODER_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(CHTTP2_HPACK_ENCODER_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/chttp2_hpack_encoder_test

endif

$(OBJDIR)/$(CONFIG)/test/core/transport/chttp2/hpack_encoder_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_chttp2_hpack_encoder_test: $(CHTTP2_HPACK_ENCODER_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(CHTTP2_HPACK_ENCODER_TEST_OBJS:.o=.dep)
endif
endif


CHTTP2_STATUS_CONVERSION_TEST_SRC = \
    test/core/transport/chttp2/status_conversion_test.c \

CHTTP2_STATUS_CONVERSION_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(CHTTP2_STATUS_CONVERSION_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/chttp2_status_conversion_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/chttp2_status_conversion_test: $(CHTTP2_STATUS_CONVERSION_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(CHTTP2_STATUS_CONVERSION_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/chttp2_status_conversion_test

endif

$(OBJDIR)/$(CONFIG)/test/core/transport/chttp2/status_conversion_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_chttp2_status_conversion_test: $(CHTTP2_STATUS_CONVERSION_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(CHTTP2_STATUS_CONVERSION_TEST_OBJS:.o=.dep)
endif
endif


CHTTP2_STREAM_MAP_TEST_SRC = \
    test/core/transport/chttp2/stream_map_test.c \

CHTTP2_STREAM_MAP_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(CHTTP2_STREAM_MAP_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/chttp2_stream_map_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/chttp2_stream_map_test: $(CHTTP2_STREAM_MAP_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(CHTTP2_STREAM_MAP_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/chttp2_stream_map_test

endif

$(OBJDIR)/$(CONFIG)/test/core/transport/chttp2/stream_map_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_chttp2_stream_map_test: $(CHTTP2_STREAM_MAP_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(CHTTP2_STREAM_MAP_TEST_OBJS:.o=.dep)
endif
endif


CHTTP2_VARINT_TEST_SRC = \
    test/core/transport/chttp2/varint_test.c \

CHTTP2_VARINT_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(CHTTP2_VARINT_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/chttp2_varint_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/chttp2_varint_test: $(CHTTP2_VARINT_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(CHTTP2_VARINT_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/chttp2_varint_test

endif

$(OBJDIR)/$(CONFIG)/test/core/transport/chttp2/varint_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_chttp2_varint_test: $(CHTTP2_VARINT_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(CHTTP2_VARINT_TEST_OBJS:.o=.dep)
endif
endif


COMPRESSION_TEST_SRC = \
    test/core/compression/compression_test.c \

COMPRESSION_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(COMPRESSION_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/compression_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/compression_test: $(COMPRESSION_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(COMPRESSION_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/compression_test

endif

$(OBJDIR)/$(CONFIG)/test/core/compression/compression_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_compression_test: $(COMPRESSION_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(COMPRESSION_TEST_OBJS:.o=.dep)
endif
endif


DNS_RESOLVER_TEST_SRC = \
    test/core/client_config/resolvers/dns_resolver_test.c \

DNS_RESOLVER_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(DNS_RESOLVER_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/dns_resolver_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/dns_resolver_test: $(DNS_RESOLVER_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(DNS_RESOLVER_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/dns_resolver_test

endif

$(OBJDIR)/$(CONFIG)/test/core/client_config/resolvers/dns_resolver_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_dns_resolver_test: $(DNS_RESOLVER_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(DNS_RESOLVER_TEST_OBJS:.o=.dep)
endif
endif


DUALSTACK_SOCKET_TEST_SRC = \
    test/core/end2end/dualstack_socket_test.c \

DUALSTACK_SOCKET_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(DUALSTACK_SOCKET_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/dualstack_socket_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/dualstack_socket_test: $(DUALSTACK_SOCKET_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(DUALSTACK_SOCKET_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/dualstack_socket_test

endif

$(OBJDIR)/$(CONFIG)/test/core/end2end/dualstack_socket_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_dualstack_socket_test: $(DUALSTACK_SOCKET_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(DUALSTACK_SOCKET_TEST_OBJS:.o=.dep)
endif
endif


ENDPOINT_PAIR_TEST_SRC = \
    test/core/iomgr/endpoint_pair_test.c \

ENDPOINT_PAIR_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(ENDPOINT_PAIR_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/endpoint_pair_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/endpoint_pair_test: $(ENDPOINT_PAIR_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(ENDPOINT_PAIR_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/endpoint_pair_test

endif

$(OBJDIR)/$(CONFIG)/test/core/iomgr/endpoint_pair_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_endpoint_pair_test: $(ENDPOINT_PAIR_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(ENDPOINT_PAIR_TEST_OBJS:.o=.dep)
endif
endif


FD_CONSERVATION_POSIX_TEST_SRC = \
    test/core/iomgr/fd_conservation_posix_test.c \

FD_CONSERVATION_POSIX_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(FD_CONSERVATION_POSIX_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/fd_conservation_posix_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/fd_conservation_posix_test: $(FD_CONSERVATION_POSIX_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(FD_CONSERVATION_POSIX_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/fd_conservation_posix_test

endif

$(OBJDIR)/$(CONFIG)/test/core/iomgr/fd_conservation_posix_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_fd_conservation_posix_test: $(FD_CONSERVATION_POSIX_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(FD_CONSERVATION_POSIX_TEST_OBJS:.o=.dep)
endif
endif


FD_POSIX_TEST_SRC = \
    test/core/iomgr/fd_posix_test.c \

FD_POSIX_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(FD_POSIX_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/fd_posix_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/fd_posix_test: $(FD_POSIX_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(FD_POSIX_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/fd_posix_test

endif

$(OBJDIR)/$(CONFIG)/test/core/iomgr/fd_posix_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_fd_posix_test: $(FD_POSIX_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(FD_POSIX_TEST_OBJS:.o=.dep)
endif
endif


FLING_CLIENT_SRC = \
    test/core/fling/client.c \

FLING_CLIENT_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(FLING_CLIENT_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/fling_client: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/fling_client: $(FLING_CLIENT_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(FLING_CLIENT_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/fling_client

endif

$(OBJDIR)/$(CONFIG)/test/core/fling/client.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_fling_client: $(FLING_CLIENT_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(FLING_CLIENT_OBJS:.o=.dep)
endif
endif


FLING_SERVER_SRC = \
    test/core/fling/server.c \

FLING_SERVER_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(FLING_SERVER_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/fling_server: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/fling_server: $(FLING_SERVER_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(FLING_SERVER_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/fling_server

endif

$(OBJDIR)/$(CONFIG)/test/core/fling/server.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_fling_server: $(FLING_SERVER_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(FLING_SERVER_OBJS:.o=.dep)
endif
endif


FLING_STREAM_TEST_SRC = \
    test/core/fling/fling_stream_test.c \

FLING_STREAM_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(FLING_STREAM_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/fling_stream_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/fling_stream_test: $(FLING_STREAM_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(FLING_STREAM_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/fling_stream_test

endif

$(OBJDIR)/$(CONFIG)/test/core/fling/fling_stream_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_fling_stream_test: $(FLING_STREAM_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(FLING_STREAM_TEST_OBJS:.o=.dep)
endif
endif


FLING_TEST_SRC = \
    test/core/fling/fling_test.c \

FLING_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(FLING_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/fling_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/fling_test: $(FLING_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(FLING_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/fling_test

endif

$(OBJDIR)/$(CONFIG)/test/core/fling/fling_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_fling_test: $(FLING_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(FLING_TEST_OBJS:.o=.dep)
endif
endif


GEN_HPACK_TABLES_SRC = \
    tools/codegen/core/gen_hpack_tables.c \

GEN_HPACK_TABLES_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(GEN_HPACK_TABLES_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/gen_hpack_tables: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/gen_hpack_tables: $(GEN_HPACK_TABLES_OBJS) $(LIBDIR)/$(CONFIG)/libgpr.a $(LIBDIR)/$(CONFIG)/libgrpc.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(GEN_HPACK_TABLES_OBJS) $(LIBDIR)/$(CONFIG)/libgpr.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/gen_hpack_tables

endif

$(OBJDIR)/$(CONFIG)/tools/codegen/core/gen_hpack_tables.o:  $(LIBDIR)/$(CONFIG)/libgpr.a $(LIBDIR)/$(CONFIG)/libgrpc.a
deps_gen_hpack_tables: $(GEN_HPACK_TABLES_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(GEN_HPACK_TABLES_OBJS:.o=.dep)
endif
endif


GEN_LEGAL_METADATA_CHARACTERS_SRC = \
    tools/codegen/core/gen_legal_metadata_characters.c \

GEN_LEGAL_METADATA_CHARACTERS_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(GEN_LEGAL_METADATA_CHARACTERS_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/gen_legal_metadata_characters: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/gen_legal_metadata_characters: $(GEN_LEGAL_METADATA_CHARACTERS_OBJS)
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(GEN_LEGAL_METADATA_CHARACTERS_OBJS) $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/gen_legal_metadata_characters

endif

$(OBJDIR)/$(CONFIG)/tools/codegen/core/gen_legal_metadata_characters.o: 
deps_gen_legal_metadata_characters: $(GEN_LEGAL_METADATA_CHARACTERS_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(GEN_LEGAL_METADATA_CHARACTERS_OBJS:.o=.dep)
endif
endif


GPR_AVL_TEST_SRC = \
    test/core/support/avl_test.c \

GPR_AVL_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(GPR_AVL_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/gpr_avl_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/gpr_avl_test: $(GPR_AVL_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(GPR_AVL_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/gpr_avl_test

endif

$(OBJDIR)/$(CONFIG)/test/core/support/avl_test.o:  $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_gpr_avl_test: $(GPR_AVL_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(GPR_AVL_TEST_OBJS:.o=.dep)
endif
endif


GPR_CMDLINE_TEST_SRC = \
    test/core/support/cmdline_test.c \

GPR_CMDLINE_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(GPR_CMDLINE_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/gpr_cmdline_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/gpr_cmdline_test: $(GPR_CMDLINE_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(GPR_CMDLINE_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/gpr_cmdline_test

endif

$(OBJDIR)/$(CONFIG)/test/core/support/cmdline_test.o:  $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_gpr_cmdline_test: $(GPR_CMDLINE_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(GPR_CMDLINE_TEST_OBJS:.o=.dep)
endif
endif


GPR_CPU_TEST_SRC = \
    test/core/support/cpu_test.c \

GPR_CPU_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(GPR_CPU_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/gpr_cpu_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/gpr_cpu_test: $(GPR_CPU_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(GPR_CPU_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/gpr_cpu_test

endif

$(OBJDIR)/$(CONFIG)/test/core/support/cpu_test.o:  $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_gpr_cpu_test: $(GPR_CPU_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(GPR_CPU_TEST_OBJS:.o=.dep)
endif
endif


GPR_ENV_TEST_SRC = \
    test/core/support/env_test.c \

GPR_ENV_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(GPR_ENV_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/gpr_env_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/gpr_env_test: $(GPR_ENV_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(GPR_ENV_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/gpr_env_test

endif

$(OBJDIR)/$(CONFIG)/test/core/support/env_test.o:  $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_gpr_env_test: $(GPR_ENV_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(GPR_ENV_TEST_OBJS:.o=.dep)
endif
endif


GPR_FILE_TEST_SRC = \
    test/core/support/file_test.c \

GPR_FILE_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(GPR_FILE_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/gpr_file_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/gpr_file_test: $(GPR_FILE_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(GPR_FILE_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/gpr_file_test

endif

$(OBJDIR)/$(CONFIG)/test/core/support/file_test.o:  $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_gpr_file_test: $(GPR_FILE_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(GPR_FILE_TEST_OBJS:.o=.dep)
endif
endif


GPR_HISTOGRAM_TEST_SRC = \
    test/core/support/histogram_test.c \

GPR_HISTOGRAM_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(GPR_HISTOGRAM_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/gpr_histogram_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/gpr_histogram_test: $(GPR_HISTOGRAM_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(GPR_HISTOGRAM_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/gpr_histogram_test

endif

$(OBJDIR)/$(CONFIG)/test/core/support/histogram_test.o:  $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_gpr_histogram_test: $(GPR_HISTOGRAM_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(GPR_HISTOGRAM_TEST_OBJS:.o=.dep)
endif
endif


GPR_HOST_PORT_TEST_SRC = \
    test/core/support/host_port_test.c \

GPR_HOST_PORT_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(GPR_HOST_PORT_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/gpr_host_port_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/gpr_host_port_test: $(GPR_HOST_PORT_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(GPR_HOST_PORT_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/gpr_host_port_test

endif

$(OBJDIR)/$(CONFIG)/test/core/support/host_port_test.o:  $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_gpr_host_port_test: $(GPR_HOST_PORT_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(GPR_HOST_PORT_TEST_OBJS:.o=.dep)
endif
endif


GPR_LOG_TEST_SRC = \
    test/core/support/log_test.c \

GPR_LOG_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(GPR_LOG_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/gpr_log_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/gpr_log_test: $(GPR_LOG_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(GPR_LOG_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/gpr_log_test

endif

$(OBJDIR)/$(CONFIG)/test/core/support/log_test.o:  $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_gpr_log_test: $(GPR_LOG_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(GPR_LOG_TEST_OBJS:.o=.dep)
endif
endif


GPR_SLICE_BUFFER_TEST_SRC = \
    test/core/support/slice_buffer_test.c \

GPR_SLICE_BUFFER_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(GPR_SLICE_BUFFER_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/gpr_slice_buffer_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/gpr_slice_buffer_test: $(GPR_SLICE_BUFFER_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(GPR_SLICE_BUFFER_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/gpr_slice_buffer_test

endif

$(OBJDIR)/$(CONFIG)/test/core/support/slice_buffer_test.o:  $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_gpr_slice_buffer_test: $(GPR_SLICE_BUFFER_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(GPR_SLICE_BUFFER_TEST_OBJS:.o=.dep)
endif
endif


GPR_SLICE_TEST_SRC = \
    test/core/support/slice_test.c \

GPR_SLICE_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(GPR_SLICE_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/gpr_slice_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/gpr_slice_test: $(GPR_SLICE_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(GPR_SLICE_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/gpr_slice_test

endif

$(OBJDIR)/$(CONFIG)/test/core/support/slice_test.o:  $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_gpr_slice_test: $(GPR_SLICE_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(GPR_SLICE_TEST_OBJS:.o=.dep)
endif
endif


GPR_STACK_LOCKFREE_TEST_SRC = \
    test/core/support/stack_lockfree_test.c \

GPR_STACK_LOCKFREE_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(GPR_STACK_LOCKFREE_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/gpr_stack_lockfree_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/gpr_stack_lockfree_test: $(GPR_STACK_LOCKFREE_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(GPR_STACK_LOCKFREE_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/gpr_stack_lockfree_test

endif

$(OBJDIR)/$(CONFIG)/test/core/support/stack_lockfree_test.o:  $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_gpr_stack_lockfree_test: $(GPR_STACK_LOCKFREE_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(GPR_STACK_LOCKFREE_TEST_OBJS:.o=.dep)
endif
endif


GPR_STRING_TEST_SRC = \
    test/core/support/string_test.c \

GPR_STRING_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(GPR_STRING_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/gpr_string_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/gpr_string_test: $(GPR_STRING_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(GPR_STRING_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/gpr_string_test

endif

$(OBJDIR)/$(CONFIG)/test/core/support/string_test.o:  $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_gpr_string_test: $(GPR_STRING_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(GPR_STRING_TEST_OBJS:.o=.dep)
endif
endif


GPR_SYNC_TEST_SRC = \
    test/core/support/sync_test.c \

GPR_SYNC_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(GPR_SYNC_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/gpr_sync_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/gpr_sync_test: $(GPR_SYNC_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(GPR_SYNC_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/gpr_sync_test

endif

$(OBJDIR)/$(CONFIG)/test/core/support/sync_test.o:  $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_gpr_sync_test: $(GPR_SYNC_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(GPR_SYNC_TEST_OBJS:.o=.dep)
endif
endif


GPR_THD_TEST_SRC = \
    test/core/support/thd_test.c \

GPR_THD_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(GPR_THD_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/gpr_thd_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/gpr_thd_test: $(GPR_THD_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(GPR_THD_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/gpr_thd_test

endif

$(OBJDIR)/$(CONFIG)/test/core/support/thd_test.o:  $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_gpr_thd_test: $(GPR_THD_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(GPR_THD_TEST_OBJS:.o=.dep)
endif
endif


GPR_TIME_TEST_SRC = \
    test/core/support/time_test.c \

GPR_TIME_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(GPR_TIME_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/gpr_time_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/gpr_time_test: $(GPR_TIME_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(GPR_TIME_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/gpr_time_test

endif

$(OBJDIR)/$(CONFIG)/test/core/support/time_test.o:  $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_gpr_time_test: $(GPR_TIME_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(GPR_TIME_TEST_OBJS:.o=.dep)
endif
endif


GPR_TLS_TEST_SRC = \
    test/core/support/tls_test.c \

GPR_TLS_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(GPR_TLS_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/gpr_tls_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/gpr_tls_test: $(GPR_TLS_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(GPR_TLS_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/gpr_tls_test

endif

$(OBJDIR)/$(CONFIG)/test/core/support/tls_test.o:  $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_gpr_tls_test: $(GPR_TLS_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(GPR_TLS_TEST_OBJS:.o=.dep)
endif
endif


GPR_USEFUL_TEST_SRC = \
    test/core/support/useful_test.c \

GPR_USEFUL_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(GPR_USEFUL_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/gpr_useful_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/gpr_useful_test: $(GPR_USEFUL_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(GPR_USEFUL_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/gpr_useful_test

endif

$(OBJDIR)/$(CONFIG)/test/core/support/useful_test.o:  $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_gpr_useful_test: $(GPR_USEFUL_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(GPR_USEFUL_TEST_OBJS:.o=.dep)
endif
endif


GRPC_AUTH_CONTEXT_TEST_SRC = \
    test/core/security/auth_context_test.c \

GRPC_AUTH_CONTEXT_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(GRPC_AUTH_CONTEXT_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/grpc_auth_context_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/grpc_auth_context_test: $(GRPC_AUTH_CONTEXT_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(GRPC_AUTH_CONTEXT_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/grpc_auth_context_test

endif

$(OBJDIR)/$(CONFIG)/test/core/security/auth_context_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_grpc_auth_context_test: $(GRPC_AUTH_CONTEXT_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(GRPC_AUTH_CONTEXT_TEST_OBJS:.o=.dep)
endif
endif


GRPC_BASE64_TEST_SRC = \
    test/core/security/base64_test.c \

GRPC_BASE64_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(GRPC_BASE64_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/grpc_base64_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/grpc_base64_test: $(GRPC_BASE64_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(GRPC_BASE64_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/grpc_base64_test

endif

$(OBJDIR)/$(CONFIG)/test/core/security/base64_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_grpc_base64_test: $(GRPC_BASE64_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(GRPC_BASE64_TEST_OBJS:.o=.dep)
endif
endif


GRPC_BYTE_BUFFER_READER_TEST_SRC = \
    test/core/surface/byte_buffer_reader_test.c \

GRPC_BYTE_BUFFER_READER_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(GRPC_BYTE_BUFFER_READER_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/grpc_byte_buffer_reader_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/grpc_byte_buffer_reader_test: $(GRPC_BYTE_BUFFER_READER_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(GRPC_BYTE_BUFFER_READER_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/grpc_byte_buffer_reader_test

endif

$(OBJDIR)/$(CONFIG)/test/core/surface/byte_buffer_reader_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_grpc_byte_buffer_reader_test: $(GRPC_BYTE_BUFFER_READER_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(GRPC_BYTE_BUFFER_READER_TEST_OBJS:.o=.dep)
endif
endif


GRPC_CHANNEL_ARGS_TEST_SRC = \
    test/core/channel/channel_args_test.c \

GRPC_CHANNEL_ARGS_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(GRPC_CHANNEL_ARGS_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/grpc_channel_args_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/grpc_channel_args_test: $(GRPC_CHANNEL_ARGS_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(GRPC_CHANNEL_ARGS_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/grpc_channel_args_test

endif

$(OBJDIR)/$(CONFIG)/test/core/channel/channel_args_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_grpc_channel_args_test: $(GRPC_CHANNEL_ARGS_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(GRPC_CHANNEL_ARGS_TEST_OBJS:.o=.dep)
endif
endif


GRPC_CHANNEL_STACK_TEST_SRC = \
    test/core/channel/channel_stack_test.c \

GRPC_CHANNEL_STACK_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(GRPC_CHANNEL_STACK_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/grpc_channel_stack_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/grpc_channel_stack_test: $(GRPC_CHANNEL_STACK_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(GRPC_CHANNEL_STACK_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/grpc_channel_stack_test

endif

$(OBJDIR)/$(CONFIG)/test/core/channel/channel_stack_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_grpc_channel_stack_test: $(GRPC_CHANNEL_STACK_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(GRPC_CHANNEL_STACK_TEST_OBJS:.o=.dep)
endif
endif


GRPC_COMPLETION_QUEUE_TEST_SRC = \
    test/core/surface/completion_queue_test.c \

GRPC_COMPLETION_QUEUE_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(GRPC_COMPLETION_QUEUE_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/grpc_completion_queue_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/grpc_completion_queue_test: $(GRPC_COMPLETION_QUEUE_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(GRPC_COMPLETION_QUEUE_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/grpc_completion_queue_test

endif

$(OBJDIR)/$(CONFIG)/test/core/surface/completion_queue_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_grpc_completion_queue_test: $(GRPC_COMPLETION_QUEUE_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(GRPC_COMPLETION_QUEUE_TEST_OBJS:.o=.dep)
endif
endif


GRPC_CREATE_JWT_SRC = \
    test/core/security/create_jwt.c \

GRPC_CREATE_JWT_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(GRPC_CREATE_JWT_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/grpc_create_jwt: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/grpc_create_jwt: $(GRPC_CREATE_JWT_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(GRPC_CREATE_JWT_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/grpc_create_jwt

endif

$(OBJDIR)/$(CONFIG)/test/core/security/create_jwt.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_grpc_create_jwt: $(GRPC_CREATE_JWT_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(GRPC_CREATE_JWT_OBJS:.o=.dep)
endif
endif


GRPC_CREDENTIALS_TEST_SRC = \
    test/core/security/credentials_test.c \

GRPC_CREDENTIALS_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(GRPC_CREDENTIALS_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/grpc_credentials_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/grpc_credentials_test: $(GRPC_CREDENTIALS_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(GRPC_CREDENTIALS_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/grpc_credentials_test

endif

$(OBJDIR)/$(CONFIG)/test/core/security/credentials_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_grpc_credentials_test: $(GRPC_CREDENTIALS_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(GRPC_CREDENTIALS_TEST_OBJS:.o=.dep)
endif
endif


GRPC_FETCH_OAUTH2_SRC = \
    test/core/security/fetch_oauth2.c \

GRPC_FETCH_OAUTH2_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(GRPC_FETCH_OAUTH2_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/grpc_fetch_oauth2: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/grpc_fetch_oauth2: $(GRPC_FETCH_OAUTH2_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(GRPC_FETCH_OAUTH2_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/grpc_fetch_oauth2

endif

$(OBJDIR)/$(CONFIG)/test/core/security/fetch_oauth2.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_grpc_fetch_oauth2: $(GRPC_FETCH_OAUTH2_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(GRPC_FETCH_OAUTH2_OBJS:.o=.dep)
endif
endif


GRPC_INVALID_CHANNEL_ARGS_TEST_SRC = \
    test/core/surface/invalid_channel_args_test.c \

GRPC_INVALID_CHANNEL_ARGS_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(GRPC_INVALID_CHANNEL_ARGS_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/grpc_invalid_channel_args_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/grpc_invalid_channel_args_test: $(GRPC_INVALID_CHANNEL_ARGS_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(GRPC_INVALID_CHANNEL_ARGS_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/grpc_invalid_channel_args_test

endif

$(OBJDIR)/$(CONFIG)/test/core/surface/invalid_channel_args_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_grpc_invalid_channel_args_test: $(GRPC_INVALID_CHANNEL_ARGS_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(GRPC_INVALID_CHANNEL_ARGS_TEST_OBJS:.o=.dep)
endif
endif


GRPC_JSON_TOKEN_TEST_SRC = \
    test/core/security/json_token_test.c \

GRPC_JSON_TOKEN_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(GRPC_JSON_TOKEN_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/grpc_json_token_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/grpc_json_token_test: $(GRPC_JSON_TOKEN_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(GRPC_JSON_TOKEN_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/grpc_json_token_test

endif

$(OBJDIR)/$(CONFIG)/test/core/security/json_token_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_grpc_json_token_test: $(GRPC_JSON_TOKEN_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(GRPC_JSON_TOKEN_TEST_OBJS:.o=.dep)
endif
endif


GRPC_JWT_VERIFIER_TEST_SRC = \
    test/core/security/jwt_verifier_test.c \

GRPC_JWT_VERIFIER_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(GRPC_JWT_VERIFIER_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/grpc_jwt_verifier_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/grpc_jwt_verifier_test: $(GRPC_JWT_VERIFIER_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(GRPC_JWT_VERIFIER_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/grpc_jwt_verifier_test

endif

$(OBJDIR)/$(CONFIG)/test/core/security/jwt_verifier_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_grpc_jwt_verifier_test: $(GRPC_JWT_VERIFIER_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(GRPC_JWT_VERIFIER_TEST_OBJS:.o=.dep)
endif
endif


GRPC_PRINT_GOOGLE_DEFAULT_CREDS_TOKEN_SRC = \
    test/core/security/print_google_default_creds_token.c \

GRPC_PRINT_GOOGLE_DEFAULT_CREDS_TOKEN_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(GRPC_PRINT_GOOGLE_DEFAULT_CREDS_TOKEN_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/grpc_print_google_default_creds_token: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/grpc_print_google_default_creds_token: $(GRPC_PRINT_GOOGLE_DEFAULT_CREDS_TOKEN_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(GRPC_PRINT_GOOGLE_DEFAULT_CREDS_TOKEN_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/grpc_print_google_default_creds_token

endif

$(OBJDIR)/$(CONFIG)/test/core/security/print_google_default_creds_token.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_grpc_print_google_default_creds_token: $(GRPC_PRINT_GOOGLE_DEFAULT_CREDS_TOKEN_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(GRPC_PRINT_GOOGLE_DEFAULT_CREDS_TOKEN_OBJS:.o=.dep)
endif
endif


GRPC_SECURITY_CONNECTOR_TEST_SRC = \
    test/core/security/security_connector_test.c \

GRPC_SECURITY_CONNECTOR_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(GRPC_SECURITY_CONNECTOR_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/grpc_security_connector_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/grpc_security_connector_test: $(GRPC_SECURITY_CONNECTOR_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(GRPC_SECURITY_CONNECTOR_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/grpc_security_connector_test

endif

$(OBJDIR)/$(CONFIG)/test/core/security/security_connector_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_grpc_security_connector_test: $(GRPC_SECURITY_CONNECTOR_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(GRPC_SECURITY_CONNECTOR_TEST_OBJS:.o=.dep)
endif
endif


GRPC_VERIFY_JWT_SRC = \
    test/core/security/verify_jwt.c \

GRPC_VERIFY_JWT_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(GRPC_VERIFY_JWT_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/grpc_verify_jwt: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/grpc_verify_jwt: $(GRPC_VERIFY_JWT_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(GRPC_VERIFY_JWT_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/grpc_verify_jwt

endif

$(OBJDIR)/$(CONFIG)/test/core/security/verify_jwt.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_grpc_verify_jwt: $(GRPC_VERIFY_JWT_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(GRPC_VERIFY_JWT_OBJS:.o=.dep)
endif
endif


HPACK_PARSER_TEST_SRC = \
    test/core/transport/chttp2/hpack_parser_test.c \

HPACK_PARSER_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(HPACK_PARSER_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/hpack_parser_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/hpack_parser_test: $(HPACK_PARSER_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(HPACK_PARSER_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/hpack_parser_test

endif

$(OBJDIR)/$(CONFIG)/test/core/transport/chttp2/hpack_parser_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_hpack_parser_test: $(HPACK_PARSER_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(HPACK_PARSER_TEST_OBJS:.o=.dep)
endif
endif


HPACK_TABLE_TEST_SRC = \
    test/core/transport/chttp2/hpack_table_test.c \

HPACK_TABLE_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(HPACK_TABLE_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/hpack_table_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/hpack_table_test: $(HPACK_TABLE_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(HPACK_TABLE_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/hpack_table_test

endif

$(OBJDIR)/$(CONFIG)/test/core/transport/chttp2/hpack_table_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_hpack_table_test: $(HPACK_TABLE_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(HPACK_TABLE_TEST_OBJS:.o=.dep)
endif
endif


HTTPCLI_FORMAT_REQUEST_TEST_SRC = \
    test/core/httpcli/format_request_test.c \

HTTPCLI_FORMAT_REQUEST_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(HTTPCLI_FORMAT_REQUEST_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/httpcli_format_request_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/httpcli_format_request_test: $(HTTPCLI_FORMAT_REQUEST_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(HTTPCLI_FORMAT_REQUEST_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/httpcli_format_request_test

endif

$(OBJDIR)/$(CONFIG)/test/core/httpcli/format_request_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_httpcli_format_request_test: $(HTTPCLI_FORMAT_REQUEST_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(HTTPCLI_FORMAT_REQUEST_TEST_OBJS:.o=.dep)
endif
endif


HTTPCLI_PARSER_TEST_SRC = \
    test/core/httpcli/parser_test.c \

HTTPCLI_PARSER_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(HTTPCLI_PARSER_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/httpcli_parser_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/httpcli_parser_test: $(HTTPCLI_PARSER_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(HTTPCLI_PARSER_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/httpcli_parser_test

endif

$(OBJDIR)/$(CONFIG)/test/core/httpcli/parser_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_httpcli_parser_test: $(HTTPCLI_PARSER_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(HTTPCLI_PARSER_TEST_OBJS:.o=.dep)
endif
endif


HTTPCLI_TEST_SRC = \
    test/core/httpcli/httpcli_test.c \

HTTPCLI_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(HTTPCLI_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/httpcli_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/httpcli_test: $(HTTPCLI_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(HTTPCLI_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/httpcli_test

endif

$(OBJDIR)/$(CONFIG)/test/core/httpcli/httpcli_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_httpcli_test: $(HTTPCLI_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(HTTPCLI_TEST_OBJS:.o=.dep)
endif
endif


HTTPSCLI_TEST_SRC = \
    test/core/httpcli/httpscli_test.c \

HTTPSCLI_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(HTTPSCLI_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/httpscli_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/httpscli_test: $(HTTPSCLI_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(HTTPSCLI_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/httpscli_test

endif

$(OBJDIR)/$(CONFIG)/test/core/httpcli/httpscli_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_httpscli_test: $(HTTPSCLI_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(HTTPSCLI_TEST_OBJS:.o=.dep)
endif
endif


INIT_TEST_SRC = \
    test/core/surface/init_test.c \

INIT_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(INIT_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/init_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/init_test: $(INIT_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(INIT_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/init_test

endif

$(OBJDIR)/$(CONFIG)/test/core/surface/init_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_init_test: $(INIT_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(INIT_TEST_OBJS:.o=.dep)
endif
endif


INVALID_CALL_ARGUMENT_TEST_SRC = \
    test/core/end2end/invalid_call_argument_test.c \

INVALID_CALL_ARGUMENT_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(INVALID_CALL_ARGUMENT_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/invalid_call_argument_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/invalid_call_argument_test: $(INVALID_CALL_ARGUMENT_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(INVALID_CALL_ARGUMENT_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/invalid_call_argument_test

endif

$(OBJDIR)/$(CONFIG)/test/core/end2end/invalid_call_argument_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_invalid_call_argument_test: $(INVALID_CALL_ARGUMENT_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(INVALID_CALL_ARGUMENT_TEST_OBJS:.o=.dep)
endif
endif


JSON_REWRITE_SRC = \
    test/core/json/json_rewrite.c \

JSON_REWRITE_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(JSON_REWRITE_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/json_rewrite: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/json_rewrite: $(JSON_REWRITE_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(JSON_REWRITE_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/json_rewrite

endif

$(OBJDIR)/$(CONFIG)/test/core/json/json_rewrite.o:  $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_json_rewrite: $(JSON_REWRITE_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(JSON_REWRITE_OBJS:.o=.dep)
endif
endif


JSON_REWRITE_TEST_SRC = \
    test/core/json/json_rewrite_test.c \

JSON_REWRITE_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(JSON_REWRITE_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/json_rewrite_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/json_rewrite_test: $(JSON_REWRITE_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(JSON_REWRITE_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/json_rewrite_test

endif

$(OBJDIR)/$(CONFIG)/test/core/json/json_rewrite_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_json_rewrite_test: $(JSON_REWRITE_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(JSON_REWRITE_TEST_OBJS:.o=.dep)
endif
endif


JSON_STREAM_ERROR_TEST_SRC = \
    test/core/json/json_stream_error_test.c \

JSON_STREAM_ERROR_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(JSON_STREAM_ERROR_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/json_stream_error_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/json_stream_error_test: $(JSON_STREAM_ERROR_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(JSON_STREAM_ERROR_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/json_stream_error_test

endif

$(OBJDIR)/$(CONFIG)/test/core/json/json_stream_error_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_json_stream_error_test: $(JSON_STREAM_ERROR_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(JSON_STREAM_ERROR_TEST_OBJS:.o=.dep)
endif
endif


JSON_TEST_SRC = \
    test/core/json/json_test.c \

JSON_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(JSON_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/json_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/json_test: $(JSON_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(JSON_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/json_test

endif

$(OBJDIR)/$(CONFIG)/test/core/json/json_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_json_test: $(JSON_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(JSON_TEST_OBJS:.o=.dep)
endif
endif


LAME_CLIENT_TEST_SRC = \
    test/core/surface/lame_client_test.c \

LAME_CLIENT_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(LAME_CLIENT_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/lame_client_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/lame_client_test: $(LAME_CLIENT_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(LAME_CLIENT_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/lame_client_test

endif

$(OBJDIR)/$(CONFIG)/test/core/surface/lame_client_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_lame_client_test: $(LAME_CLIENT_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(LAME_CLIENT_TEST_OBJS:.o=.dep)
endif
endif


LB_POLICIES_TEST_SRC = \
    test/core/client_config/lb_policies_test.c \

LB_POLICIES_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(LB_POLICIES_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/lb_policies_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/lb_policies_test: $(LB_POLICIES_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(LB_POLICIES_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/lb_policies_test

endif

$(OBJDIR)/$(CONFIG)/test/core/client_config/lb_policies_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_lb_policies_test: $(LB_POLICIES_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(LB_POLICIES_TEST_OBJS:.o=.dep)
endif
endif


LOW_LEVEL_PING_PONG_BENCHMARK_SRC = \
    test/core/network_benchmarks/low_level_ping_pong.c \

LOW_LEVEL_PING_PONG_BENCHMARK_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(LOW_LEVEL_PING_PONG_BENCHMARK_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/low_level_ping_pong_benchmark: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/low_level_ping_pong_benchmark: $(LOW_LEVEL_PING_PONG_BENCHMARK_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(LOW_LEVEL_PING_PONG_BENCHMARK_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/low_level_ping_pong_benchmark

endif

$(OBJDIR)/$(CONFIG)/test/core/network_benchmarks/low_level_ping_pong.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_low_level_ping_pong_benchmark: $(LOW_LEVEL_PING_PONG_BENCHMARK_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(LOW_LEVEL_PING_PONG_BENCHMARK_OBJS:.o=.dep)
endif
endif


MESSAGE_COMPRESS_TEST_SRC = \
    test/core/compression/message_compress_test.c \

MESSAGE_COMPRESS_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(MESSAGE_COMPRESS_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/message_compress_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/message_compress_test: $(MESSAGE_COMPRESS_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(MESSAGE_COMPRESS_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/message_compress_test

endif

$(OBJDIR)/$(CONFIG)/test/core/compression/message_compress_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_message_compress_test: $(MESSAGE_COMPRESS_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(MESSAGE_COMPRESS_TEST_OBJS:.o=.dep)
endif
endif


MULTIPLE_SERVER_QUEUES_TEST_SRC = \
    test/core/end2end/multiple_server_queues_test.c \

MULTIPLE_SERVER_QUEUES_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(MULTIPLE_SERVER_QUEUES_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/multiple_server_queues_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/multiple_server_queues_test: $(MULTIPLE_SERVER_QUEUES_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(MULTIPLE_SERVER_QUEUES_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/multiple_server_queues_test

endif

$(OBJDIR)/$(CONFIG)/test/core/end2end/multiple_server_queues_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_multiple_server_queues_test: $(MULTIPLE_SERVER_QUEUES_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(MULTIPLE_SERVER_QUEUES_TEST_OBJS:.o=.dep)
endif
endif


MURMUR_HASH_TEST_SRC = \
    test/core/support/murmur_hash_test.c \

MURMUR_HASH_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(MURMUR_HASH_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/murmur_hash_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/murmur_hash_test: $(MURMUR_HASH_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(MURMUR_HASH_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/murmur_hash_test

endif

$(OBJDIR)/$(CONFIG)/test/core/support/murmur_hash_test.o:  $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_murmur_hash_test: $(MURMUR_HASH_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(MURMUR_HASH_TEST_OBJS:.o=.dep)
endif
endif


NO_SERVER_TEST_SRC = \
    test/core/end2end/no_server_test.c \

NO_SERVER_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(NO_SERVER_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/no_server_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/no_server_test: $(NO_SERVER_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(NO_SERVER_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/no_server_test

endif

$(OBJDIR)/$(CONFIG)/test/core/end2end/no_server_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_no_server_test: $(NO_SERVER_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(NO_SERVER_TEST_OBJS:.o=.dep)
endif
endif


RESOLVE_ADDRESS_TEST_SRC = \
    test/core/iomgr/resolve_address_test.c \

RESOLVE_ADDRESS_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(RESOLVE_ADDRESS_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/resolve_address_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/resolve_address_test: $(RESOLVE_ADDRESS_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(RESOLVE_ADDRESS_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/resolve_address_test

endif

$(OBJDIR)/$(CONFIG)/test/core/iomgr/resolve_address_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_resolve_address_test: $(RESOLVE_ADDRESS_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(RESOLVE_ADDRESS_TEST_OBJS:.o=.dep)
endif
endif


SECURE_CHANNEL_CREATE_TEST_SRC = \
    test/core/surface/secure_channel_create_test.c \

SECURE_CHANNEL_CREATE_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(SECURE_CHANNEL_CREATE_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/secure_channel_create_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/secure_channel_create_test: $(SECURE_CHANNEL_CREATE_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(SECURE_CHANNEL_CREATE_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/secure_channel_create_test

endif

$(OBJDIR)/$(CONFIG)/test/core/surface/secure_channel_create_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_secure_channel_create_test: $(SECURE_CHANNEL_CREATE_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(SECURE_CHANNEL_CREATE_TEST_OBJS:.o=.dep)
endif
endif


SECURE_ENDPOINT_TEST_SRC = \
    test/core/security/secure_endpoint_test.c \

SECURE_ENDPOINT_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(SECURE_ENDPOINT_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/secure_endpoint_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/secure_endpoint_test: $(SECURE_ENDPOINT_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(SECURE_ENDPOINT_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/secure_endpoint_test

endif

$(OBJDIR)/$(CONFIG)/test/core/security/secure_endpoint_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_secure_endpoint_test: $(SECURE_ENDPOINT_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(SECURE_ENDPOINT_TEST_OBJS:.o=.dep)
endif
endif


SERVER_CHTTP2_TEST_SRC = \
    test/core/surface/server_chttp2_test.c \

SERVER_CHTTP2_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(SERVER_CHTTP2_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/server_chttp2_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/server_chttp2_test: $(SERVER_CHTTP2_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(SERVER_CHTTP2_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/server_chttp2_test

endif

$(OBJDIR)/$(CONFIG)/test/core/surface/server_chttp2_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_server_chttp2_test: $(SERVER_CHTTP2_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(SERVER_CHTTP2_TEST_OBJS:.o=.dep)
endif
endif


SERVER_TEST_SRC = \
    test/core/surface/server_test.c \

SERVER_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(SERVER_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/server_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/server_test: $(SERVER_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(SERVER_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/server_test

endif

$(OBJDIR)/$(CONFIG)/test/core/surface/server_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_server_test: $(SERVER_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(SERVER_TEST_OBJS:.o=.dep)
endif
endif


SET_INITIAL_CONNECT_STRING_TEST_SRC = \
    test/core/client_config/set_initial_connect_string_test.c \

SET_INITIAL_CONNECT_STRING_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(SET_INITIAL_CONNECT_STRING_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/set_initial_connect_string_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/set_initial_connect_string_test: $(SET_INITIAL_CONNECT_STRING_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libtest_tcp_server.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(SET_INITIAL_CONNECT_STRING_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libtest_tcp_server.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/set_initial_connect_string_test

endif

$(OBJDIR)/$(CONFIG)/test/core/client_config/set_initial_connect_string_test.o:  $(LIBDIR)/$(CONFIG)/libtest_tcp_server.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_set_initial_connect_string_test: $(SET_INITIAL_CONNECT_STRING_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(SET_INITIAL_CONNECT_STRING_TEST_OBJS:.o=.dep)
endif
endif


SOCKADDR_RESOLVER_TEST_SRC = \
    test/core/client_config/resolvers/sockaddr_resolver_test.c \

SOCKADDR_RESOLVER_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(SOCKADDR_RESOLVER_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/sockaddr_resolver_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/sockaddr_resolver_test: $(SOCKADDR_RESOLVER_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(SOCKADDR_RESOLVER_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/sockaddr_resolver_test

endif

$(OBJDIR)/$(CONFIG)/test/core/client_config/resolvers/sockaddr_resolver_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_sockaddr_resolver_test: $(SOCKADDR_RESOLVER_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(SOCKADDR_RESOLVER_TEST_OBJS:.o=.dep)
endif
endif


SOCKADDR_UTILS_TEST_SRC = \
    test/core/iomgr/sockaddr_utils_test.c \

SOCKADDR_UTILS_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(SOCKADDR_UTILS_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/sockaddr_utils_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/sockaddr_utils_test: $(SOCKADDR_UTILS_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(SOCKADDR_UTILS_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/sockaddr_utils_test

endif

$(OBJDIR)/$(CONFIG)/test/core/iomgr/sockaddr_utils_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_sockaddr_utils_test: $(SOCKADDR_UTILS_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(SOCKADDR_UTILS_TEST_OBJS:.o=.dep)
endif
endif


SOCKET_UTILS_TEST_SRC = \
    test/core/iomgr/socket_utils_test.c \

SOCKET_UTILS_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(SOCKET_UTILS_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/socket_utils_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/socket_utils_test: $(SOCKET_UTILS_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(SOCKET_UTILS_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/socket_utils_test

endif

$(OBJDIR)/$(CONFIG)/test/core/iomgr/socket_utils_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_socket_utils_test: $(SOCKET_UTILS_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(SOCKET_UTILS_TEST_OBJS:.o=.dep)
endif
endif


TCP_CLIENT_POSIX_TEST_SRC = \
    test/core/iomgr/tcp_client_posix_test.c \

TCP_CLIENT_POSIX_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(TCP_CLIENT_POSIX_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/tcp_client_posix_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/tcp_client_posix_test: $(TCP_CLIENT_POSIX_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(TCP_CLIENT_POSIX_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/tcp_client_posix_test

endif

$(OBJDIR)/$(CONFIG)/test/core/iomgr/tcp_client_posix_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_tcp_client_posix_test: $(TCP_CLIENT_POSIX_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(TCP_CLIENT_POSIX_TEST_OBJS:.o=.dep)
endif
endif


TCP_POSIX_TEST_SRC = \
    test/core/iomgr/tcp_posix_test.c \

TCP_POSIX_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(TCP_POSIX_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/tcp_posix_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/tcp_posix_test: $(TCP_POSIX_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(TCP_POSIX_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/tcp_posix_test

endif

$(OBJDIR)/$(CONFIG)/test/core/iomgr/tcp_posix_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_tcp_posix_test: $(TCP_POSIX_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(TCP_POSIX_TEST_OBJS:.o=.dep)
endif
endif


TCP_SERVER_POSIX_TEST_SRC = \
    test/core/iomgr/tcp_server_posix_test.c \

TCP_SERVER_POSIX_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(TCP_SERVER_POSIX_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/tcp_server_posix_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/tcp_server_posix_test: $(TCP_SERVER_POSIX_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(TCP_SERVER_POSIX_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/tcp_server_posix_test

endif

$(OBJDIR)/$(CONFIG)/test/core/iomgr/tcp_server_posix_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_tcp_server_posix_test: $(TCP_SERVER_POSIX_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(TCP_SERVER_POSIX_TEST_OBJS:.o=.dep)
endif
endif


TIME_AVERAGED_STATS_TEST_SRC = \
    test/core/iomgr/time_averaged_stats_test.c \

TIME_AVERAGED_STATS_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(TIME_AVERAGED_STATS_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/time_averaged_stats_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/time_averaged_stats_test: $(TIME_AVERAGED_STATS_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(TIME_AVERAGED_STATS_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/time_averaged_stats_test

endif

$(OBJDIR)/$(CONFIG)/test/core/iomgr/time_averaged_stats_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_time_averaged_stats_test: $(TIME_AVERAGED_STATS_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(TIME_AVERAGED_STATS_TEST_OBJS:.o=.dep)
endif
endif


TIMEOUT_ENCODING_TEST_SRC = \
    test/core/transport/chttp2/timeout_encoding_test.c \

TIMEOUT_ENCODING_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(TIMEOUT_ENCODING_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/timeout_encoding_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/timeout_encoding_test: $(TIMEOUT_ENCODING_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(TIMEOUT_ENCODING_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/timeout_encoding_test

endif

$(OBJDIR)/$(CONFIG)/test/core/transport/chttp2/timeout_encoding_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_timeout_encoding_test: $(TIMEOUT_ENCODING_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(TIMEOUT_ENCODING_TEST_OBJS:.o=.dep)
endif
endif


TIMER_HEAP_TEST_SRC = \
    test/core/iomgr/timer_heap_test.c \

TIMER_HEAP_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(TIMER_HEAP_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/timer_heap_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/timer_heap_test: $(TIMER_HEAP_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(TIMER_HEAP_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/timer_heap_test

endif

$(OBJDIR)/$(CONFIG)/test/core/iomgr/timer_heap_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_timer_heap_test: $(TIMER_HEAP_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(TIMER_HEAP_TEST_OBJS:.o=.dep)
endif
endif


TIMER_LIST_TEST_SRC = \
    test/core/iomgr/timer_list_test.c \

TIMER_LIST_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(TIMER_LIST_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/timer_list_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/timer_list_test: $(TIMER_LIST_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(TIMER_LIST_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/timer_list_test

endif

$(OBJDIR)/$(CONFIG)/test/core/iomgr/timer_list_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_timer_list_test: $(TIMER_LIST_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(TIMER_LIST_TEST_OBJS:.o=.dep)
endif
endif


TIMERS_TEST_SRC = \
    test/core/profiling/timers_test.c \

TIMERS_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(TIMERS_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/timers_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/timers_test: $(TIMERS_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(TIMERS_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/timers_test

endif

$(OBJDIR)/$(CONFIG)/test/core/profiling/timers_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_timers_test: $(TIMERS_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(TIMERS_TEST_OBJS:.o=.dep)
endif
endif


TRANSPORT_CONNECTIVITY_STATE_TEST_SRC = \
    test/core/transport/connectivity_state_test.c \

TRANSPORT_CONNECTIVITY_STATE_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(TRANSPORT_CONNECTIVITY_STATE_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/transport_connectivity_state_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/transport_connectivity_state_test: $(TRANSPORT_CONNECTIVITY_STATE_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(TRANSPORT_CONNECTIVITY_STATE_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/transport_connectivity_state_test

endif

$(OBJDIR)/$(CONFIG)/test/core/transport/connectivity_state_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_transport_connectivity_state_test: $(TRANSPORT_CONNECTIVITY_STATE_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(TRANSPORT_CONNECTIVITY_STATE_TEST_OBJS:.o=.dep)
endif
endif


TRANSPORT_METADATA_TEST_SRC = \
    test/core/transport/metadata_test.c \

TRANSPORT_METADATA_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(TRANSPORT_METADATA_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/transport_metadata_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/transport_metadata_test: $(TRANSPORT_METADATA_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(TRANSPORT_METADATA_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/transport_metadata_test

endif

$(OBJDIR)/$(CONFIG)/test/core/transport/metadata_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_transport_metadata_test: $(TRANSPORT_METADATA_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(TRANSPORT_METADATA_TEST_OBJS:.o=.dep)
endif
endif


TRANSPORT_SECURITY_TEST_SRC = \
    test/core/tsi/transport_security_test.c \

TRANSPORT_SECURITY_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(TRANSPORT_SECURITY_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/transport_security_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/transport_security_test: $(TRANSPORT_SECURITY_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(TRANSPORT_SECURITY_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/transport_security_test

endif

$(OBJDIR)/$(CONFIG)/test/core/tsi/transport_security_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_transport_security_test: $(TRANSPORT_SECURITY_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(TRANSPORT_SECURITY_TEST_OBJS:.o=.dep)
endif
endif


UDP_SERVER_TEST_SRC = \
    test/core/iomgr/udp_server_test.c \

UDP_SERVER_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(UDP_SERVER_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/udp_server_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/udp_server_test: $(UDP_SERVER_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(UDP_SERVER_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/udp_server_test

endif

$(OBJDIR)/$(CONFIG)/test/core/iomgr/udp_server_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_udp_server_test: $(UDP_SERVER_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(UDP_SERVER_TEST_OBJS:.o=.dep)
endif
endif


URI_PARSER_TEST_SRC = \
    test/core/client_config/uri_parser_test.c \

URI_PARSER_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(URI_PARSER_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/uri_parser_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/uri_parser_test: $(URI_PARSER_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(URI_PARSER_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/uri_parser_test

endif

$(OBJDIR)/$(CONFIG)/test/core/client_config/uri_parser_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_uri_parser_test: $(URI_PARSER_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(URI_PARSER_TEST_OBJS:.o=.dep)
endif
endif


WORKQUEUE_TEST_SRC = \
    test/core/iomgr/workqueue_test.c \

WORKQUEUE_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(WORKQUEUE_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/workqueue_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/workqueue_test: $(WORKQUEUE_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(WORKQUEUE_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/workqueue_test

endif

$(OBJDIR)/$(CONFIG)/test/core/iomgr/workqueue_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_workqueue_test: $(WORKQUEUE_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(WORKQUEUE_TEST_OBJS:.o=.dep)
endif
endif


ASYNC_END2END_TEST_SRC = \
    test/cpp/end2end/async_end2end_test.cc \

ASYNC_END2END_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(ASYNC_END2END_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/async_end2end_test: openssl_dep_error

else




ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/async_end2end_test: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/async_end2end_test: $(PROTOBUF_DEP) $(ASYNC_END2END_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS) $(ASYNC_END2END_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBSXX) $(LDLIBS_PROTOBUF) $(LDLIBS) $(LDLIBS_SECURE) $(GTEST_LIB) -o $(BINDIR)/$(CONFIG)/async_end2end_test

endif

endif

$(OBJDIR)/$(CONFIG)/test/cpp/end2end/async_end2end_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_async_end2end_test: $(ASYNC_END2END_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(ASYNC_END2END_TEST_OBJS:.o=.dep)
endif
endif


ASYNC_STREAMING_PING_PONG_TEST_SRC = \
    test/cpp/qps/async_streaming_ping_pong_test.cc \

ASYNC_STREAMING_PING_PONG_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(ASYNC_STREAMING_PING_PONG_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/async_streaming_ping_pong_test: openssl_dep_error

else




ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/async_streaming_ping_pong_test: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/async_streaming_ping_pong_test: $(PROTOBUF_DEP) $(ASYNC_STREAMING_PING_PONG_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libqps.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS) $(ASYNC_STREAMING_PING_PONG_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libqps.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBSXX) $(LDLIBS_PROTOBUF) $(LDLIBS) $(LDLIBS_SECURE) $(GTEST_LIB) -o $(BINDIR)/$(CONFIG)/async_streaming_ping_pong_test

endif

endif

$(OBJDIR)/$(CONFIG)/test/cpp/qps/async_streaming_ping_pong_test.o:  $(LIBDIR)/$(CONFIG)/libqps.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_async_streaming_ping_pong_test: $(ASYNC_STREAMING_PING_PONG_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(ASYNC_STREAMING_PING_PONG_TEST_OBJS:.o=.dep)
endif
endif


ASYNC_UNARY_PING_PONG_TEST_SRC = \
    test/cpp/qps/async_unary_ping_pong_test.cc \

ASYNC_UNARY_PING_PONG_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(ASYNC_UNARY_PING_PONG_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/async_unary_ping_pong_test: openssl_dep_error

else




ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/async_unary_ping_pong_test: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/async_unary_ping_pong_test: $(PROTOBUF_DEP) $(ASYNC_UNARY_PING_PONG_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libqps.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS) $(ASYNC_UNARY_PING_PONG_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libqps.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBSXX) $(LDLIBS_PROTOBUF) $(LDLIBS) $(LDLIBS_SECURE) $(GTEST_LIB) -o $(BINDIR)/$(CONFIG)/async_unary_ping_pong_test

endif

endif

$(OBJDIR)/$(CONFIG)/test/cpp/qps/async_unary_ping_pong_test.o:  $(LIBDIR)/$(CONFIG)/libqps.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_async_unary_ping_pong_test: $(ASYNC_UNARY_PING_PONG_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(ASYNC_UNARY_PING_PONG_TEST_OBJS:.o=.dep)
endif
endif


AUTH_PROPERTY_ITERATOR_TEST_SRC = \
    test/cpp/common/auth_property_iterator_test.cc \

AUTH_PROPERTY_ITERATOR_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(AUTH_PROPERTY_ITERATOR_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/auth_property_iterator_test: openssl_dep_error

else




ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/auth_property_iterator_test: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/auth_property_iterator_test: $(PROTOBUF_DEP) $(AUTH_PROPERTY_ITERATOR_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS) $(AUTH_PROPERTY_ITERATOR_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBSXX) $(LDLIBS_PROTOBUF) $(LDLIBS) $(LDLIBS_SECURE) $(GTEST_LIB) -o $(BINDIR)/$(CONFIG)/auth_property_iterator_test

endif

endif

$(OBJDIR)/$(CONFIG)/test/cpp/common/auth_property_iterator_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_auth_property_iterator_test: $(AUTH_PROPERTY_ITERATOR_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(AUTH_PROPERTY_ITERATOR_TEST_OBJS:.o=.dep)
endif
endif


CHANNEL_ARGUMENTS_TEST_SRC = \
    test/cpp/common/channel_arguments_test.cc \

CHANNEL_ARGUMENTS_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(CHANNEL_ARGUMENTS_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/channel_arguments_test: openssl_dep_error

else




ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/channel_arguments_test: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/channel_arguments_test: $(PROTOBUF_DEP) $(CHANNEL_ARGUMENTS_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS) $(CHANNEL_ARGUMENTS_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBSXX) $(LDLIBS_PROTOBUF) $(LDLIBS) $(LDLIBS_SECURE) $(GTEST_LIB) -o $(BINDIR)/$(CONFIG)/channel_arguments_test

endif

endif

$(OBJDIR)/$(CONFIG)/test/cpp/common/channel_arguments_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_channel_arguments_test: $(CHANNEL_ARGUMENTS_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(CHANNEL_ARGUMENTS_TEST_OBJS:.o=.dep)
endif
endif


CLI_CALL_TEST_SRC = \
    test/cpp/util/cli_call_test.cc \

CLI_CALL_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(CLI_CALL_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/cli_call_test: openssl_dep_error

else




ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/cli_call_test: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/cli_call_test: $(PROTOBUF_DEP) $(CLI_CALL_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS) $(CLI_CALL_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBSXX) $(LDLIBS_PROTOBUF) $(LDLIBS) $(LDLIBS_SECURE) $(GTEST_LIB) -o $(BINDIR)/$(CONFIG)/cli_call_test

endif

endif

$(OBJDIR)/$(CONFIG)/test/cpp/util/cli_call_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_cli_call_test: $(CLI_CALL_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(CLI_CALL_TEST_OBJS:.o=.dep)
endif
endif


CLIENT_CRASH_TEST_SRC = \
    test/cpp/end2end/client_crash_test.cc \

CLIENT_CRASH_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(CLIENT_CRASH_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/client_crash_test: openssl_dep_error

else




ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/client_crash_test: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/client_crash_test: $(PROTOBUF_DEP) $(CLIENT_CRASH_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS) $(CLIENT_CRASH_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBSXX) $(LDLIBS_PROTOBUF) $(LDLIBS) $(LDLIBS_SECURE) $(GTEST_LIB) -o $(BINDIR)/$(CONFIG)/client_crash_test

endif

endif

$(OBJDIR)/$(CONFIG)/test/cpp/end2end/client_crash_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_client_crash_test: $(CLIENT_CRASH_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(CLIENT_CRASH_TEST_OBJS:.o=.dep)
endif
endif


CLIENT_CRASH_TEST_SERVER_SRC = \
    test/cpp/end2end/client_crash_test_server.cc \

CLIENT_CRASH_TEST_SERVER_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(CLIENT_CRASH_TEST_SERVER_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/client_crash_test_server: openssl_dep_error

else




ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/client_crash_test_server: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/client_crash_test_server: $(PROTOBUF_DEP) $(CLIENT_CRASH_TEST_SERVER_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS) $(CLIENT_CRASH_TEST_SERVER_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBSXX) $(LDLIBS_PROTOBUF) $(LDLIBS) $(LDLIBS_SECURE) $(GTEST_LIB) -o $(BINDIR)/$(CONFIG)/client_crash_test_server

endif

endif

$(OBJDIR)/$(CONFIG)/test/cpp/end2end/client_crash_test_server.o:  $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_client_crash_test_server: $(CLIENT_CRASH_TEST_SERVER_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(CLIENT_CRASH_TEST_SERVER_OBJS:.o=.dep)
endif
endif


CREDENTIALS_TEST_SRC = \
    test/cpp/client/credentials_test.cc \

CREDENTIALS_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(CREDENTIALS_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/credentials_test: openssl_dep_error

else




ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/credentials_test: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/credentials_test: $(PROTOBUF_DEP) $(CREDENTIALS_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS) $(CREDENTIALS_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBSXX) $(LDLIBS_PROTOBUF) $(LDLIBS) $(LDLIBS_SECURE) $(GTEST_LIB) -o $(BINDIR)/$(CONFIG)/credentials_test

endif

endif

$(OBJDIR)/$(CONFIG)/test/cpp/client/credentials_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_credentials_test: $(CREDENTIALS_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(CREDENTIALS_TEST_OBJS:.o=.dep)
endif
endif


CXX_BYTE_BUFFER_TEST_SRC = \
    test/cpp/util/byte_buffer_test.cc \

CXX_BYTE_BUFFER_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(CXX_BYTE_BUFFER_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/cxx_byte_buffer_test: openssl_dep_error

else




ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/cxx_byte_buffer_test: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/cxx_byte_buffer_test: $(PROTOBUF_DEP) $(CXX_BYTE_BUFFER_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS) $(CXX_BYTE_BUFFER_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBSXX) $(LDLIBS_PROTOBUF) $(LDLIBS) $(LDLIBS_SECURE) $(GTEST_LIB) -o $(BINDIR)/$(CONFIG)/cxx_byte_buffer_test

endif

endif

$(OBJDIR)/$(CONFIG)/test/cpp/util/byte_buffer_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_cxx_byte_buffer_test: $(CXX_BYTE_BUFFER_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(CXX_BYTE_BUFFER_TEST_OBJS:.o=.dep)
endif
endif


CXX_SLICE_TEST_SRC = \
    test/cpp/util/slice_test.cc \

CXX_SLICE_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(CXX_SLICE_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/cxx_slice_test: openssl_dep_error

else




ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/cxx_slice_test: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/cxx_slice_test: $(PROTOBUF_DEP) $(CXX_SLICE_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS) $(CXX_SLICE_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBSXX) $(LDLIBS_PROTOBUF) $(LDLIBS) $(LDLIBS_SECURE) $(GTEST_LIB) -o $(BINDIR)/$(CONFIG)/cxx_slice_test

endif

endif

$(OBJDIR)/$(CONFIG)/test/cpp/util/slice_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_cxx_slice_test: $(CXX_SLICE_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(CXX_SLICE_TEST_OBJS:.o=.dep)
endif
endif


CXX_STRING_REF_TEST_SRC = \
    test/cpp/util/string_ref_test.cc \

CXX_STRING_REF_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(CXX_STRING_REF_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/cxx_string_ref_test: openssl_dep_error

else




ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/cxx_string_ref_test: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/cxx_string_ref_test: $(PROTOBUF_DEP) $(CXX_STRING_REF_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc++.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS) $(CXX_STRING_REF_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LDLIBSXX) $(LDLIBS_PROTOBUF) $(LDLIBS) $(LDLIBS_SECURE) $(GTEST_LIB) -o $(BINDIR)/$(CONFIG)/cxx_string_ref_test

endif

endif

$(OBJDIR)/$(CONFIG)/test/cpp/util/string_ref_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc++.a
deps_cxx_string_ref_test: $(CXX_STRING_REF_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(CXX_STRING_REF_TEST_OBJS:.o=.dep)
endif
endif


CXX_TIME_TEST_SRC = \
    test/cpp/util/time_test.cc \

CXX_TIME_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(CXX_TIME_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/cxx_time_test: openssl_dep_error

else




ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/cxx_time_test: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/cxx_time_test: $(PROTOBUF_DEP) $(CXX_TIME_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS) $(CXX_TIME_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBSXX) $(LDLIBS_PROTOBUF) $(LDLIBS) $(LDLIBS_SECURE) $(GTEST_LIB) -o $(BINDIR)/$(CONFIG)/cxx_time_test

endif

endif

$(OBJDIR)/$(CONFIG)/test/cpp/util/time_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_cxx_time_test: $(CXX_TIME_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(CXX_TIME_TEST_OBJS:.o=.dep)
endif
endif


END2END_TEST_SRC = \
    test/cpp/end2end/end2end_test.cc \

END2END_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(END2END_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/end2end_test: openssl_dep_error

else




ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/end2end_test: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/end2end_test: $(PROTOBUF_DEP) $(END2END_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS) $(END2END_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBSXX) $(LDLIBS_PROTOBUF) $(LDLIBS) $(LDLIBS_SECURE) $(GTEST_LIB) -o $(BINDIR)/$(CONFIG)/end2end_test

endif

endif

$(OBJDIR)/$(CONFIG)/test/cpp/end2end/end2end_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_end2end_test: $(END2END_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(END2END_TEST_OBJS:.o=.dep)
endif
endif


GENERIC_END2END_TEST_SRC = \
    test/cpp/end2end/generic_end2end_test.cc \

GENERIC_END2END_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(GENERIC_END2END_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/generic_end2end_test: openssl_dep_error

else




ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/generic_end2end_test: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/generic_end2end_test: $(PROTOBUF_DEP) $(GENERIC_END2END_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS) $(GENERIC_END2END_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBSXX) $(LDLIBS_PROTOBUF) $(LDLIBS) $(LDLIBS_SECURE) $(GTEST_LIB) -o $(BINDIR)/$(CONFIG)/generic_end2end_test

endif

endif

$(OBJDIR)/$(CONFIG)/test/cpp/end2end/generic_end2end_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_generic_end2end_test: $(GENERIC_END2END_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(GENERIC_END2END_TEST_OBJS:.o=.dep)
endif
endif


GRPC_CLI_SRC = \
    test/cpp/util/grpc_cli.cc \

GRPC_CLI_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(GRPC_CLI_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/grpc_cli: openssl_dep_error

else




ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/grpc_cli: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/grpc_cli: $(PROTOBUF_DEP) $(GRPC_CLI_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_config.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS) $(GRPC_CLI_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_config.a $(LDLIBSXX) $(LDLIBS_PROTOBUF) $(LDLIBS) $(LDLIBS_SECURE) $(GTEST_LIB) -o $(BINDIR)/$(CONFIG)/grpc_cli

endif

endif

$(OBJDIR)/$(CONFIG)/test/cpp/util/grpc_cli.o:  $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_config.a
deps_grpc_cli: $(GRPC_CLI_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(GRPC_CLI_OBJS:.o=.dep)
endif
endif


GRPC_CPP_PLUGIN_SRC = \
    src/compiler/cpp_plugin.cc \

GRPC_CPP_PLUGIN_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(GRPC_CPP_PLUGIN_SRC))))



ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/grpc_cpp_plugin: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/grpc_cpp_plugin: $(PROTOBUF_DEP) $(GRPC_CPP_PLUGIN_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_plugin_support.a
	$(E) "[HOSTLD]  Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(HOST_LDXX) $(HOST_LDFLAGS) $(GRPC_CPP_PLUGIN_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_plugin_support.a $(HOST_LDLIBSXX) $(HOST_LDLIBS_PROTOC) $(HOST_LDLIBS) $(HOST_LDLIBS_PROTOC) -o $(BINDIR)/$(CONFIG)/grpc_cpp_plugin

endif

$(OBJDIR)/$(CONFIG)/src/compiler/cpp_plugin.o:  $(LIBDIR)/$(CONFIG)/libgrpc_plugin_support.a
deps_grpc_cpp_plugin: $(GRPC_CPP_PLUGIN_OBJS:.o=.dep)

ifneq ($(NO_DEPS),true)
-include $(GRPC_CPP_PLUGIN_OBJS:.o=.dep)
endif


GRPC_CSHARP_PLUGIN_SRC = \
    src/compiler/csharp_plugin.cc \

GRPC_CSHARP_PLUGIN_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(GRPC_CSHARP_PLUGIN_SRC))))



ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/grpc_csharp_plugin: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/grpc_csharp_plugin: $(PROTOBUF_DEP) $(GRPC_CSHARP_PLUGIN_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_plugin_support.a
	$(E) "[HOSTLD]  Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(HOST_LDXX) $(HOST_LDFLAGS) $(GRPC_CSHARP_PLUGIN_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_plugin_support.a $(HOST_LDLIBSXX) $(HOST_LDLIBS_PROTOC) $(HOST_LDLIBS) $(HOST_LDLIBS_PROTOC) -o $(BINDIR)/$(CONFIG)/grpc_csharp_plugin

endif

$(OBJDIR)/$(CONFIG)/src/compiler/csharp_plugin.o:  $(LIBDIR)/$(CONFIG)/libgrpc_plugin_support.a
deps_grpc_csharp_plugin: $(GRPC_CSHARP_PLUGIN_OBJS:.o=.dep)

ifneq ($(NO_DEPS),true)
-include $(GRPC_CSHARP_PLUGIN_OBJS:.o=.dep)
endif


GRPC_OBJECTIVE_C_PLUGIN_SRC = \
    src/compiler/objective_c_plugin.cc \

GRPC_OBJECTIVE_C_PLUGIN_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(GRPC_OBJECTIVE_C_PLUGIN_SRC))))



ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/grpc_objective_c_plugin: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/grpc_objective_c_plugin: $(PROTOBUF_DEP) $(GRPC_OBJECTIVE_C_PLUGIN_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_plugin_support.a
	$(E) "[HOSTLD]  Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(HOST_LDXX) $(HOST_LDFLAGS) $(GRPC_OBJECTIVE_C_PLUGIN_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_plugin_support.a $(HOST_LDLIBSXX) $(HOST_LDLIBS_PROTOC) $(HOST_LDLIBS) $(HOST_LDLIBS_PROTOC) -o $(BINDIR)/$(CONFIG)/grpc_objective_c_plugin

endif

$(OBJDIR)/$(CONFIG)/src/compiler/objective_c_plugin.o:  $(LIBDIR)/$(CONFIG)/libgrpc_plugin_support.a
deps_grpc_objective_c_plugin: $(GRPC_OBJECTIVE_C_PLUGIN_OBJS:.o=.dep)

ifneq ($(NO_DEPS),true)
-include $(GRPC_OBJECTIVE_C_PLUGIN_OBJS:.o=.dep)
endif


GRPC_PYTHON_PLUGIN_SRC = \
    src/compiler/python_plugin.cc \

GRPC_PYTHON_PLUGIN_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(GRPC_PYTHON_PLUGIN_SRC))))



ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/grpc_python_plugin: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/grpc_python_plugin: $(PROTOBUF_DEP) $(GRPC_PYTHON_PLUGIN_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_plugin_support.a
	$(E) "[HOSTLD]  Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(HOST_LDXX) $(HOST_LDFLAGS) $(GRPC_PYTHON_PLUGIN_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_plugin_support.a $(HOST_LDLIBSXX) $(HOST_LDLIBS_PROTOC) $(HOST_LDLIBS) $(HOST_LDLIBS_PROTOC) -o $(BINDIR)/$(CONFIG)/grpc_python_plugin

endif

$(OBJDIR)/$(CONFIG)/src/compiler/python_plugin.o:  $(LIBDIR)/$(CONFIG)/libgrpc_plugin_support.a
deps_grpc_python_plugin: $(GRPC_PYTHON_PLUGIN_OBJS:.o=.dep)

ifneq ($(NO_DEPS),true)
-include $(GRPC_PYTHON_PLUGIN_OBJS:.o=.dep)
endif


GRPC_RUBY_PLUGIN_SRC = \
    src/compiler/ruby_plugin.cc \

GRPC_RUBY_PLUGIN_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(GRPC_RUBY_PLUGIN_SRC))))



ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/grpc_ruby_plugin: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/grpc_ruby_plugin: $(PROTOBUF_DEP) $(GRPC_RUBY_PLUGIN_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_plugin_support.a
	$(E) "[HOSTLD]  Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(HOST_LDXX) $(HOST_LDFLAGS) $(GRPC_RUBY_PLUGIN_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_plugin_support.a $(HOST_LDLIBSXX) $(HOST_LDLIBS_PROTOC) $(HOST_LDLIBS) $(HOST_LDLIBS_PROTOC) -o $(BINDIR)/$(CONFIG)/grpc_ruby_plugin

endif

$(OBJDIR)/$(CONFIG)/src/compiler/ruby_plugin.o:  $(LIBDIR)/$(CONFIG)/libgrpc_plugin_support.a
deps_grpc_ruby_plugin: $(GRPC_RUBY_PLUGIN_OBJS:.o=.dep)

ifneq ($(NO_DEPS),true)
-include $(GRPC_RUBY_PLUGIN_OBJS:.o=.dep)
endif


ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/interop_client: openssl_dep_error

else




ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/interop_client: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/interop_client:  $(LIBDIR)/$(CONFIG)/libinterop_client_main.a $(LIBDIR)/$(CONFIG)/libinterop_client_helper.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_config.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS)  $(LIBDIR)/$(CONFIG)/libinterop_client_main.a $(LIBDIR)/$(CONFIG)/libinterop_client_helper.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_config.a $(LDLIBSXX) $(LDLIBS_PROTOBUF) $(LDLIBS) $(LDLIBS_SECURE) $(GTEST_LIB) -o $(BINDIR)/$(CONFIG)/interop_client

endif

endif




ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/interop_server: openssl_dep_error

else




ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/interop_server: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/interop_server:  $(LIBDIR)/$(CONFIG)/libinterop_server_main.a $(LIBDIR)/$(CONFIG)/libinterop_server_helper.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_config.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS)  $(LIBDIR)/$(CONFIG)/libinterop_server_main.a $(LIBDIR)/$(CONFIG)/libinterop_server_helper.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_config.a $(LDLIBSXX) $(LDLIBS_PROTOBUF) $(LDLIBS) $(LDLIBS_SECURE) $(GTEST_LIB) -o $(BINDIR)/$(CONFIG)/interop_server

endif

endif




INTEROP_TEST_SRC = \
    test/cpp/interop/interop_test.cc \

INTEROP_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(INTEROP_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/interop_test: openssl_dep_error

else




ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/interop_test: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/interop_test: $(PROTOBUF_DEP) $(INTEROP_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS) $(INTEROP_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBSXX) $(LDLIBS_PROTOBUF) $(LDLIBS) $(LDLIBS_SECURE) $(GTEST_LIB) -o $(BINDIR)/$(CONFIG)/interop_test

endif

endif

$(OBJDIR)/$(CONFIG)/test/cpp/interop/interop_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_interop_test: $(INTEROP_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(INTEROP_TEST_OBJS:.o=.dep)
endif
endif


METRICS_CLIENT_SRC = \
    $(GENDIR)/test/proto/metrics.pb.cc $(GENDIR)/test/proto/metrics.grpc.pb.cc \
    test/cpp/interop/metrics_client.cc \

METRICS_CLIENT_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(METRICS_CLIENT_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/metrics_client: openssl_dep_error

else




ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/metrics_client: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/metrics_client: $(PROTOBUF_DEP) $(METRICS_CLIENT_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_config.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS) $(METRICS_CLIENT_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_config.a $(LDLIBSXX) $(LDLIBS_PROTOBUF) $(LDLIBS) $(LDLIBS_SECURE) $(GTEST_LIB) -o $(BINDIR)/$(CONFIG)/metrics_client

endif

endif

$(OBJDIR)/$(CONFIG)/test/proto/metrics.o:  $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_config.a
$(OBJDIR)/$(CONFIG)/test/cpp/interop/metrics_client.o:  $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_config.a
deps_metrics_client: $(METRICS_CLIENT_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(METRICS_CLIENT_OBJS:.o=.dep)
endif
endif
$(OBJDIR)/$(CONFIG)/test/cpp/interop/metrics_client.o: $(GENDIR)/test/proto/metrics.pb.cc $(GENDIR)/test/proto/metrics.grpc.pb.cc


MOCK_TEST_SRC = \
    test/cpp/end2end/mock_test.cc \

MOCK_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(MOCK_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/mock_test: openssl_dep_error

else




ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/mock_test: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/mock_test: $(PROTOBUF_DEP) $(MOCK_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS) $(MOCK_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBSXX) $(LDLIBS_PROTOBUF) $(LDLIBS) $(LDLIBS_SECURE) $(GTEST_LIB) -o $(BINDIR)/$(CONFIG)/mock_test

endif

endif

$(OBJDIR)/$(CONFIG)/test/cpp/end2end/mock_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_mock_test: $(MOCK_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(MOCK_TEST_OBJS:.o=.dep)
endif
endif


QPS_DRIVER_SRC = \
    test/cpp/qps/qps_driver.cc \

QPS_DRIVER_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(QPS_DRIVER_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/qps_driver: openssl_dep_error

else




ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/qps_driver: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/qps_driver: $(PROTOBUF_DEP) $(QPS_DRIVER_OBJS) $(LIBDIR)/$(CONFIG)/libqps.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_config.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS) $(QPS_DRIVER_OBJS) $(LIBDIR)/$(CONFIG)/libqps.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_config.a $(LDLIBSXX) $(LDLIBS_PROTOBUF) $(LDLIBS) $(LDLIBS_SECURE) $(GTEST_LIB) -o $(BINDIR)/$(CONFIG)/qps_driver

endif

endif

$(OBJDIR)/$(CONFIG)/test/cpp/qps/qps_driver.o:  $(LIBDIR)/$(CONFIG)/libqps.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_config.a
deps_qps_driver: $(QPS_DRIVER_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(QPS_DRIVER_OBJS:.o=.dep)
endif
endif


QPS_INTERARRIVAL_TEST_SRC = \
    test/cpp/qps/qps_interarrival_test.cc \

QPS_INTERARRIVAL_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(QPS_INTERARRIVAL_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/qps_interarrival_test: openssl_dep_error

else




ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/qps_interarrival_test: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/qps_interarrival_test: $(PROTOBUF_DEP) $(QPS_INTERARRIVAL_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libqps.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS) $(QPS_INTERARRIVAL_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libqps.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBSXX) $(LDLIBS_PROTOBUF) $(LDLIBS) $(LDLIBS_SECURE) $(GTEST_LIB) -o $(BINDIR)/$(CONFIG)/qps_interarrival_test

endif

endif

$(OBJDIR)/$(CONFIG)/test/cpp/qps/qps_interarrival_test.o:  $(LIBDIR)/$(CONFIG)/libqps.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_qps_interarrival_test: $(QPS_INTERARRIVAL_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(QPS_INTERARRIVAL_TEST_OBJS:.o=.dep)
endif
endif


QPS_OPENLOOP_TEST_SRC = \
    test/cpp/qps/qps_openloop_test.cc \

QPS_OPENLOOP_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(QPS_OPENLOOP_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/qps_openloop_test: openssl_dep_error

else




ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/qps_openloop_test: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/qps_openloop_test: $(PROTOBUF_DEP) $(QPS_OPENLOOP_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libqps.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_config.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS) $(QPS_OPENLOOP_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libqps.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_config.a $(LDLIBSXX) $(LDLIBS_PROTOBUF) $(LDLIBS) $(LDLIBS_SECURE) $(GTEST_LIB) -o $(BINDIR)/$(CONFIG)/qps_openloop_test

endif

endif

$(OBJDIR)/$(CONFIG)/test/cpp/qps/qps_openloop_test.o:  $(LIBDIR)/$(CONFIG)/libqps.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_config.a
deps_qps_openloop_test: $(QPS_OPENLOOP_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(QPS_OPENLOOP_TEST_OBJS:.o=.dep)
endif
endif


QPS_TEST_SRC = \
    test/cpp/qps/qps_test.cc \

QPS_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(QPS_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/qps_test: openssl_dep_error

else




ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/qps_test: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/qps_test: $(PROTOBUF_DEP) $(QPS_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libqps.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_config.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS) $(QPS_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libqps.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_config.a $(LDLIBSXX) $(LDLIBS_PROTOBUF) $(LDLIBS) $(LDLIBS_SECURE) $(GTEST_LIB) -o $(BINDIR)/$(CONFIG)/qps_test

endif

endif

$(OBJDIR)/$(CONFIG)/test/cpp/qps/qps_test.o:  $(LIBDIR)/$(CONFIG)/libqps.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_config.a
deps_qps_test: $(QPS_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(QPS_TEST_OBJS:.o=.dep)
endif
endif


QPS_WORKER_SRC = \
    test/cpp/qps/worker.cc \

QPS_WORKER_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(QPS_WORKER_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/qps_worker: openssl_dep_error

else




ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/qps_worker: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/qps_worker: $(PROTOBUF_DEP) $(QPS_WORKER_OBJS) $(LIBDIR)/$(CONFIG)/libqps.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_config.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS) $(QPS_WORKER_OBJS) $(LIBDIR)/$(CONFIG)/libqps.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_config.a $(LDLIBSXX) $(LDLIBS_PROTOBUF) $(LDLIBS) $(LDLIBS_SECURE) $(GTEST_LIB) -o $(BINDIR)/$(CONFIG)/qps_worker

endif

endif

$(OBJDIR)/$(CONFIG)/test/cpp/qps/worker.o:  $(LIBDIR)/$(CONFIG)/libqps.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_config.a
deps_qps_worker: $(QPS_WORKER_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(QPS_WORKER_OBJS:.o=.dep)
endif
endif


RECONNECT_INTEROP_CLIENT_SRC = \
    $(GENDIR)/test/proto/empty.pb.cc $(GENDIR)/test/proto/empty.grpc.pb.cc \
    $(GENDIR)/test/proto/messages.pb.cc $(GENDIR)/test/proto/messages.grpc.pb.cc \
    $(GENDIR)/test/proto/test.pb.cc $(GENDIR)/test/proto/test.grpc.pb.cc \
    test/cpp/interop/reconnect_interop_client.cc \

RECONNECT_INTEROP_CLIENT_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(RECONNECT_INTEROP_CLIENT_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/reconnect_interop_client: openssl_dep_error

else




ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/reconnect_interop_client: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/reconnect_interop_client: $(PROTOBUF_DEP) $(RECONNECT_INTEROP_CLIENT_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_config.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS) $(RECONNECT_INTEROP_CLIENT_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_config.a $(LDLIBSXX) $(LDLIBS_PROTOBUF) $(LDLIBS) $(LDLIBS_SECURE) $(GTEST_LIB) -o $(BINDIR)/$(CONFIG)/reconnect_interop_client

endif

endif

$(OBJDIR)/$(CONFIG)/test/proto/empty.o:  $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_config.a
$(OBJDIR)/$(CONFIG)/test/proto/messages.o:  $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_config.a
$(OBJDIR)/$(CONFIG)/test/proto/test.o:  $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_config.a
$(OBJDIR)/$(CONFIG)/test/cpp/interop/reconnect_interop_client.o:  $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_config.a
deps_reconnect_interop_client: $(RECONNECT_INTEROP_CLIENT_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(RECONNECT_INTEROP_CLIENT_OBJS:.o=.dep)
endif
endif
$(OBJDIR)/$(CONFIG)/test/cpp/interop/reconnect_interop_client.o: $(GENDIR)/test/proto/empty.pb.cc $(GENDIR)/test/proto/empty.grpc.pb.cc $(GENDIR)/test/proto/messages.pb.cc $(GENDIR)/test/proto/messages.grpc.pb.cc $(GENDIR)/test/proto/test.pb.cc $(GENDIR)/test/proto/test.grpc.pb.cc


RECONNECT_INTEROP_SERVER_SRC = \
    $(GENDIR)/test/proto/empty.pb.cc $(GENDIR)/test/proto/empty.grpc.pb.cc \
    $(GENDIR)/test/proto/messages.pb.cc $(GENDIR)/test/proto/messages.grpc.pb.cc \
    $(GENDIR)/test/proto/test.pb.cc $(GENDIR)/test/proto/test.grpc.pb.cc \
    test/cpp/interop/reconnect_interop_server.cc \

RECONNECT_INTEROP_SERVER_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(RECONNECT_INTEROP_SERVER_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/reconnect_interop_server: openssl_dep_error

else




ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/reconnect_interop_server: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/reconnect_interop_server: $(PROTOBUF_DEP) $(RECONNECT_INTEROP_SERVER_OBJS) $(LIBDIR)/$(CONFIG)/libreconnect_server.a $(LIBDIR)/$(CONFIG)/libtest_tcp_server.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_config.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS) $(RECONNECT_INTEROP_SERVER_OBJS) $(LIBDIR)/$(CONFIG)/libreconnect_server.a $(LIBDIR)/$(CONFIG)/libtest_tcp_server.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_config.a $(LDLIBSXX) $(LDLIBS_PROTOBUF) $(LDLIBS) $(LDLIBS_SECURE) $(GTEST_LIB) -o $(BINDIR)/$(CONFIG)/reconnect_interop_server

endif

endif

$(OBJDIR)/$(CONFIG)/test/proto/empty.o:  $(LIBDIR)/$(CONFIG)/libreconnect_server.a $(LIBDIR)/$(CONFIG)/libtest_tcp_server.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_config.a
$(OBJDIR)/$(CONFIG)/test/proto/messages.o:  $(LIBDIR)/$(CONFIG)/libreconnect_server.a $(LIBDIR)/$(CONFIG)/libtest_tcp_server.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_config.a
$(OBJDIR)/$(CONFIG)/test/proto/test.o:  $(LIBDIR)/$(CONFIG)/libreconnect_server.a $(LIBDIR)/$(CONFIG)/libtest_tcp_server.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_config.a
$(OBJDIR)/$(CONFIG)/test/cpp/interop/reconnect_interop_server.o:  $(LIBDIR)/$(CONFIG)/libreconnect_server.a $(LIBDIR)/$(CONFIG)/libtest_tcp_server.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_config.a
deps_reconnect_interop_server: $(RECONNECT_INTEROP_SERVER_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(RECONNECT_INTEROP_SERVER_OBJS:.o=.dep)
endif
endif
$(OBJDIR)/$(CONFIG)/test/cpp/interop/reconnect_interop_server.o: $(GENDIR)/test/proto/empty.pb.cc $(GENDIR)/test/proto/empty.grpc.pb.cc $(GENDIR)/test/proto/messages.pb.cc $(GENDIR)/test/proto/messages.grpc.pb.cc $(GENDIR)/test/proto/test.pb.cc $(GENDIR)/test/proto/test.grpc.pb.cc


SECURE_AUTH_CONTEXT_TEST_SRC = \
    test/cpp/common/secure_auth_context_test.cc \

SECURE_AUTH_CONTEXT_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(SECURE_AUTH_CONTEXT_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/secure_auth_context_test: openssl_dep_error

else




ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/secure_auth_context_test: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/secure_auth_context_test: $(PROTOBUF_DEP) $(SECURE_AUTH_CONTEXT_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS) $(SECURE_AUTH_CONTEXT_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBSXX) $(LDLIBS_PROTOBUF) $(LDLIBS) $(LDLIBS_SECURE) $(GTEST_LIB) -o $(BINDIR)/$(CONFIG)/secure_auth_context_test

endif

endif

$(OBJDIR)/$(CONFIG)/test/cpp/common/secure_auth_context_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_secure_auth_context_test: $(SECURE_AUTH_CONTEXT_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(SECURE_AUTH_CONTEXT_TEST_OBJS:.o=.dep)
endif
endif


SECURE_SYNC_UNARY_PING_PONG_TEST_SRC = \
    test/cpp/qps/secure_sync_unary_ping_pong_test.cc \

SECURE_SYNC_UNARY_PING_PONG_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(SECURE_SYNC_UNARY_PING_PONG_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/secure_sync_unary_ping_pong_test: openssl_dep_error

else




ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/secure_sync_unary_ping_pong_test: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/secure_sync_unary_ping_pong_test: $(PROTOBUF_DEP) $(SECURE_SYNC_UNARY_PING_PONG_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libqps.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS) $(SECURE_SYNC_UNARY_PING_PONG_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libqps.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBSXX) $(LDLIBS_PROTOBUF) $(LDLIBS) $(LDLIBS_SECURE) $(GTEST_LIB) -o $(BINDIR)/$(CONFIG)/secure_sync_unary_ping_pong_test

endif

endif

$(OBJDIR)/$(CONFIG)/test/cpp/qps/secure_sync_unary_ping_pong_test.o:  $(LIBDIR)/$(CONFIG)/libqps.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_secure_sync_unary_ping_pong_test: $(SECURE_SYNC_UNARY_PING_PONG_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(SECURE_SYNC_UNARY_PING_PONG_TEST_OBJS:.o=.dep)
endif
endif


SERVER_CRASH_TEST_SRC = \
    test/cpp/end2end/server_crash_test.cc \

SERVER_CRASH_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(SERVER_CRASH_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/server_crash_test: openssl_dep_error

else




ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/server_crash_test: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/server_crash_test: $(PROTOBUF_DEP) $(SERVER_CRASH_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS) $(SERVER_CRASH_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBSXX) $(LDLIBS_PROTOBUF) $(LDLIBS) $(LDLIBS_SECURE) $(GTEST_LIB) -o $(BINDIR)/$(CONFIG)/server_crash_test

endif

endif

$(OBJDIR)/$(CONFIG)/test/cpp/end2end/server_crash_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_server_crash_test: $(SERVER_CRASH_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(SERVER_CRASH_TEST_OBJS:.o=.dep)
endif
endif


SERVER_CRASH_TEST_CLIENT_SRC = \
    test/cpp/end2end/server_crash_test_client.cc \

SERVER_CRASH_TEST_CLIENT_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(SERVER_CRASH_TEST_CLIENT_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/server_crash_test_client: openssl_dep_error

else




ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/server_crash_test_client: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/server_crash_test_client: $(PROTOBUF_DEP) $(SERVER_CRASH_TEST_CLIENT_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS) $(SERVER_CRASH_TEST_CLIENT_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBSXX) $(LDLIBS_PROTOBUF) $(LDLIBS) $(LDLIBS_SECURE) $(GTEST_LIB) -o $(BINDIR)/$(CONFIG)/server_crash_test_client

endif

endif

$(OBJDIR)/$(CONFIG)/test/cpp/end2end/server_crash_test_client.o:  $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_server_crash_test_client: $(SERVER_CRASH_TEST_CLIENT_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(SERVER_CRASH_TEST_CLIENT_OBJS:.o=.dep)
endif
endif


SHUTDOWN_TEST_SRC = \
    test/cpp/end2end/shutdown_test.cc \

SHUTDOWN_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(SHUTDOWN_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/shutdown_test: openssl_dep_error

else




ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/shutdown_test: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/shutdown_test: $(PROTOBUF_DEP) $(SHUTDOWN_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS) $(SHUTDOWN_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBSXX) $(LDLIBS_PROTOBUF) $(LDLIBS) $(LDLIBS_SECURE) $(GTEST_LIB) -o $(BINDIR)/$(CONFIG)/shutdown_test

endif

endif

$(OBJDIR)/$(CONFIG)/test/cpp/end2end/shutdown_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_shutdown_test: $(SHUTDOWN_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(SHUTDOWN_TEST_OBJS:.o=.dep)
endif
endif


STATUS_TEST_SRC = \
    test/cpp/util/status_test.cc \

STATUS_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(STATUS_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/status_test: openssl_dep_error

else




ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/status_test: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/status_test: $(PROTOBUF_DEP) $(STATUS_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS) $(STATUS_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBSXX) $(LDLIBS_PROTOBUF) $(LDLIBS) $(LDLIBS_SECURE) $(GTEST_LIB) -o $(BINDIR)/$(CONFIG)/status_test

endif

endif

$(OBJDIR)/$(CONFIG)/test/cpp/util/status_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_status_test: $(STATUS_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(STATUS_TEST_OBJS:.o=.dep)
endif
endif


STREAMING_THROUGHPUT_TEST_SRC = \
    test/cpp/end2end/streaming_throughput_test.cc \

STREAMING_THROUGHPUT_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(STREAMING_THROUGHPUT_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/streaming_throughput_test: openssl_dep_error

else




ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/streaming_throughput_test: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/streaming_throughput_test: $(PROTOBUF_DEP) $(STREAMING_THROUGHPUT_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS) $(STREAMING_THROUGHPUT_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBSXX) $(LDLIBS_PROTOBUF) $(LDLIBS) $(LDLIBS_SECURE) $(GTEST_LIB) -o $(BINDIR)/$(CONFIG)/streaming_throughput_test

endif

endif

$(OBJDIR)/$(CONFIG)/test/cpp/end2end/streaming_throughput_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_streaming_throughput_test: $(STREAMING_THROUGHPUT_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(STREAMING_THROUGHPUT_TEST_OBJS:.o=.dep)
endif
endif


STRESS_TEST_SRC = \
    $(GENDIR)/test/proto/empty.pb.cc $(GENDIR)/test/proto/empty.grpc.pb.cc \
    $(GENDIR)/test/proto/messages.pb.cc $(GENDIR)/test/proto/messages.grpc.pb.cc \
    $(GENDIR)/test/proto/metrics.pb.cc $(GENDIR)/test/proto/metrics.grpc.pb.cc \
    $(GENDIR)/test/proto/test.pb.cc $(GENDIR)/test/proto/test.grpc.pb.cc \
    test/cpp/interop/interop_client.cc \
    test/cpp/interop/stress_interop_client.cc \
    test/cpp/interop/stress_test.cc \
    test/cpp/util/metrics_server.cc \

STRESS_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(STRESS_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/stress_test: openssl_dep_error

else




ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/stress_test: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/stress_test: $(PROTOBUF_DEP) $(STRESS_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_config.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS) $(STRESS_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_config.a $(LDLIBSXX) $(LDLIBS_PROTOBUF) $(LDLIBS) $(LDLIBS_SECURE) $(GTEST_LIB) -o $(BINDIR)/$(CONFIG)/stress_test

endif

endif

$(OBJDIR)/$(CONFIG)/test/proto/empty.o:  $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_config.a
$(OBJDIR)/$(CONFIG)/test/proto/messages.o:  $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_config.a
$(OBJDIR)/$(CONFIG)/test/proto/metrics.o:  $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_config.a
$(OBJDIR)/$(CONFIG)/test/proto/test.o:  $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_config.a
$(OBJDIR)/$(CONFIG)/test/cpp/interop/interop_client.o:  $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_config.a
$(OBJDIR)/$(CONFIG)/test/cpp/interop/stress_interop_client.o:  $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_config.a
$(OBJDIR)/$(CONFIG)/test/cpp/interop/stress_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_config.a
$(OBJDIR)/$(CONFIG)/test/cpp/util/metrics_server.o:  $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_config.a
deps_stress_test: $(STRESS_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(STRESS_TEST_OBJS:.o=.dep)
endif
endif
$(OBJDIR)/$(CONFIG)/test/cpp/interop/interop_client.o: $(GENDIR)/test/proto/empty.pb.cc $(GENDIR)/test/proto/empty.grpc.pb.cc $(GENDIR)/test/proto/messages.pb.cc $(GENDIR)/test/proto/messages.grpc.pb.cc $(GENDIR)/test/proto/metrics.pb.cc $(GENDIR)/test/proto/metrics.grpc.pb.cc $(GENDIR)/test/proto/test.pb.cc $(GENDIR)/test/proto/test.grpc.pb.cc
$(OBJDIR)/$(CONFIG)/test/cpp/interop/stress_interop_client.o: $(GENDIR)/test/proto/empty.pb.cc $(GENDIR)/test/proto/empty.grpc.pb.cc $(GENDIR)/test/proto/messages.pb.cc $(GENDIR)/test/proto/messages.grpc.pb.cc $(GENDIR)/test/proto/metrics.pb.cc $(GENDIR)/test/proto/metrics.grpc.pb.cc $(GENDIR)/test/proto/test.pb.cc $(GENDIR)/test/proto/test.grpc.pb.cc
$(OBJDIR)/$(CONFIG)/test/cpp/interop/stress_test.o: $(GENDIR)/test/proto/empty.pb.cc $(GENDIR)/test/proto/empty.grpc.pb.cc $(GENDIR)/test/proto/messages.pb.cc $(GENDIR)/test/proto/messages.grpc.pb.cc $(GENDIR)/test/proto/metrics.pb.cc $(GENDIR)/test/proto/metrics.grpc.pb.cc $(GENDIR)/test/proto/test.pb.cc $(GENDIR)/test/proto/test.grpc.pb.cc
$(OBJDIR)/$(CONFIG)/test/cpp/util/metrics_server.o: $(GENDIR)/test/proto/empty.pb.cc $(GENDIR)/test/proto/empty.grpc.pb.cc $(GENDIR)/test/proto/messages.pb.cc $(GENDIR)/test/proto/messages.grpc.pb.cc $(GENDIR)/test/proto/metrics.pb.cc $(GENDIR)/test/proto/metrics.grpc.pb.cc $(GENDIR)/test/proto/test.pb.cc $(GENDIR)/test/proto/test.grpc.pb.cc


SYNC_STREAMING_PING_PONG_TEST_SRC = \
    test/cpp/qps/sync_streaming_ping_pong_test.cc \

SYNC_STREAMING_PING_PONG_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(SYNC_STREAMING_PING_PONG_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/sync_streaming_ping_pong_test: openssl_dep_error

else




ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/sync_streaming_ping_pong_test: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/sync_streaming_ping_pong_test: $(PROTOBUF_DEP) $(SYNC_STREAMING_PING_PONG_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libqps.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS) $(SYNC_STREAMING_PING_PONG_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libqps.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBSXX) $(LDLIBS_PROTOBUF) $(LDLIBS) $(LDLIBS_SECURE) $(GTEST_LIB) -o $(BINDIR)/$(CONFIG)/sync_streaming_ping_pong_test

endif

endif

$(OBJDIR)/$(CONFIG)/test/cpp/qps/sync_streaming_ping_pong_test.o:  $(LIBDIR)/$(CONFIG)/libqps.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_sync_streaming_ping_pong_test: $(SYNC_STREAMING_PING_PONG_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(SYNC_STREAMING_PING_PONG_TEST_OBJS:.o=.dep)
endif
endif


SYNC_UNARY_PING_PONG_TEST_SRC = \
    test/cpp/qps/sync_unary_ping_pong_test.cc \

SYNC_UNARY_PING_PONG_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(SYNC_UNARY_PING_PONG_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/sync_unary_ping_pong_test: openssl_dep_error

else




ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/sync_unary_ping_pong_test: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/sync_unary_ping_pong_test: $(PROTOBUF_DEP) $(SYNC_UNARY_PING_PONG_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libqps.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS) $(SYNC_UNARY_PING_PONG_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libqps.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBSXX) $(LDLIBS_PROTOBUF) $(LDLIBS) $(LDLIBS_SECURE) $(GTEST_LIB) -o $(BINDIR)/$(CONFIG)/sync_unary_ping_pong_test

endif

endif

$(OBJDIR)/$(CONFIG)/test/cpp/qps/sync_unary_ping_pong_test.o:  $(LIBDIR)/$(CONFIG)/libqps.a $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_sync_unary_ping_pong_test: $(SYNC_UNARY_PING_PONG_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(SYNC_UNARY_PING_PONG_TEST_OBJS:.o=.dep)
endif
endif


THREAD_STRESS_TEST_SRC = \
    test/cpp/end2end/thread_stress_test.cc \

THREAD_STRESS_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(THREAD_STRESS_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/thread_stress_test: openssl_dep_error

else




ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/thread_stress_test: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/thread_stress_test: $(PROTOBUF_DEP) $(THREAD_STRESS_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS) $(THREAD_STRESS_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBSXX) $(LDLIBS_PROTOBUF) $(LDLIBS) $(LDLIBS_SECURE) $(GTEST_LIB) -o $(BINDIR)/$(CONFIG)/thread_stress_test

endif

endif

$(OBJDIR)/$(CONFIG)/test/cpp/end2end/thread_stress_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_thread_stress_test: $(THREAD_STRESS_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(THREAD_STRESS_TEST_OBJS:.o=.dep)
endif
endif


ZOOKEEPER_TEST_SRC = \
    test/cpp/end2end/zookeeper_test.cc \

ZOOKEEPER_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(ZOOKEEPER_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/zookeeper_test: openssl_dep_error

else




ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/zookeeper_test: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/zookeeper_test: $(PROTOBUF_DEP) $(ZOOKEEPER_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc_zookeeper.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS) $(ZOOKEEPER_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc_zookeeper.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a -lzookeeper_mt $(LDLIBSXX) $(LDLIBS_PROTOBUF) $(LDLIBS) $(LDLIBS_SECURE) $(GTEST_LIB) -o $(BINDIR)/$(CONFIG)/zookeeper_test

endif

endif

$(OBJDIR)/$(CONFIG)/test/cpp/end2end/zookeeper_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc++_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc++.a $(LIBDIR)/$(CONFIG)/libgrpc_zookeeper.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_zookeeper_test: $(ZOOKEEPER_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(ZOOKEEPER_TEST_OBJS:.o=.dep)
endif
endif



# boringssl needs an override to ensure that it does not include
# system openssl headers regardless of other configuration
# we do so here with a target specific variable assignment
$(BORINGSSL_AES_TEST_OBJS): CFLAGS := -Ithird_party/boringssl/include $(CFLAGS) -Wno-sign-conversion -Wno-conversion -Wno-unused-value
$(BORINGSSL_AES_TEST_OBJS): CXXFLAGS := -Ithird_party/boringssl/include $(CXXFLAGS)
$(BORINGSSL_AES_TEST_OBJS): CPPFLAGS += -DOPENSSL_NO_ASM -D_GNU_SOURCE


ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/boringssl_aes_test: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/boringssl_aes_test:  $(LIBDIR)/$(CONFIG)/libboringssl_aes_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_test_util.a $(LIBDIR)/$(CONFIG)/libboringssl.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS)  $(LIBDIR)/$(CONFIG)/libboringssl_aes_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_test_util.a $(LIBDIR)/$(CONFIG)/libboringssl.a $(LDLIBSXX) $(LDLIBS_PROTOBUF) $(LDLIBS) $(GTEST_LIB) -o $(BINDIR)/$(CONFIG)/boringssl_aes_test

endif





# boringssl needs an override to ensure that it does not include
# system openssl headers regardless of other configuration
# we do so here with a target specific variable assignment
$(BORINGSSL_BASE64_TEST_OBJS): CFLAGS := -Ithird_party/boringssl/include $(CFLAGS) -Wno-sign-conversion -Wno-conversion -Wno-unused-value
$(BORINGSSL_BASE64_TEST_OBJS): CXXFLAGS := -Ithird_party/boringssl/include $(CXXFLAGS)
$(BORINGSSL_BASE64_TEST_OBJS): CPPFLAGS += -DOPENSSL_NO_ASM -D_GNU_SOURCE


ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/boringssl_base64_test: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/boringssl_base64_test:  $(LIBDIR)/$(CONFIG)/libboringssl_base64_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_test_util.a $(LIBDIR)/$(CONFIG)/libboringssl.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS)  $(LIBDIR)/$(CONFIG)/libboringssl_base64_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_test_util.a $(LIBDIR)/$(CONFIG)/libboringssl.a $(LDLIBSXX) $(LDLIBS_PROTOBUF) $(LDLIBS) $(GTEST_LIB) -o $(BINDIR)/$(CONFIG)/boringssl_base64_test

endif





# boringssl needs an override to ensure that it does not include
# system openssl headers regardless of other configuration
# we do so here with a target specific variable assignment
$(BORINGSSL_BIO_TEST_OBJS): CFLAGS := -Ithird_party/boringssl/include $(CFLAGS) -Wno-sign-conversion -Wno-conversion -Wno-unused-value
$(BORINGSSL_BIO_TEST_OBJS): CXXFLAGS := -Ithird_party/boringssl/include $(CXXFLAGS)
$(BORINGSSL_BIO_TEST_OBJS): CPPFLAGS += -DOPENSSL_NO_ASM -D_GNU_SOURCE


ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/boringssl_bio_test: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/boringssl_bio_test:  $(LIBDIR)/$(CONFIG)/libboringssl_bio_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_test_util.a $(LIBDIR)/$(CONFIG)/libboringssl.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS)  $(LIBDIR)/$(CONFIG)/libboringssl_bio_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_test_util.a $(LIBDIR)/$(CONFIG)/libboringssl.a $(LDLIBSXX) $(LDLIBS_PROTOBUF) $(LDLIBS) $(GTEST_LIB) -o $(BINDIR)/$(CONFIG)/boringssl_bio_test

endif





# boringssl needs an override to ensure that it does not include
# system openssl headers regardless of other configuration
# we do so here with a target specific variable assignment
$(BORINGSSL_BN_TEST_OBJS): CFLAGS := -Ithird_party/boringssl/include $(CFLAGS) -Wno-sign-conversion -Wno-conversion -Wno-unused-value
$(BORINGSSL_BN_TEST_OBJS): CXXFLAGS := -Ithird_party/boringssl/include $(CXXFLAGS)
$(BORINGSSL_BN_TEST_OBJS): CPPFLAGS += -DOPENSSL_NO_ASM -D_GNU_SOURCE


ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/boringssl_bn_test: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/boringssl_bn_test:  $(LIBDIR)/$(CONFIG)/libboringssl_bn_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_test_util.a $(LIBDIR)/$(CONFIG)/libboringssl.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS)  $(LIBDIR)/$(CONFIG)/libboringssl_bn_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_test_util.a $(LIBDIR)/$(CONFIG)/libboringssl.a $(LDLIBSXX) $(LDLIBS_PROTOBUF) $(LDLIBS) $(GTEST_LIB) -o $(BINDIR)/$(CONFIG)/boringssl_bn_test

endif





# boringssl needs an override to ensure that it does not include
# system openssl headers regardless of other configuration
# we do so here with a target specific variable assignment
$(BORINGSSL_BYTESTRING_TEST_OBJS): CFLAGS := -Ithird_party/boringssl/include $(CFLAGS) -Wno-sign-conversion -Wno-conversion -Wno-unused-value
$(BORINGSSL_BYTESTRING_TEST_OBJS): CXXFLAGS := -Ithird_party/boringssl/include $(CXXFLAGS)
$(BORINGSSL_BYTESTRING_TEST_OBJS): CPPFLAGS += -DOPENSSL_NO_ASM -D_GNU_SOURCE


ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/boringssl_bytestring_test: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/boringssl_bytestring_test:  $(LIBDIR)/$(CONFIG)/libboringssl_bytestring_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_test_util.a $(LIBDIR)/$(CONFIG)/libboringssl.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS)  $(LIBDIR)/$(CONFIG)/libboringssl_bytestring_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_test_util.a $(LIBDIR)/$(CONFIG)/libboringssl.a $(LDLIBSXX) $(LDLIBS_PROTOBUF) $(LDLIBS) $(GTEST_LIB) -o $(BINDIR)/$(CONFIG)/boringssl_bytestring_test

endif





# boringssl needs an override to ensure that it does not include
# system openssl headers regardless of other configuration
# we do so here with a target specific variable assignment
$(BORINGSSL_AEAD_TEST_OBJS): CFLAGS := -Ithird_party/boringssl/include $(CFLAGS) -Wno-sign-conversion -Wno-conversion -Wno-unused-value
$(BORINGSSL_AEAD_TEST_OBJS): CXXFLAGS := -Ithird_party/boringssl/include $(CXXFLAGS)
$(BORINGSSL_AEAD_TEST_OBJS): CPPFLAGS += -DOPENSSL_NO_ASM -D_GNU_SOURCE


ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/boringssl_aead_test: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/boringssl_aead_test:  $(LIBDIR)/$(CONFIG)/libboringssl_aead_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_test_util.a $(LIBDIR)/$(CONFIG)/libboringssl.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS)  $(LIBDIR)/$(CONFIG)/libboringssl_aead_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_test_util.a $(LIBDIR)/$(CONFIG)/libboringssl.a $(LDLIBSXX) $(LDLIBS_PROTOBUF) $(LDLIBS) $(GTEST_LIB) -o $(BINDIR)/$(CONFIG)/boringssl_aead_test

endif





# boringssl needs an override to ensure that it does not include
# system openssl headers regardless of other configuration
# we do so here with a target specific variable assignment
$(BORINGSSL_CIPHER_TEST_OBJS): CFLAGS := -Ithird_party/boringssl/include $(CFLAGS) -Wno-sign-conversion -Wno-conversion -Wno-unused-value
$(BORINGSSL_CIPHER_TEST_OBJS): CXXFLAGS := -Ithird_party/boringssl/include $(CXXFLAGS)
$(BORINGSSL_CIPHER_TEST_OBJS): CPPFLAGS += -DOPENSSL_NO_ASM -D_GNU_SOURCE


ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/boringssl_cipher_test: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/boringssl_cipher_test:  $(LIBDIR)/$(CONFIG)/libboringssl_cipher_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_test_util.a $(LIBDIR)/$(CONFIG)/libboringssl.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS)  $(LIBDIR)/$(CONFIG)/libboringssl_cipher_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_test_util.a $(LIBDIR)/$(CONFIG)/libboringssl.a $(LDLIBSXX) $(LDLIBS_PROTOBUF) $(LDLIBS) $(GTEST_LIB) -o $(BINDIR)/$(CONFIG)/boringssl_cipher_test

endif





# boringssl needs an override to ensure that it does not include
# system openssl headers regardless of other configuration
# we do so here with a target specific variable assignment
$(BORINGSSL_CMAC_TEST_OBJS): CFLAGS := -Ithird_party/boringssl/include $(CFLAGS) -Wno-sign-conversion -Wno-conversion -Wno-unused-value
$(BORINGSSL_CMAC_TEST_OBJS): CXXFLAGS := -Ithird_party/boringssl/include $(CXXFLAGS)
$(BORINGSSL_CMAC_TEST_OBJS): CPPFLAGS += -DOPENSSL_NO_ASM -D_GNU_SOURCE


ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/boringssl_cmac_test: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/boringssl_cmac_test:  $(LIBDIR)/$(CONFIG)/libboringssl_cmac_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_test_util.a $(LIBDIR)/$(CONFIG)/libboringssl.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS)  $(LIBDIR)/$(CONFIG)/libboringssl_cmac_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_test_util.a $(LIBDIR)/$(CONFIG)/libboringssl.a $(LDLIBSXX) $(LDLIBS_PROTOBUF) $(LDLIBS) $(GTEST_LIB) -o $(BINDIR)/$(CONFIG)/boringssl_cmac_test

endif





# boringssl needs an override to ensure that it does not include
# system openssl headers regardless of other configuration
# we do so here with a target specific variable assignment
$(BORINGSSL_CONSTANT_TIME_TEST_OBJS): CFLAGS := -Ithird_party/boringssl/include $(CFLAGS) -Wno-sign-conversion -Wno-conversion -Wno-unused-value
$(BORINGSSL_CONSTANT_TIME_TEST_OBJS): CXXFLAGS := -Ithird_party/boringssl/include $(CXXFLAGS)
$(BORINGSSL_CONSTANT_TIME_TEST_OBJS): CPPFLAGS += -DOPENSSL_NO_ASM -D_GNU_SOURCE


ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/boringssl_constant_time_test: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/boringssl_constant_time_test:  $(LIBDIR)/$(CONFIG)/libboringssl_constant_time_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_test_util.a $(LIBDIR)/$(CONFIG)/libboringssl.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS)  $(LIBDIR)/$(CONFIG)/libboringssl_constant_time_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_test_util.a $(LIBDIR)/$(CONFIG)/libboringssl.a $(LDLIBSXX) $(LDLIBS_PROTOBUF) $(LDLIBS) $(GTEST_LIB) -o $(BINDIR)/$(CONFIG)/boringssl_constant_time_test

endif





# boringssl needs an override to ensure that it does not include
# system openssl headers regardless of other configuration
# we do so here with a target specific variable assignment
$(BORINGSSL_ED25519_TEST_OBJS): CFLAGS := -Ithird_party/boringssl/include $(CFLAGS) -Wno-sign-conversion -Wno-conversion -Wno-unused-value
$(BORINGSSL_ED25519_TEST_OBJS): CXXFLAGS := -Ithird_party/boringssl/include $(CXXFLAGS)
$(BORINGSSL_ED25519_TEST_OBJS): CPPFLAGS += -DOPENSSL_NO_ASM -D_GNU_SOURCE


ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/boringssl_ed25519_test: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/boringssl_ed25519_test:  $(LIBDIR)/$(CONFIG)/libboringssl_ed25519_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_test_util.a $(LIBDIR)/$(CONFIG)/libboringssl.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS)  $(LIBDIR)/$(CONFIG)/libboringssl_ed25519_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_test_util.a $(LIBDIR)/$(CONFIG)/libboringssl.a $(LDLIBSXX) $(LDLIBS_PROTOBUF) $(LDLIBS) $(GTEST_LIB) -o $(BINDIR)/$(CONFIG)/boringssl_ed25519_test

endif





# boringssl needs an override to ensure that it does not include
# system openssl headers regardless of other configuration
# we do so here with a target specific variable assignment
$(BORINGSSL_X25519_TEST_OBJS): CFLAGS := -Ithird_party/boringssl/include $(CFLAGS) -Wno-sign-conversion -Wno-conversion -Wno-unused-value
$(BORINGSSL_X25519_TEST_OBJS): CXXFLAGS := -Ithird_party/boringssl/include $(CXXFLAGS)
$(BORINGSSL_X25519_TEST_OBJS): CPPFLAGS += -DOPENSSL_NO_ASM -D_GNU_SOURCE


ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/boringssl_x25519_test: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/boringssl_x25519_test:  $(LIBDIR)/$(CONFIG)/libboringssl_x25519_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_test_util.a $(LIBDIR)/$(CONFIG)/libboringssl.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS)  $(LIBDIR)/$(CONFIG)/libboringssl_x25519_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_test_util.a $(LIBDIR)/$(CONFIG)/libboringssl.a $(LDLIBSXX) $(LDLIBS_PROTOBUF) $(LDLIBS) $(GTEST_LIB) -o $(BINDIR)/$(CONFIG)/boringssl_x25519_test

endif





# boringssl needs an override to ensure that it does not include
# system openssl headers regardless of other configuration
# we do so here with a target specific variable assignment
$(BORINGSSL_DH_TEST_OBJS): CFLAGS := -Ithird_party/boringssl/include $(CFLAGS) -Wno-sign-conversion -Wno-conversion -Wno-unused-value
$(BORINGSSL_DH_TEST_OBJS): CXXFLAGS := -Ithird_party/boringssl/include $(CXXFLAGS)
$(BORINGSSL_DH_TEST_OBJS): CPPFLAGS += -DOPENSSL_NO_ASM -D_GNU_SOURCE


ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/boringssl_dh_test: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/boringssl_dh_test:  $(LIBDIR)/$(CONFIG)/libboringssl_dh_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_test_util.a $(LIBDIR)/$(CONFIG)/libboringssl.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS)  $(LIBDIR)/$(CONFIG)/libboringssl_dh_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_test_util.a $(LIBDIR)/$(CONFIG)/libboringssl.a $(LDLIBSXX) $(LDLIBS_PROTOBUF) $(LDLIBS) $(GTEST_LIB) -o $(BINDIR)/$(CONFIG)/boringssl_dh_test

endif





# boringssl needs an override to ensure that it does not include
# system openssl headers regardless of other configuration
# we do so here with a target specific variable assignment
$(BORINGSSL_DIGEST_TEST_OBJS): CFLAGS := -Ithird_party/boringssl/include $(CFLAGS) -Wno-sign-conversion -Wno-conversion -Wno-unused-value
$(BORINGSSL_DIGEST_TEST_OBJS): CXXFLAGS := -Ithird_party/boringssl/include $(CXXFLAGS)
$(BORINGSSL_DIGEST_TEST_OBJS): CPPFLAGS += -DOPENSSL_NO_ASM -D_GNU_SOURCE


ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/boringssl_digest_test: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/boringssl_digest_test:  $(LIBDIR)/$(CONFIG)/libboringssl_digest_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_test_util.a $(LIBDIR)/$(CONFIG)/libboringssl.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS)  $(LIBDIR)/$(CONFIG)/libboringssl_digest_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_test_util.a $(LIBDIR)/$(CONFIG)/libboringssl.a $(LDLIBSXX) $(LDLIBS_PROTOBUF) $(LDLIBS) $(GTEST_LIB) -o $(BINDIR)/$(CONFIG)/boringssl_digest_test

endif





# boringssl needs an override to ensure that it does not include
# system openssl headers regardless of other configuration
# we do so here with a target specific variable assignment
$(BORINGSSL_DSA_TEST_OBJS): CFLAGS := -Ithird_party/boringssl/include $(CFLAGS) -Wno-sign-conversion -Wno-conversion -Wno-unused-value
$(BORINGSSL_DSA_TEST_OBJS): CXXFLAGS := -Ithird_party/boringssl/include $(CXXFLAGS)
$(BORINGSSL_DSA_TEST_OBJS): CPPFLAGS += -DOPENSSL_NO_ASM -D_GNU_SOURCE


ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/boringssl_dsa_test: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/boringssl_dsa_test:  $(LIBDIR)/$(CONFIG)/libboringssl_dsa_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_test_util.a $(LIBDIR)/$(CONFIG)/libboringssl.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS)  $(LIBDIR)/$(CONFIG)/libboringssl_dsa_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_test_util.a $(LIBDIR)/$(CONFIG)/libboringssl.a $(LDLIBSXX) $(LDLIBS_PROTOBUF) $(LDLIBS) $(GTEST_LIB) -o $(BINDIR)/$(CONFIG)/boringssl_dsa_test

endif





# boringssl needs an override to ensure that it does not include
# system openssl headers regardless of other configuration
# we do so here with a target specific variable assignment
$(BORINGSSL_EC_TEST_OBJS): CFLAGS := -Ithird_party/boringssl/include $(CFLAGS) -Wno-sign-conversion -Wno-conversion -Wno-unused-value
$(BORINGSSL_EC_TEST_OBJS): CXXFLAGS := -Ithird_party/boringssl/include $(CXXFLAGS)
$(BORINGSSL_EC_TEST_OBJS): CPPFLAGS += -DOPENSSL_NO_ASM -D_GNU_SOURCE


ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/boringssl_ec_test: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/boringssl_ec_test:  $(LIBDIR)/$(CONFIG)/libboringssl_ec_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_test_util.a $(LIBDIR)/$(CONFIG)/libboringssl.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS)  $(LIBDIR)/$(CONFIG)/libboringssl_ec_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_test_util.a $(LIBDIR)/$(CONFIG)/libboringssl.a $(LDLIBSXX) $(LDLIBS_PROTOBUF) $(LDLIBS) $(GTEST_LIB) -o $(BINDIR)/$(CONFIG)/boringssl_ec_test

endif





# boringssl needs an override to ensure that it does not include
# system openssl headers regardless of other configuration
# we do so here with a target specific variable assignment
$(BORINGSSL_EXAMPLE_MUL_OBJS): CFLAGS := -Ithird_party/boringssl/include $(CFLAGS) -Wno-sign-conversion -Wno-conversion -Wno-unused-value
$(BORINGSSL_EXAMPLE_MUL_OBJS): CXXFLAGS := -Ithird_party/boringssl/include $(CXXFLAGS)
$(BORINGSSL_EXAMPLE_MUL_OBJS): CPPFLAGS += -DOPENSSL_NO_ASM -D_GNU_SOURCE


ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/boringssl_example_mul: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/boringssl_example_mul:  $(LIBDIR)/$(CONFIG)/libboringssl_example_mul_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_test_util.a $(LIBDIR)/$(CONFIG)/libboringssl.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS)  $(LIBDIR)/$(CONFIG)/libboringssl_example_mul_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_test_util.a $(LIBDIR)/$(CONFIG)/libboringssl.a $(LDLIBSXX) $(LDLIBS_PROTOBUF) $(LDLIBS) $(GTEST_LIB) -o $(BINDIR)/$(CONFIG)/boringssl_example_mul

endif





# boringssl needs an override to ensure that it does not include
# system openssl headers regardless of other configuration
# we do so here with a target specific variable assignment
$(BORINGSSL_ECDSA_TEST_OBJS): CFLAGS := -Ithird_party/boringssl/include $(CFLAGS) -Wno-sign-conversion -Wno-conversion -Wno-unused-value
$(BORINGSSL_ECDSA_TEST_OBJS): CXXFLAGS := -Ithird_party/boringssl/include $(CXXFLAGS)
$(BORINGSSL_ECDSA_TEST_OBJS): CPPFLAGS += -DOPENSSL_NO_ASM -D_GNU_SOURCE


ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/boringssl_ecdsa_test: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/boringssl_ecdsa_test:  $(LIBDIR)/$(CONFIG)/libboringssl_ecdsa_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_test_util.a $(LIBDIR)/$(CONFIG)/libboringssl.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS)  $(LIBDIR)/$(CONFIG)/libboringssl_ecdsa_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_test_util.a $(LIBDIR)/$(CONFIG)/libboringssl.a $(LDLIBSXX) $(LDLIBS_PROTOBUF) $(LDLIBS) $(GTEST_LIB) -o $(BINDIR)/$(CONFIG)/boringssl_ecdsa_test

endif





# boringssl needs an override to ensure that it does not include
# system openssl headers regardless of other configuration
# we do so here with a target specific variable assignment
$(BORINGSSL_ERR_TEST_OBJS): CFLAGS := -Ithird_party/boringssl/include $(CFLAGS) -Wno-sign-conversion -Wno-conversion -Wno-unused-value
$(BORINGSSL_ERR_TEST_OBJS): CXXFLAGS := -Ithird_party/boringssl/include $(CXXFLAGS)
$(BORINGSSL_ERR_TEST_OBJS): CPPFLAGS += -DOPENSSL_NO_ASM -D_GNU_SOURCE


ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/boringssl_err_test: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/boringssl_err_test:  $(LIBDIR)/$(CONFIG)/libboringssl_err_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_test_util.a $(LIBDIR)/$(CONFIG)/libboringssl.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS)  $(LIBDIR)/$(CONFIG)/libboringssl_err_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_test_util.a $(LIBDIR)/$(CONFIG)/libboringssl.a $(LDLIBSXX) $(LDLIBS_PROTOBUF) $(LDLIBS) $(GTEST_LIB) -o $(BINDIR)/$(CONFIG)/boringssl_err_test

endif





# boringssl needs an override to ensure that it does not include
# system openssl headers regardless of other configuration
# we do so here with a target specific variable assignment
$(BORINGSSL_EVP_EXTRA_TEST_OBJS): CFLAGS := -Ithird_party/boringssl/include $(CFLAGS) -Wno-sign-conversion -Wno-conversion -Wno-unused-value
$(BORINGSSL_EVP_EXTRA_TEST_OBJS): CXXFLAGS := -Ithird_party/boringssl/include $(CXXFLAGS)
$(BORINGSSL_EVP_EXTRA_TEST_OBJS): CPPFLAGS += -DOPENSSL_NO_ASM -D_GNU_SOURCE


ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/boringssl_evp_extra_test: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/boringssl_evp_extra_test:  $(LIBDIR)/$(CONFIG)/libboringssl_evp_extra_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_test_util.a $(LIBDIR)/$(CONFIG)/libboringssl.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS)  $(LIBDIR)/$(CONFIG)/libboringssl_evp_extra_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_test_util.a $(LIBDIR)/$(CONFIG)/libboringssl.a $(LDLIBSXX) $(LDLIBS_PROTOBUF) $(LDLIBS) $(GTEST_LIB) -o $(BINDIR)/$(CONFIG)/boringssl_evp_extra_test

endif





# boringssl needs an override to ensure that it does not include
# system openssl headers regardless of other configuration
# we do so here with a target specific variable assignment
$(BORINGSSL_EVP_TEST_OBJS): CFLAGS := -Ithird_party/boringssl/include $(CFLAGS) -Wno-sign-conversion -Wno-conversion -Wno-unused-value
$(BORINGSSL_EVP_TEST_OBJS): CXXFLAGS := -Ithird_party/boringssl/include $(CXXFLAGS)
$(BORINGSSL_EVP_TEST_OBJS): CPPFLAGS += -DOPENSSL_NO_ASM -D_GNU_SOURCE


ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/boringssl_evp_test: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/boringssl_evp_test:  $(LIBDIR)/$(CONFIG)/libboringssl_evp_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_test_util.a $(LIBDIR)/$(CONFIG)/libboringssl.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS)  $(LIBDIR)/$(CONFIG)/libboringssl_evp_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_test_util.a $(LIBDIR)/$(CONFIG)/libboringssl.a $(LDLIBSXX) $(LDLIBS_PROTOBUF) $(LDLIBS) $(GTEST_LIB) -o $(BINDIR)/$(CONFIG)/boringssl_evp_test

endif





# boringssl needs an override to ensure that it does not include
# system openssl headers regardless of other configuration
# we do so here with a target specific variable assignment
$(BORINGSSL_PBKDF_TEST_OBJS): CFLAGS := -Ithird_party/boringssl/include $(CFLAGS) -Wno-sign-conversion -Wno-conversion -Wno-unused-value
$(BORINGSSL_PBKDF_TEST_OBJS): CXXFLAGS := -Ithird_party/boringssl/include $(CXXFLAGS)
$(BORINGSSL_PBKDF_TEST_OBJS): CPPFLAGS += -DOPENSSL_NO_ASM -D_GNU_SOURCE


ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/boringssl_pbkdf_test: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/boringssl_pbkdf_test:  $(LIBDIR)/$(CONFIG)/libboringssl_pbkdf_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_test_util.a $(LIBDIR)/$(CONFIG)/libboringssl.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS)  $(LIBDIR)/$(CONFIG)/libboringssl_pbkdf_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_test_util.a $(LIBDIR)/$(CONFIG)/libboringssl.a $(LDLIBSXX) $(LDLIBS_PROTOBUF) $(LDLIBS) $(GTEST_LIB) -o $(BINDIR)/$(CONFIG)/boringssl_pbkdf_test

endif





# boringssl needs an override to ensure that it does not include
# system openssl headers regardless of other configuration
# we do so here with a target specific variable assignment
$(BORINGSSL_HKDF_TEST_OBJS): CFLAGS := -Ithird_party/boringssl/include $(CFLAGS) -Wno-sign-conversion -Wno-conversion -Wno-unused-value
$(BORINGSSL_HKDF_TEST_OBJS): CXXFLAGS := -Ithird_party/boringssl/include $(CXXFLAGS)
$(BORINGSSL_HKDF_TEST_OBJS): CPPFLAGS += -DOPENSSL_NO_ASM -D_GNU_SOURCE


ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/boringssl_hkdf_test: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/boringssl_hkdf_test:  $(LIBDIR)/$(CONFIG)/libboringssl_hkdf_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_test_util.a $(LIBDIR)/$(CONFIG)/libboringssl.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS)  $(LIBDIR)/$(CONFIG)/libboringssl_hkdf_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_test_util.a $(LIBDIR)/$(CONFIG)/libboringssl.a $(LDLIBSXX) $(LDLIBS_PROTOBUF) $(LDLIBS) $(GTEST_LIB) -o $(BINDIR)/$(CONFIG)/boringssl_hkdf_test

endif





# boringssl needs an override to ensure that it does not include
# system openssl headers regardless of other configuration
# we do so here with a target specific variable assignment
$(BORINGSSL_HMAC_TEST_OBJS): CFLAGS := -Ithird_party/boringssl/include $(CFLAGS) -Wno-sign-conversion -Wno-conversion -Wno-unused-value
$(BORINGSSL_HMAC_TEST_OBJS): CXXFLAGS := -Ithird_party/boringssl/include $(CXXFLAGS)
$(BORINGSSL_HMAC_TEST_OBJS): CPPFLAGS += -DOPENSSL_NO_ASM -D_GNU_SOURCE


ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/boringssl_hmac_test: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/boringssl_hmac_test:  $(LIBDIR)/$(CONFIG)/libboringssl_hmac_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_test_util.a $(LIBDIR)/$(CONFIG)/libboringssl.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS)  $(LIBDIR)/$(CONFIG)/libboringssl_hmac_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_test_util.a $(LIBDIR)/$(CONFIG)/libboringssl.a $(LDLIBSXX) $(LDLIBS_PROTOBUF) $(LDLIBS) $(GTEST_LIB) -o $(BINDIR)/$(CONFIG)/boringssl_hmac_test

endif





# boringssl needs an override to ensure that it does not include
# system openssl headers regardless of other configuration
# we do so here with a target specific variable assignment
$(BORINGSSL_LHASH_TEST_OBJS): CFLAGS := -Ithird_party/boringssl/include $(CFLAGS) -Wno-sign-conversion -Wno-conversion -Wno-unused-value
$(BORINGSSL_LHASH_TEST_OBJS): CXXFLAGS := -Ithird_party/boringssl/include $(CXXFLAGS)
$(BORINGSSL_LHASH_TEST_OBJS): CPPFLAGS += -DOPENSSL_NO_ASM -D_GNU_SOURCE


ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/boringssl_lhash_test: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/boringssl_lhash_test:  $(LIBDIR)/$(CONFIG)/libboringssl_lhash_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_test_util.a $(LIBDIR)/$(CONFIG)/libboringssl.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS)  $(LIBDIR)/$(CONFIG)/libboringssl_lhash_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_test_util.a $(LIBDIR)/$(CONFIG)/libboringssl.a $(LDLIBSXX) $(LDLIBS_PROTOBUF) $(LDLIBS) $(GTEST_LIB) -o $(BINDIR)/$(CONFIG)/boringssl_lhash_test

endif





# boringssl needs an override to ensure that it does not include
# system openssl headers regardless of other configuration
# we do so here with a target specific variable assignment
$(BORINGSSL_GCM_TEST_OBJS): CFLAGS := -Ithird_party/boringssl/include $(CFLAGS) -Wno-sign-conversion -Wno-conversion -Wno-unused-value
$(BORINGSSL_GCM_TEST_OBJS): CXXFLAGS := -Ithird_party/boringssl/include $(CXXFLAGS)
$(BORINGSSL_GCM_TEST_OBJS): CPPFLAGS += -DOPENSSL_NO_ASM -D_GNU_SOURCE


ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/boringssl_gcm_test: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/boringssl_gcm_test:  $(LIBDIR)/$(CONFIG)/libboringssl_gcm_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_test_util.a $(LIBDIR)/$(CONFIG)/libboringssl.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS)  $(LIBDIR)/$(CONFIG)/libboringssl_gcm_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_test_util.a $(LIBDIR)/$(CONFIG)/libboringssl.a $(LDLIBSXX) $(LDLIBS_PROTOBUF) $(LDLIBS) $(GTEST_LIB) -o $(BINDIR)/$(CONFIG)/boringssl_gcm_test

endif





# boringssl needs an override to ensure that it does not include
# system openssl headers regardless of other configuration
# we do so here with a target specific variable assignment
$(BORINGSSL_PKCS12_TEST_OBJS): CFLAGS := -Ithird_party/boringssl/include $(CFLAGS) -Wno-sign-conversion -Wno-conversion -Wno-unused-value
$(BORINGSSL_PKCS12_TEST_OBJS): CXXFLAGS := -Ithird_party/boringssl/include $(CXXFLAGS)
$(BORINGSSL_PKCS12_TEST_OBJS): CPPFLAGS += -DOPENSSL_NO_ASM -D_GNU_SOURCE


ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/boringssl_pkcs12_test: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/boringssl_pkcs12_test:  $(LIBDIR)/$(CONFIG)/libboringssl_pkcs12_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_test_util.a $(LIBDIR)/$(CONFIG)/libboringssl.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS)  $(LIBDIR)/$(CONFIG)/libboringssl_pkcs12_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_test_util.a $(LIBDIR)/$(CONFIG)/libboringssl.a $(LDLIBSXX) $(LDLIBS_PROTOBUF) $(LDLIBS) $(GTEST_LIB) -o $(BINDIR)/$(CONFIG)/boringssl_pkcs12_test

endif





# boringssl needs an override to ensure that it does not include
# system openssl headers regardless of other configuration
# we do so here with a target specific variable assignment
$(BORINGSSL_PKCS8_TEST_OBJS): CFLAGS := -Ithird_party/boringssl/include $(CFLAGS) -Wno-sign-conversion -Wno-conversion -Wno-unused-value
$(BORINGSSL_PKCS8_TEST_OBJS): CXXFLAGS := -Ithird_party/boringssl/include $(CXXFLAGS)
$(BORINGSSL_PKCS8_TEST_OBJS): CPPFLAGS += -DOPENSSL_NO_ASM -D_GNU_SOURCE


ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/boringssl_pkcs8_test: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/boringssl_pkcs8_test:  $(LIBDIR)/$(CONFIG)/libboringssl_pkcs8_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_test_util.a $(LIBDIR)/$(CONFIG)/libboringssl.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS)  $(LIBDIR)/$(CONFIG)/libboringssl_pkcs8_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_test_util.a $(LIBDIR)/$(CONFIG)/libboringssl.a $(LDLIBSXX) $(LDLIBS_PROTOBUF) $(LDLIBS) $(GTEST_LIB) -o $(BINDIR)/$(CONFIG)/boringssl_pkcs8_test

endif





# boringssl needs an override to ensure that it does not include
# system openssl headers regardless of other configuration
# we do so here with a target specific variable assignment
$(BORINGSSL_POLY1305_TEST_OBJS): CFLAGS := -Ithird_party/boringssl/include $(CFLAGS) -Wno-sign-conversion -Wno-conversion -Wno-unused-value
$(BORINGSSL_POLY1305_TEST_OBJS): CXXFLAGS := -Ithird_party/boringssl/include $(CXXFLAGS)
$(BORINGSSL_POLY1305_TEST_OBJS): CPPFLAGS += -DOPENSSL_NO_ASM -D_GNU_SOURCE


ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/boringssl_poly1305_test: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/boringssl_poly1305_test:  $(LIBDIR)/$(CONFIG)/libboringssl_poly1305_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_test_util.a $(LIBDIR)/$(CONFIG)/libboringssl.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS)  $(LIBDIR)/$(CONFIG)/libboringssl_poly1305_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_test_util.a $(LIBDIR)/$(CONFIG)/libboringssl.a $(LDLIBSXX) $(LDLIBS_PROTOBUF) $(LDLIBS) $(GTEST_LIB) -o $(BINDIR)/$(CONFIG)/boringssl_poly1305_test

endif





# boringssl needs an override to ensure that it does not include
# system openssl headers regardless of other configuration
# we do so here with a target specific variable assignment
$(BORINGSSL_REFCOUNT_TEST_OBJS): CFLAGS := -Ithird_party/boringssl/include $(CFLAGS) -Wno-sign-conversion -Wno-conversion -Wno-unused-value
$(BORINGSSL_REFCOUNT_TEST_OBJS): CXXFLAGS := -Ithird_party/boringssl/include $(CXXFLAGS)
$(BORINGSSL_REFCOUNT_TEST_OBJS): CPPFLAGS += -DOPENSSL_NO_ASM -D_GNU_SOURCE


ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/boringssl_refcount_test: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/boringssl_refcount_test:  $(LIBDIR)/$(CONFIG)/libboringssl_refcount_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_test_util.a $(LIBDIR)/$(CONFIG)/libboringssl.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS)  $(LIBDIR)/$(CONFIG)/libboringssl_refcount_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_test_util.a $(LIBDIR)/$(CONFIG)/libboringssl.a $(LDLIBSXX) $(LDLIBS_PROTOBUF) $(LDLIBS) $(GTEST_LIB) -o $(BINDIR)/$(CONFIG)/boringssl_refcount_test

endif





# boringssl needs an override to ensure that it does not include
# system openssl headers regardless of other configuration
# we do so here with a target specific variable assignment
$(BORINGSSL_RSA_TEST_OBJS): CFLAGS := -Ithird_party/boringssl/include $(CFLAGS) -Wno-sign-conversion -Wno-conversion -Wno-unused-value
$(BORINGSSL_RSA_TEST_OBJS): CXXFLAGS := -Ithird_party/boringssl/include $(CXXFLAGS)
$(BORINGSSL_RSA_TEST_OBJS): CPPFLAGS += -DOPENSSL_NO_ASM -D_GNU_SOURCE


ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/boringssl_rsa_test: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/boringssl_rsa_test:  $(LIBDIR)/$(CONFIG)/libboringssl_rsa_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_test_util.a $(LIBDIR)/$(CONFIG)/libboringssl.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS)  $(LIBDIR)/$(CONFIG)/libboringssl_rsa_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_test_util.a $(LIBDIR)/$(CONFIG)/libboringssl.a $(LDLIBSXX) $(LDLIBS_PROTOBUF) $(LDLIBS) $(GTEST_LIB) -o $(BINDIR)/$(CONFIG)/boringssl_rsa_test

endif





# boringssl needs an override to ensure that it does not include
# system openssl headers regardless of other configuration
# we do so here with a target specific variable assignment
$(BORINGSSL_THREAD_TEST_OBJS): CFLAGS := -Ithird_party/boringssl/include $(CFLAGS) -Wno-sign-conversion -Wno-conversion -Wno-unused-value
$(BORINGSSL_THREAD_TEST_OBJS): CXXFLAGS := -Ithird_party/boringssl/include $(CXXFLAGS)
$(BORINGSSL_THREAD_TEST_OBJS): CPPFLAGS += -DOPENSSL_NO_ASM -D_GNU_SOURCE


ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/boringssl_thread_test: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/boringssl_thread_test:  $(LIBDIR)/$(CONFIG)/libboringssl_thread_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_test_util.a $(LIBDIR)/$(CONFIG)/libboringssl.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS)  $(LIBDIR)/$(CONFIG)/libboringssl_thread_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_test_util.a $(LIBDIR)/$(CONFIG)/libboringssl.a $(LDLIBSXX) $(LDLIBS_PROTOBUF) $(LDLIBS) $(GTEST_LIB) -o $(BINDIR)/$(CONFIG)/boringssl_thread_test

endif





# boringssl needs an override to ensure that it does not include
# system openssl headers regardless of other configuration
# we do so here with a target specific variable assignment
$(BORINGSSL_PKCS7_TEST_OBJS): CFLAGS := -Ithird_party/boringssl/include $(CFLAGS) -Wno-sign-conversion -Wno-conversion -Wno-unused-value
$(BORINGSSL_PKCS7_TEST_OBJS): CXXFLAGS := -Ithird_party/boringssl/include $(CXXFLAGS)
$(BORINGSSL_PKCS7_TEST_OBJS): CPPFLAGS += -DOPENSSL_NO_ASM -D_GNU_SOURCE


ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/boringssl_pkcs7_test: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/boringssl_pkcs7_test:  $(LIBDIR)/$(CONFIG)/libboringssl_pkcs7_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_test_util.a $(LIBDIR)/$(CONFIG)/libboringssl.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS)  $(LIBDIR)/$(CONFIG)/libboringssl_pkcs7_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_test_util.a $(LIBDIR)/$(CONFIG)/libboringssl.a $(LDLIBSXX) $(LDLIBS_PROTOBUF) $(LDLIBS) $(GTEST_LIB) -o $(BINDIR)/$(CONFIG)/boringssl_pkcs7_test

endif





# boringssl needs an override to ensure that it does not include
# system openssl headers regardless of other configuration
# we do so here with a target specific variable assignment
$(BORINGSSL_TAB_TEST_OBJS): CFLAGS := -Ithird_party/boringssl/include $(CFLAGS) -Wno-sign-conversion -Wno-conversion -Wno-unused-value
$(BORINGSSL_TAB_TEST_OBJS): CXXFLAGS := -Ithird_party/boringssl/include $(CXXFLAGS)
$(BORINGSSL_TAB_TEST_OBJS): CPPFLAGS += -DOPENSSL_NO_ASM -D_GNU_SOURCE


ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/boringssl_tab_test: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/boringssl_tab_test:  $(LIBDIR)/$(CONFIG)/libboringssl_tab_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_test_util.a $(LIBDIR)/$(CONFIG)/libboringssl.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS)  $(LIBDIR)/$(CONFIG)/libboringssl_tab_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_test_util.a $(LIBDIR)/$(CONFIG)/libboringssl.a $(LDLIBSXX) $(LDLIBS_PROTOBUF) $(LDLIBS) $(GTEST_LIB) -o $(BINDIR)/$(CONFIG)/boringssl_tab_test

endif





# boringssl needs an override to ensure that it does not include
# system openssl headers regardless of other configuration
# we do so here with a target specific variable assignment
$(BORINGSSL_V3NAME_TEST_OBJS): CFLAGS := -Ithird_party/boringssl/include $(CFLAGS) -Wno-sign-conversion -Wno-conversion -Wno-unused-value
$(BORINGSSL_V3NAME_TEST_OBJS): CXXFLAGS := -Ithird_party/boringssl/include $(CXXFLAGS)
$(BORINGSSL_V3NAME_TEST_OBJS): CPPFLAGS += -DOPENSSL_NO_ASM -D_GNU_SOURCE


ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/boringssl_v3name_test: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/boringssl_v3name_test:  $(LIBDIR)/$(CONFIG)/libboringssl_v3name_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_test_util.a $(LIBDIR)/$(CONFIG)/libboringssl.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS)  $(LIBDIR)/$(CONFIG)/libboringssl_v3name_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_test_util.a $(LIBDIR)/$(CONFIG)/libboringssl.a $(LDLIBSXX) $(LDLIBS_PROTOBUF) $(LDLIBS) $(GTEST_LIB) -o $(BINDIR)/$(CONFIG)/boringssl_v3name_test

endif





# boringssl needs an override to ensure that it does not include
# system openssl headers regardless of other configuration
# we do so here with a target specific variable assignment
$(BORINGSSL_PQUEUE_TEST_OBJS): CFLAGS := -Ithird_party/boringssl/include $(CFLAGS) -Wno-sign-conversion -Wno-conversion -Wno-unused-value
$(BORINGSSL_PQUEUE_TEST_OBJS): CXXFLAGS := -Ithird_party/boringssl/include $(CXXFLAGS)
$(BORINGSSL_PQUEUE_TEST_OBJS): CPPFLAGS += -DOPENSSL_NO_ASM -D_GNU_SOURCE


ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/boringssl_pqueue_test: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/boringssl_pqueue_test:  $(LIBDIR)/$(CONFIG)/libboringssl_pqueue_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_test_util.a $(LIBDIR)/$(CONFIG)/libboringssl.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS)  $(LIBDIR)/$(CONFIG)/libboringssl_pqueue_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_test_util.a $(LIBDIR)/$(CONFIG)/libboringssl.a $(LDLIBSXX) $(LDLIBS_PROTOBUF) $(LDLIBS) $(GTEST_LIB) -o $(BINDIR)/$(CONFIG)/boringssl_pqueue_test

endif





# boringssl needs an override to ensure that it does not include
# system openssl headers regardless of other configuration
# we do so here with a target specific variable assignment
$(BORINGSSL_SSL_TEST_OBJS): CFLAGS := -Ithird_party/boringssl/include $(CFLAGS) -Wno-sign-conversion -Wno-conversion -Wno-unused-value
$(BORINGSSL_SSL_TEST_OBJS): CXXFLAGS := -Ithird_party/boringssl/include $(CXXFLAGS)
$(BORINGSSL_SSL_TEST_OBJS): CPPFLAGS += -DOPENSSL_NO_ASM -D_GNU_SOURCE


ifeq ($(NO_PROTOBUF),true)

# You can't build the protoc plugins or protobuf-enabled targets if you don't have protobuf 3.0.0+.

$(BINDIR)/$(CONFIG)/boringssl_ssl_test: protobuf_dep_error

else

$(BINDIR)/$(CONFIG)/boringssl_ssl_test:  $(LIBDIR)/$(CONFIG)/libboringssl_ssl_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_test_util.a $(LIBDIR)/$(CONFIG)/libboringssl.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LDXX) $(LDFLAGS)  $(LIBDIR)/$(CONFIG)/libboringssl_ssl_test_lib.a $(LIBDIR)/$(CONFIG)/libboringssl_test_util.a $(LIBDIR)/$(CONFIG)/libboringssl.a $(LDLIBSXX) $(LDLIBS_PROTOBUF) $(LDLIBS) $(GTEST_LIB) -o $(BINDIR)/$(CONFIG)/boringssl_ssl_test

endif




H2_CENSUS_TEST_SRC = \
    test/core/end2end/fixtures/h2_census.c \

H2_CENSUS_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(H2_CENSUS_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/h2_census_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/h2_census_test: $(H2_CENSUS_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libend2end_tests.a $(LIBDIR)/$(CONFIG)/libend2end_certs.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(H2_CENSUS_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libend2end_tests.a $(LIBDIR)/$(CONFIG)/libend2end_certs.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/h2_census_test

endif

$(OBJDIR)/$(CONFIG)/test/core/end2end/fixtures/h2_census.o:  $(LIBDIR)/$(CONFIG)/libend2end_tests.a $(LIBDIR)/$(CONFIG)/libend2end_certs.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_h2_census_test: $(H2_CENSUS_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(H2_CENSUS_TEST_OBJS:.o=.dep)
endif
endif


H2_COMPRESS_TEST_SRC = \
    test/core/end2end/fixtures/h2_compress.c \

H2_COMPRESS_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(H2_COMPRESS_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/h2_compress_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/h2_compress_test: $(H2_COMPRESS_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libend2end_tests.a $(LIBDIR)/$(CONFIG)/libend2end_certs.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(H2_COMPRESS_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libend2end_tests.a $(LIBDIR)/$(CONFIG)/libend2end_certs.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/h2_compress_test

endif

$(OBJDIR)/$(CONFIG)/test/core/end2end/fixtures/h2_compress.o:  $(LIBDIR)/$(CONFIG)/libend2end_tests.a $(LIBDIR)/$(CONFIG)/libend2end_certs.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_h2_compress_test: $(H2_COMPRESS_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(H2_COMPRESS_TEST_OBJS:.o=.dep)
endif
endif


H2_FAKESEC_TEST_SRC = \
    test/core/end2end/fixtures/h2_fakesec.c \

H2_FAKESEC_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(H2_FAKESEC_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/h2_fakesec_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/h2_fakesec_test: $(H2_FAKESEC_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libend2end_tests.a $(LIBDIR)/$(CONFIG)/libend2end_certs.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(H2_FAKESEC_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libend2end_tests.a $(LIBDIR)/$(CONFIG)/libend2end_certs.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/h2_fakesec_test

endif

$(OBJDIR)/$(CONFIG)/test/core/end2end/fixtures/h2_fakesec.o:  $(LIBDIR)/$(CONFIG)/libend2end_tests.a $(LIBDIR)/$(CONFIG)/libend2end_certs.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_h2_fakesec_test: $(H2_FAKESEC_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(H2_FAKESEC_TEST_OBJS:.o=.dep)
endif
endif


H2_FULL_TEST_SRC = \
    test/core/end2end/fixtures/h2_full.c \

H2_FULL_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(H2_FULL_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/h2_full_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/h2_full_test: $(H2_FULL_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libend2end_tests.a $(LIBDIR)/$(CONFIG)/libend2end_certs.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(H2_FULL_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libend2end_tests.a $(LIBDIR)/$(CONFIG)/libend2end_certs.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/h2_full_test

endif

$(OBJDIR)/$(CONFIG)/test/core/end2end/fixtures/h2_full.o:  $(LIBDIR)/$(CONFIG)/libend2end_tests.a $(LIBDIR)/$(CONFIG)/libend2end_certs.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_h2_full_test: $(H2_FULL_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(H2_FULL_TEST_OBJS:.o=.dep)
endif
endif


H2_FULL+PIPE_TEST_SRC = \
    test/core/end2end/fixtures/h2_full+pipe.c \

H2_FULL+PIPE_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(H2_FULL+PIPE_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/h2_full+pipe_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/h2_full+pipe_test: $(H2_FULL+PIPE_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libend2end_tests.a $(LIBDIR)/$(CONFIG)/libend2end_certs.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(H2_FULL+PIPE_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libend2end_tests.a $(LIBDIR)/$(CONFIG)/libend2end_certs.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/h2_full+pipe_test

endif

$(OBJDIR)/$(CONFIG)/test/core/end2end/fixtures/h2_full+pipe.o:  $(LIBDIR)/$(CONFIG)/libend2end_tests.a $(LIBDIR)/$(CONFIG)/libend2end_certs.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_h2_full+pipe_test: $(H2_FULL+PIPE_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(H2_FULL+PIPE_TEST_OBJS:.o=.dep)
endif
endif


H2_FULL+POLL_TEST_SRC = \
    test/core/end2end/fixtures/h2_full+poll.c \

H2_FULL+POLL_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(H2_FULL+POLL_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/h2_full+poll_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/h2_full+poll_test: $(H2_FULL+POLL_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libend2end_tests.a $(LIBDIR)/$(CONFIG)/libend2end_certs.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(H2_FULL+POLL_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libend2end_tests.a $(LIBDIR)/$(CONFIG)/libend2end_certs.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/h2_full+poll_test

endif

$(OBJDIR)/$(CONFIG)/test/core/end2end/fixtures/h2_full+poll.o:  $(LIBDIR)/$(CONFIG)/libend2end_tests.a $(LIBDIR)/$(CONFIG)/libend2end_certs.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_h2_full+poll_test: $(H2_FULL+POLL_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(H2_FULL+POLL_TEST_OBJS:.o=.dep)
endif
endif


H2_FULL+POLL+PIPE_TEST_SRC = \
    test/core/end2end/fixtures/h2_full+poll+pipe.c \

H2_FULL+POLL+PIPE_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(H2_FULL+POLL+PIPE_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/h2_full+poll+pipe_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/h2_full+poll+pipe_test: $(H2_FULL+POLL+PIPE_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libend2end_tests.a $(LIBDIR)/$(CONFIG)/libend2end_certs.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(H2_FULL+POLL+PIPE_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libend2end_tests.a $(LIBDIR)/$(CONFIG)/libend2end_certs.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/h2_full+poll+pipe_test

endif

$(OBJDIR)/$(CONFIG)/test/core/end2end/fixtures/h2_full+poll+pipe.o:  $(LIBDIR)/$(CONFIG)/libend2end_tests.a $(LIBDIR)/$(CONFIG)/libend2end_certs.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_h2_full+poll+pipe_test: $(H2_FULL+POLL+PIPE_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(H2_FULL+POLL+PIPE_TEST_OBJS:.o=.dep)
endif
endif


H2_OAUTH2_TEST_SRC = \
    test/core/end2end/fixtures/h2_oauth2.c \

H2_OAUTH2_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(H2_OAUTH2_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/h2_oauth2_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/h2_oauth2_test: $(H2_OAUTH2_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libend2end_tests.a $(LIBDIR)/$(CONFIG)/libend2end_certs.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(H2_OAUTH2_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libend2end_tests.a $(LIBDIR)/$(CONFIG)/libend2end_certs.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/h2_oauth2_test

endif

$(OBJDIR)/$(CONFIG)/test/core/end2end/fixtures/h2_oauth2.o:  $(LIBDIR)/$(CONFIG)/libend2end_tests.a $(LIBDIR)/$(CONFIG)/libend2end_certs.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_h2_oauth2_test: $(H2_OAUTH2_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(H2_OAUTH2_TEST_OBJS:.o=.dep)
endif
endif


H2_PROXY_TEST_SRC = \
    test/core/end2end/fixtures/h2_proxy.c \

H2_PROXY_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(H2_PROXY_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/h2_proxy_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/h2_proxy_test: $(H2_PROXY_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libend2end_tests.a $(LIBDIR)/$(CONFIG)/libend2end_certs.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(H2_PROXY_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libend2end_tests.a $(LIBDIR)/$(CONFIG)/libend2end_certs.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/h2_proxy_test

endif

$(OBJDIR)/$(CONFIG)/test/core/end2end/fixtures/h2_proxy.o:  $(LIBDIR)/$(CONFIG)/libend2end_tests.a $(LIBDIR)/$(CONFIG)/libend2end_certs.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_h2_proxy_test: $(H2_PROXY_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(H2_PROXY_TEST_OBJS:.o=.dep)
endif
endif


H2_SOCKPAIR_TEST_SRC = \
    test/core/end2end/fixtures/h2_sockpair.c \

H2_SOCKPAIR_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(H2_SOCKPAIR_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/h2_sockpair_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/h2_sockpair_test: $(H2_SOCKPAIR_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libend2end_tests.a $(LIBDIR)/$(CONFIG)/libend2end_certs.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(H2_SOCKPAIR_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libend2end_tests.a $(LIBDIR)/$(CONFIG)/libend2end_certs.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/h2_sockpair_test

endif

$(OBJDIR)/$(CONFIG)/test/core/end2end/fixtures/h2_sockpair.o:  $(LIBDIR)/$(CONFIG)/libend2end_tests.a $(LIBDIR)/$(CONFIG)/libend2end_certs.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_h2_sockpair_test: $(H2_SOCKPAIR_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(H2_SOCKPAIR_TEST_OBJS:.o=.dep)
endif
endif


H2_SOCKPAIR+TRACE_TEST_SRC = \
    test/core/end2end/fixtures/h2_sockpair+trace.c \

H2_SOCKPAIR+TRACE_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(H2_SOCKPAIR+TRACE_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/h2_sockpair+trace_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/h2_sockpair+trace_test: $(H2_SOCKPAIR+TRACE_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libend2end_tests.a $(LIBDIR)/$(CONFIG)/libend2end_certs.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(H2_SOCKPAIR+TRACE_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libend2end_tests.a $(LIBDIR)/$(CONFIG)/libend2end_certs.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/h2_sockpair+trace_test

endif

$(OBJDIR)/$(CONFIG)/test/core/end2end/fixtures/h2_sockpair+trace.o:  $(LIBDIR)/$(CONFIG)/libend2end_tests.a $(LIBDIR)/$(CONFIG)/libend2end_certs.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_h2_sockpair+trace_test: $(H2_SOCKPAIR+TRACE_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(H2_SOCKPAIR+TRACE_TEST_OBJS:.o=.dep)
endif
endif


H2_SOCKPAIR_1BYTE_TEST_SRC = \
    test/core/end2end/fixtures/h2_sockpair_1byte.c \

H2_SOCKPAIR_1BYTE_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(H2_SOCKPAIR_1BYTE_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/h2_sockpair_1byte_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/h2_sockpair_1byte_test: $(H2_SOCKPAIR_1BYTE_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libend2end_tests.a $(LIBDIR)/$(CONFIG)/libend2end_certs.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(H2_SOCKPAIR_1BYTE_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libend2end_tests.a $(LIBDIR)/$(CONFIG)/libend2end_certs.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/h2_sockpair_1byte_test

endif

$(OBJDIR)/$(CONFIG)/test/core/end2end/fixtures/h2_sockpair_1byte.o:  $(LIBDIR)/$(CONFIG)/libend2end_tests.a $(LIBDIR)/$(CONFIG)/libend2end_certs.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_h2_sockpair_1byte_test: $(H2_SOCKPAIR_1BYTE_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(H2_SOCKPAIR_1BYTE_TEST_OBJS:.o=.dep)
endif
endif


H2_SSL_TEST_SRC = \
    test/core/end2end/fixtures/h2_ssl.c \

H2_SSL_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(H2_SSL_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/h2_ssl_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/h2_ssl_test: $(H2_SSL_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libend2end_tests.a $(LIBDIR)/$(CONFIG)/libend2end_certs.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(H2_SSL_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libend2end_tests.a $(LIBDIR)/$(CONFIG)/libend2end_certs.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/h2_ssl_test

endif

$(OBJDIR)/$(CONFIG)/test/core/end2end/fixtures/h2_ssl.o:  $(LIBDIR)/$(CONFIG)/libend2end_tests.a $(LIBDIR)/$(CONFIG)/libend2end_certs.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_h2_ssl_test: $(H2_SSL_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(H2_SSL_TEST_OBJS:.o=.dep)
endif
endif


H2_SSL+POLL_TEST_SRC = \
    test/core/end2end/fixtures/h2_ssl+poll.c \

H2_SSL+POLL_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(H2_SSL+POLL_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/h2_ssl+poll_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/h2_ssl+poll_test: $(H2_SSL+POLL_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libend2end_tests.a $(LIBDIR)/$(CONFIG)/libend2end_certs.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(H2_SSL+POLL_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libend2end_tests.a $(LIBDIR)/$(CONFIG)/libend2end_certs.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/h2_ssl+poll_test

endif

$(OBJDIR)/$(CONFIG)/test/core/end2end/fixtures/h2_ssl+poll.o:  $(LIBDIR)/$(CONFIG)/libend2end_tests.a $(LIBDIR)/$(CONFIG)/libend2end_certs.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_h2_ssl+poll_test: $(H2_SSL+POLL_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(H2_SSL+POLL_TEST_OBJS:.o=.dep)
endif
endif


H2_SSL_PROXY_TEST_SRC = \
    test/core/end2end/fixtures/h2_ssl_proxy.c \

H2_SSL_PROXY_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(H2_SSL_PROXY_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/h2_ssl_proxy_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/h2_ssl_proxy_test: $(H2_SSL_PROXY_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libend2end_tests.a $(LIBDIR)/$(CONFIG)/libend2end_certs.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(H2_SSL_PROXY_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libend2end_tests.a $(LIBDIR)/$(CONFIG)/libend2end_certs.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/h2_ssl_proxy_test

endif

$(OBJDIR)/$(CONFIG)/test/core/end2end/fixtures/h2_ssl_proxy.o:  $(LIBDIR)/$(CONFIG)/libend2end_tests.a $(LIBDIR)/$(CONFIG)/libend2end_certs.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_h2_ssl_proxy_test: $(H2_SSL_PROXY_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(H2_SSL_PROXY_TEST_OBJS:.o=.dep)
endif
endif


H2_UCHANNEL_TEST_SRC = \
    test/core/end2end/fixtures/h2_uchannel.c \

H2_UCHANNEL_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(H2_UCHANNEL_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/h2_uchannel_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/h2_uchannel_test: $(H2_UCHANNEL_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libend2end_tests.a $(LIBDIR)/$(CONFIG)/libend2end_certs.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(H2_UCHANNEL_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libend2end_tests.a $(LIBDIR)/$(CONFIG)/libend2end_certs.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/h2_uchannel_test

endif

$(OBJDIR)/$(CONFIG)/test/core/end2end/fixtures/h2_uchannel.o:  $(LIBDIR)/$(CONFIG)/libend2end_tests.a $(LIBDIR)/$(CONFIG)/libend2end_certs.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_h2_uchannel_test: $(H2_UCHANNEL_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(H2_UCHANNEL_TEST_OBJS:.o=.dep)
endif
endif


H2_UDS_TEST_SRC = \
    test/core/end2end/fixtures/h2_uds.c \

H2_UDS_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(H2_UDS_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/h2_uds_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/h2_uds_test: $(H2_UDS_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libend2end_tests.a $(LIBDIR)/$(CONFIG)/libend2end_certs.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(H2_UDS_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libend2end_tests.a $(LIBDIR)/$(CONFIG)/libend2end_certs.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/h2_uds_test

endif

$(OBJDIR)/$(CONFIG)/test/core/end2end/fixtures/h2_uds.o:  $(LIBDIR)/$(CONFIG)/libend2end_tests.a $(LIBDIR)/$(CONFIG)/libend2end_certs.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_h2_uds_test: $(H2_UDS_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(H2_UDS_TEST_OBJS:.o=.dep)
endif
endif


H2_UDS+POLL_TEST_SRC = \
    test/core/end2end/fixtures/h2_uds+poll.c \

H2_UDS+POLL_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(H2_UDS+POLL_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/h2_uds+poll_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/h2_uds+poll_test: $(H2_UDS+POLL_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libend2end_tests.a $(LIBDIR)/$(CONFIG)/libend2end_certs.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(H2_UDS+POLL_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libend2end_tests.a $(LIBDIR)/$(CONFIG)/libend2end_certs.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/h2_uds+poll_test

endif

$(OBJDIR)/$(CONFIG)/test/core/end2end/fixtures/h2_uds+poll.o:  $(LIBDIR)/$(CONFIG)/libend2end_tests.a $(LIBDIR)/$(CONFIG)/libend2end_certs.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_h2_uds+poll_test: $(H2_UDS+POLL_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(H2_UDS+POLL_TEST_OBJS:.o=.dep)
endif
endif


H2_CENSUS_NOSEC_TEST_SRC = \
    test/core/end2end/fixtures/h2_census.c \

H2_CENSUS_NOSEC_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(H2_CENSUS_NOSEC_TEST_SRC))))


$(BINDIR)/$(CONFIG)/h2_census_nosec_test: $(H2_CENSUS_NOSEC_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libend2end_nosec_tests.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util_unsecure.a $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(H2_CENSUS_NOSEC_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libend2end_nosec_tests.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util_unsecure.a $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) -o $(BINDIR)/$(CONFIG)/h2_census_nosec_test

$(OBJDIR)/$(CONFIG)/test/core/end2end/fixtures/h2_census.o:  $(LIBDIR)/$(CONFIG)/libend2end_nosec_tests.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util_unsecure.a $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_h2_census_nosec_test: $(H2_CENSUS_NOSEC_TEST_OBJS:.o=.dep)

ifneq ($(NO_DEPS),true)
-include $(H2_CENSUS_NOSEC_TEST_OBJS:.o=.dep)
endif


H2_COMPRESS_NOSEC_TEST_SRC = \
    test/core/end2end/fixtures/h2_compress.c \

H2_COMPRESS_NOSEC_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(H2_COMPRESS_NOSEC_TEST_SRC))))


$(BINDIR)/$(CONFIG)/h2_compress_nosec_test: $(H2_COMPRESS_NOSEC_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libend2end_nosec_tests.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util_unsecure.a $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(H2_COMPRESS_NOSEC_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libend2end_nosec_tests.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util_unsecure.a $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) -o $(BINDIR)/$(CONFIG)/h2_compress_nosec_test

$(OBJDIR)/$(CONFIG)/test/core/end2end/fixtures/h2_compress.o:  $(LIBDIR)/$(CONFIG)/libend2end_nosec_tests.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util_unsecure.a $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_h2_compress_nosec_test: $(H2_COMPRESS_NOSEC_TEST_OBJS:.o=.dep)

ifneq ($(NO_DEPS),true)
-include $(H2_COMPRESS_NOSEC_TEST_OBJS:.o=.dep)
endif


H2_FULL_NOSEC_TEST_SRC = \
    test/core/end2end/fixtures/h2_full.c \

H2_FULL_NOSEC_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(H2_FULL_NOSEC_TEST_SRC))))


$(BINDIR)/$(CONFIG)/h2_full_nosec_test: $(H2_FULL_NOSEC_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libend2end_nosec_tests.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util_unsecure.a $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(H2_FULL_NOSEC_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libend2end_nosec_tests.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util_unsecure.a $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) -o $(BINDIR)/$(CONFIG)/h2_full_nosec_test

$(OBJDIR)/$(CONFIG)/test/core/end2end/fixtures/h2_full.o:  $(LIBDIR)/$(CONFIG)/libend2end_nosec_tests.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util_unsecure.a $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_h2_full_nosec_test: $(H2_FULL_NOSEC_TEST_OBJS:.o=.dep)

ifneq ($(NO_DEPS),true)
-include $(H2_FULL_NOSEC_TEST_OBJS:.o=.dep)
endif


H2_FULL+PIPE_NOSEC_TEST_SRC = \
    test/core/end2end/fixtures/h2_full+pipe.c \

H2_FULL+PIPE_NOSEC_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(H2_FULL+PIPE_NOSEC_TEST_SRC))))


$(BINDIR)/$(CONFIG)/h2_full+pipe_nosec_test: $(H2_FULL+PIPE_NOSEC_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libend2end_nosec_tests.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util_unsecure.a $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(H2_FULL+PIPE_NOSEC_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libend2end_nosec_tests.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util_unsecure.a $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) -o $(BINDIR)/$(CONFIG)/h2_full+pipe_nosec_test

$(OBJDIR)/$(CONFIG)/test/core/end2end/fixtures/h2_full+pipe.o:  $(LIBDIR)/$(CONFIG)/libend2end_nosec_tests.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util_unsecure.a $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_h2_full+pipe_nosec_test: $(H2_FULL+PIPE_NOSEC_TEST_OBJS:.o=.dep)

ifneq ($(NO_DEPS),true)
-include $(H2_FULL+PIPE_NOSEC_TEST_OBJS:.o=.dep)
endif


H2_FULL+POLL_NOSEC_TEST_SRC = \
    test/core/end2end/fixtures/h2_full+poll.c \

H2_FULL+POLL_NOSEC_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(H2_FULL+POLL_NOSEC_TEST_SRC))))


$(BINDIR)/$(CONFIG)/h2_full+poll_nosec_test: $(H2_FULL+POLL_NOSEC_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libend2end_nosec_tests.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util_unsecure.a $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(H2_FULL+POLL_NOSEC_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libend2end_nosec_tests.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util_unsecure.a $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) -o $(BINDIR)/$(CONFIG)/h2_full+poll_nosec_test

$(OBJDIR)/$(CONFIG)/test/core/end2end/fixtures/h2_full+poll.o:  $(LIBDIR)/$(CONFIG)/libend2end_nosec_tests.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util_unsecure.a $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_h2_full+poll_nosec_test: $(H2_FULL+POLL_NOSEC_TEST_OBJS:.o=.dep)

ifneq ($(NO_DEPS),true)
-include $(H2_FULL+POLL_NOSEC_TEST_OBJS:.o=.dep)
endif


H2_FULL+POLL+PIPE_NOSEC_TEST_SRC = \
    test/core/end2end/fixtures/h2_full+poll+pipe.c \

H2_FULL+POLL+PIPE_NOSEC_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(H2_FULL+POLL+PIPE_NOSEC_TEST_SRC))))


$(BINDIR)/$(CONFIG)/h2_full+poll+pipe_nosec_test: $(H2_FULL+POLL+PIPE_NOSEC_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libend2end_nosec_tests.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util_unsecure.a $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(H2_FULL+POLL+PIPE_NOSEC_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libend2end_nosec_tests.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util_unsecure.a $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) -o $(BINDIR)/$(CONFIG)/h2_full+poll+pipe_nosec_test

$(OBJDIR)/$(CONFIG)/test/core/end2end/fixtures/h2_full+poll+pipe.o:  $(LIBDIR)/$(CONFIG)/libend2end_nosec_tests.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util_unsecure.a $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_h2_full+poll+pipe_nosec_test: $(H2_FULL+POLL+PIPE_NOSEC_TEST_OBJS:.o=.dep)

ifneq ($(NO_DEPS),true)
-include $(H2_FULL+POLL+PIPE_NOSEC_TEST_OBJS:.o=.dep)
endif


H2_PROXY_NOSEC_TEST_SRC = \
    test/core/end2end/fixtures/h2_proxy.c \

H2_PROXY_NOSEC_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(H2_PROXY_NOSEC_TEST_SRC))))


$(BINDIR)/$(CONFIG)/h2_proxy_nosec_test: $(H2_PROXY_NOSEC_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libend2end_nosec_tests.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util_unsecure.a $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(H2_PROXY_NOSEC_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libend2end_nosec_tests.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util_unsecure.a $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) -o $(BINDIR)/$(CONFIG)/h2_proxy_nosec_test

$(OBJDIR)/$(CONFIG)/test/core/end2end/fixtures/h2_proxy.o:  $(LIBDIR)/$(CONFIG)/libend2end_nosec_tests.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util_unsecure.a $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_h2_proxy_nosec_test: $(H2_PROXY_NOSEC_TEST_OBJS:.o=.dep)

ifneq ($(NO_DEPS),true)
-include $(H2_PROXY_NOSEC_TEST_OBJS:.o=.dep)
endif


H2_SOCKPAIR_NOSEC_TEST_SRC = \
    test/core/end2end/fixtures/h2_sockpair.c \

H2_SOCKPAIR_NOSEC_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(H2_SOCKPAIR_NOSEC_TEST_SRC))))


$(BINDIR)/$(CONFIG)/h2_sockpair_nosec_test: $(H2_SOCKPAIR_NOSEC_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libend2end_nosec_tests.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util_unsecure.a $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(H2_SOCKPAIR_NOSEC_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libend2end_nosec_tests.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util_unsecure.a $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) -o $(BINDIR)/$(CONFIG)/h2_sockpair_nosec_test

$(OBJDIR)/$(CONFIG)/test/core/end2end/fixtures/h2_sockpair.o:  $(LIBDIR)/$(CONFIG)/libend2end_nosec_tests.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util_unsecure.a $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_h2_sockpair_nosec_test: $(H2_SOCKPAIR_NOSEC_TEST_OBJS:.o=.dep)

ifneq ($(NO_DEPS),true)
-include $(H2_SOCKPAIR_NOSEC_TEST_OBJS:.o=.dep)
endif


H2_SOCKPAIR+TRACE_NOSEC_TEST_SRC = \
    test/core/end2end/fixtures/h2_sockpair+trace.c \

H2_SOCKPAIR+TRACE_NOSEC_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(H2_SOCKPAIR+TRACE_NOSEC_TEST_SRC))))


$(BINDIR)/$(CONFIG)/h2_sockpair+trace_nosec_test: $(H2_SOCKPAIR+TRACE_NOSEC_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libend2end_nosec_tests.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util_unsecure.a $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(H2_SOCKPAIR+TRACE_NOSEC_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libend2end_nosec_tests.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util_unsecure.a $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) -o $(BINDIR)/$(CONFIG)/h2_sockpair+trace_nosec_test

$(OBJDIR)/$(CONFIG)/test/core/end2end/fixtures/h2_sockpair+trace.o:  $(LIBDIR)/$(CONFIG)/libend2end_nosec_tests.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util_unsecure.a $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_h2_sockpair+trace_nosec_test: $(H2_SOCKPAIR+TRACE_NOSEC_TEST_OBJS:.o=.dep)

ifneq ($(NO_DEPS),true)
-include $(H2_SOCKPAIR+TRACE_NOSEC_TEST_OBJS:.o=.dep)
endif


H2_SOCKPAIR_1BYTE_NOSEC_TEST_SRC = \
    test/core/end2end/fixtures/h2_sockpair_1byte.c \

H2_SOCKPAIR_1BYTE_NOSEC_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(H2_SOCKPAIR_1BYTE_NOSEC_TEST_SRC))))


$(BINDIR)/$(CONFIG)/h2_sockpair_1byte_nosec_test: $(H2_SOCKPAIR_1BYTE_NOSEC_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libend2end_nosec_tests.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util_unsecure.a $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(H2_SOCKPAIR_1BYTE_NOSEC_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libend2end_nosec_tests.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util_unsecure.a $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) -o $(BINDIR)/$(CONFIG)/h2_sockpair_1byte_nosec_test

$(OBJDIR)/$(CONFIG)/test/core/end2end/fixtures/h2_sockpair_1byte.o:  $(LIBDIR)/$(CONFIG)/libend2end_nosec_tests.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util_unsecure.a $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_h2_sockpair_1byte_nosec_test: $(H2_SOCKPAIR_1BYTE_NOSEC_TEST_OBJS:.o=.dep)

ifneq ($(NO_DEPS),true)
-include $(H2_SOCKPAIR_1BYTE_NOSEC_TEST_OBJS:.o=.dep)
endif


H2_UCHANNEL_NOSEC_TEST_SRC = \
    test/core/end2end/fixtures/h2_uchannel.c \

H2_UCHANNEL_NOSEC_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(H2_UCHANNEL_NOSEC_TEST_SRC))))


$(BINDIR)/$(CONFIG)/h2_uchannel_nosec_test: $(H2_UCHANNEL_NOSEC_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libend2end_nosec_tests.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util_unsecure.a $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(H2_UCHANNEL_NOSEC_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libend2end_nosec_tests.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util_unsecure.a $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) -o $(BINDIR)/$(CONFIG)/h2_uchannel_nosec_test

$(OBJDIR)/$(CONFIG)/test/core/end2end/fixtures/h2_uchannel.o:  $(LIBDIR)/$(CONFIG)/libend2end_nosec_tests.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util_unsecure.a $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_h2_uchannel_nosec_test: $(H2_UCHANNEL_NOSEC_TEST_OBJS:.o=.dep)

ifneq ($(NO_DEPS),true)
-include $(H2_UCHANNEL_NOSEC_TEST_OBJS:.o=.dep)
endif


H2_UDS_NOSEC_TEST_SRC = \
    test/core/end2end/fixtures/h2_uds.c \

H2_UDS_NOSEC_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(H2_UDS_NOSEC_TEST_SRC))))


$(BINDIR)/$(CONFIG)/h2_uds_nosec_test: $(H2_UDS_NOSEC_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libend2end_nosec_tests.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util_unsecure.a $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(H2_UDS_NOSEC_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libend2end_nosec_tests.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util_unsecure.a $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) -o $(BINDIR)/$(CONFIG)/h2_uds_nosec_test

$(OBJDIR)/$(CONFIG)/test/core/end2end/fixtures/h2_uds.o:  $(LIBDIR)/$(CONFIG)/libend2end_nosec_tests.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util_unsecure.a $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_h2_uds_nosec_test: $(H2_UDS_NOSEC_TEST_OBJS:.o=.dep)

ifneq ($(NO_DEPS),true)
-include $(H2_UDS_NOSEC_TEST_OBJS:.o=.dep)
endif


H2_UDS+POLL_NOSEC_TEST_SRC = \
    test/core/end2end/fixtures/h2_uds+poll.c \

H2_UDS+POLL_NOSEC_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(H2_UDS+POLL_NOSEC_TEST_SRC))))


$(BINDIR)/$(CONFIG)/h2_uds+poll_nosec_test: $(H2_UDS+POLL_NOSEC_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libend2end_nosec_tests.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util_unsecure.a $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(H2_UDS+POLL_NOSEC_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libend2end_nosec_tests.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util_unsecure.a $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) -o $(BINDIR)/$(CONFIG)/h2_uds+poll_nosec_test

$(OBJDIR)/$(CONFIG)/test/core/end2end/fixtures/h2_uds+poll.o:  $(LIBDIR)/$(CONFIG)/libend2end_nosec_tests.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util_unsecure.a $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_h2_uds+poll_nosec_test: $(H2_UDS+POLL_NOSEC_TEST_OBJS:.o=.dep)

ifneq ($(NO_DEPS),true)
-include $(H2_UDS+POLL_NOSEC_TEST_OBJS:.o=.dep)
endif


BADREQ_BAD_CLIENT_TEST_SRC = \
    test/core/bad_client/tests/badreq.c \

BADREQ_BAD_CLIENT_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(BADREQ_BAD_CLIENT_TEST_SRC))))


$(BINDIR)/$(CONFIG)/badreq_bad_client_test: $(BADREQ_BAD_CLIENT_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libbad_client_test.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util_unsecure.a $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(BADREQ_BAD_CLIENT_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libbad_client_test.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util_unsecure.a $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) -o $(BINDIR)/$(CONFIG)/badreq_bad_client_test

$(OBJDIR)/$(CONFIG)/test/core/bad_client/tests/badreq.o:  $(LIBDIR)/$(CONFIG)/libbad_client_test.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util_unsecure.a $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_badreq_bad_client_test: $(BADREQ_BAD_CLIENT_TEST_OBJS:.o=.dep)

ifneq ($(NO_DEPS),true)
-include $(BADREQ_BAD_CLIENT_TEST_OBJS:.o=.dep)
endif


CONNECTION_PREFIX_BAD_CLIENT_TEST_SRC = \
    test/core/bad_client/tests/connection_prefix.c \

CONNECTION_PREFIX_BAD_CLIENT_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(CONNECTION_PREFIX_BAD_CLIENT_TEST_SRC))))


$(BINDIR)/$(CONFIG)/connection_prefix_bad_client_test: $(CONNECTION_PREFIX_BAD_CLIENT_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libbad_client_test.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util_unsecure.a $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(CONNECTION_PREFIX_BAD_CLIENT_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libbad_client_test.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util_unsecure.a $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) -o $(BINDIR)/$(CONFIG)/connection_prefix_bad_client_test

$(OBJDIR)/$(CONFIG)/test/core/bad_client/tests/connection_prefix.o:  $(LIBDIR)/$(CONFIG)/libbad_client_test.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util_unsecure.a $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_connection_prefix_bad_client_test: $(CONNECTION_PREFIX_BAD_CLIENT_TEST_OBJS:.o=.dep)

ifneq ($(NO_DEPS),true)
-include $(CONNECTION_PREFIX_BAD_CLIENT_TEST_OBJS:.o=.dep)
endif


HEADERS_BAD_CLIENT_TEST_SRC = \
    test/core/bad_client/tests/headers.c \

HEADERS_BAD_CLIENT_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(HEADERS_BAD_CLIENT_TEST_SRC))))


$(BINDIR)/$(CONFIG)/headers_bad_client_test: $(HEADERS_BAD_CLIENT_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libbad_client_test.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util_unsecure.a $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(HEADERS_BAD_CLIENT_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libbad_client_test.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util_unsecure.a $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) -o $(BINDIR)/$(CONFIG)/headers_bad_client_test

$(OBJDIR)/$(CONFIG)/test/core/bad_client/tests/headers.o:  $(LIBDIR)/$(CONFIG)/libbad_client_test.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util_unsecure.a $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_headers_bad_client_test: $(HEADERS_BAD_CLIENT_TEST_OBJS:.o=.dep)

ifneq ($(NO_DEPS),true)
-include $(HEADERS_BAD_CLIENT_TEST_OBJS:.o=.dep)
endif


INITIAL_SETTINGS_FRAME_BAD_CLIENT_TEST_SRC = \
    test/core/bad_client/tests/initial_settings_frame.c \

INITIAL_SETTINGS_FRAME_BAD_CLIENT_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(INITIAL_SETTINGS_FRAME_BAD_CLIENT_TEST_SRC))))


$(BINDIR)/$(CONFIG)/initial_settings_frame_bad_client_test: $(INITIAL_SETTINGS_FRAME_BAD_CLIENT_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libbad_client_test.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util_unsecure.a $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(INITIAL_SETTINGS_FRAME_BAD_CLIENT_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libbad_client_test.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util_unsecure.a $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) -o $(BINDIR)/$(CONFIG)/initial_settings_frame_bad_client_test

$(OBJDIR)/$(CONFIG)/test/core/bad_client/tests/initial_settings_frame.o:  $(LIBDIR)/$(CONFIG)/libbad_client_test.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util_unsecure.a $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_initial_settings_frame_bad_client_test: $(INITIAL_SETTINGS_FRAME_BAD_CLIENT_TEST_OBJS:.o=.dep)

ifneq ($(NO_DEPS),true)
-include $(INITIAL_SETTINGS_FRAME_BAD_CLIENT_TEST_OBJS:.o=.dep)
endif


SERVER_REGISTERED_METHOD_BAD_CLIENT_TEST_SRC = \
    test/core/bad_client/tests/server_registered_method.c \

SERVER_REGISTERED_METHOD_BAD_CLIENT_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(SERVER_REGISTERED_METHOD_BAD_CLIENT_TEST_SRC))))


$(BINDIR)/$(CONFIG)/server_registered_method_bad_client_test: $(SERVER_REGISTERED_METHOD_BAD_CLIENT_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libbad_client_test.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util_unsecure.a $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(SERVER_REGISTERED_METHOD_BAD_CLIENT_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libbad_client_test.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util_unsecure.a $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) -o $(BINDIR)/$(CONFIG)/server_registered_method_bad_client_test

$(OBJDIR)/$(CONFIG)/test/core/bad_client/tests/server_registered_method.o:  $(LIBDIR)/$(CONFIG)/libbad_client_test.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util_unsecure.a $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_server_registered_method_bad_client_test: $(SERVER_REGISTERED_METHOD_BAD_CLIENT_TEST_OBJS:.o=.dep)

ifneq ($(NO_DEPS),true)
-include $(SERVER_REGISTERED_METHOD_BAD_CLIENT_TEST_OBJS:.o=.dep)
endif


SIMPLE_REQUEST_BAD_CLIENT_TEST_SRC = \
    test/core/bad_client/tests/simple_request.c \

SIMPLE_REQUEST_BAD_CLIENT_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(SIMPLE_REQUEST_BAD_CLIENT_TEST_SRC))))


$(BINDIR)/$(CONFIG)/simple_request_bad_client_test: $(SIMPLE_REQUEST_BAD_CLIENT_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libbad_client_test.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util_unsecure.a $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(SIMPLE_REQUEST_BAD_CLIENT_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libbad_client_test.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util_unsecure.a $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) -o $(BINDIR)/$(CONFIG)/simple_request_bad_client_test

$(OBJDIR)/$(CONFIG)/test/core/bad_client/tests/simple_request.o:  $(LIBDIR)/$(CONFIG)/libbad_client_test.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util_unsecure.a $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_simple_request_bad_client_test: $(SIMPLE_REQUEST_BAD_CLIENT_TEST_OBJS:.o=.dep)

ifneq ($(NO_DEPS),true)
-include $(SIMPLE_REQUEST_BAD_CLIENT_TEST_OBJS:.o=.dep)
endif


UNKNOWN_FRAME_BAD_CLIENT_TEST_SRC = \
    test/core/bad_client/tests/unknown_frame.c \

UNKNOWN_FRAME_BAD_CLIENT_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(UNKNOWN_FRAME_BAD_CLIENT_TEST_SRC))))


$(BINDIR)/$(CONFIG)/unknown_frame_bad_client_test: $(UNKNOWN_FRAME_BAD_CLIENT_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libbad_client_test.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util_unsecure.a $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(UNKNOWN_FRAME_BAD_CLIENT_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libbad_client_test.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util_unsecure.a $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) -o $(BINDIR)/$(CONFIG)/unknown_frame_bad_client_test

$(OBJDIR)/$(CONFIG)/test/core/bad_client/tests/unknown_frame.o:  $(LIBDIR)/$(CONFIG)/libbad_client_test.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util_unsecure.a $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_unknown_frame_bad_client_test: $(UNKNOWN_FRAME_BAD_CLIENT_TEST_OBJS:.o=.dep)

ifneq ($(NO_DEPS),true)
-include $(UNKNOWN_FRAME_BAD_CLIENT_TEST_OBJS:.o=.dep)
endif


WINDOW_OVERFLOW_BAD_CLIENT_TEST_SRC = \
    test/core/bad_client/tests/window_overflow.c \

WINDOW_OVERFLOW_BAD_CLIENT_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(WINDOW_OVERFLOW_BAD_CLIENT_TEST_SRC))))


$(BINDIR)/$(CONFIG)/window_overflow_bad_client_test: $(WINDOW_OVERFLOW_BAD_CLIENT_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libbad_client_test.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util_unsecure.a $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(WINDOW_OVERFLOW_BAD_CLIENT_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libbad_client_test.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util_unsecure.a $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) -o $(BINDIR)/$(CONFIG)/window_overflow_bad_client_test

$(OBJDIR)/$(CONFIG)/test/core/bad_client/tests/window_overflow.o:  $(LIBDIR)/$(CONFIG)/libbad_client_test.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util_unsecure.a $(LIBDIR)/$(CONFIG)/libgrpc_unsecure.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_window_overflow_bad_client_test: $(WINDOW_OVERFLOW_BAD_CLIENT_TEST_OBJS:.o=.dep)

ifneq ($(NO_DEPS),true)
-include $(WINDOW_OVERFLOW_BAD_CLIENT_TEST_OBJS:.o=.dep)
endif


BAD_SSL_ALPN_SERVER_SRC = \
    test/core/bad_ssl/servers/alpn.c \

BAD_SSL_ALPN_SERVER_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(BAD_SSL_ALPN_SERVER_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/bad_ssl_alpn_server: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/bad_ssl_alpn_server: $(BAD_SSL_ALPN_SERVER_OBJS) $(LIBDIR)/$(CONFIG)/libbad_ssl_test_server.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(BAD_SSL_ALPN_SERVER_OBJS) $(LIBDIR)/$(CONFIG)/libbad_ssl_test_server.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/bad_ssl_alpn_server

endif

$(OBJDIR)/$(CONFIG)/test/core/bad_ssl/servers/alpn.o:  $(LIBDIR)/$(CONFIG)/libbad_ssl_test_server.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_bad_ssl_alpn_server: $(BAD_SSL_ALPN_SERVER_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(BAD_SSL_ALPN_SERVER_OBJS:.o=.dep)
endif
endif


BAD_SSL_CERT_SERVER_SRC = \
    test/core/bad_ssl/servers/cert.c \

BAD_SSL_CERT_SERVER_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(BAD_SSL_CERT_SERVER_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/bad_ssl_cert_server: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/bad_ssl_cert_server: $(BAD_SSL_CERT_SERVER_OBJS) $(LIBDIR)/$(CONFIG)/libbad_ssl_test_server.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(BAD_SSL_CERT_SERVER_OBJS) $(LIBDIR)/$(CONFIG)/libbad_ssl_test_server.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/bad_ssl_cert_server

endif

$(OBJDIR)/$(CONFIG)/test/core/bad_ssl/servers/cert.o:  $(LIBDIR)/$(CONFIG)/libbad_ssl_test_server.a $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_bad_ssl_cert_server: $(BAD_SSL_CERT_SERVER_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(BAD_SSL_CERT_SERVER_OBJS:.o=.dep)
endif
endif


BAD_SSL_ALPN_TEST_SRC = \
    test/core/bad_ssl/bad_ssl_test.c \

BAD_SSL_ALPN_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(BAD_SSL_ALPN_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/bad_ssl_alpn_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/bad_ssl_alpn_test: $(BAD_SSL_ALPN_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(BAD_SSL_ALPN_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/bad_ssl_alpn_test

endif

$(OBJDIR)/$(CONFIG)/test/core/bad_ssl/bad_ssl_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_bad_ssl_alpn_test: $(BAD_SSL_ALPN_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(BAD_SSL_ALPN_TEST_OBJS:.o=.dep)
endif
endif


BAD_SSL_CERT_TEST_SRC = \
    test/core/bad_ssl/bad_ssl_test.c \

BAD_SSL_CERT_TEST_OBJS = $(addprefix $(OBJDIR)/$(CONFIG)/, $(addsuffix .o, $(basename $(BAD_SSL_CERT_TEST_SRC))))
ifeq ($(NO_SECURE),true)

# You can't build secure targets if you don't have OpenSSL.

$(BINDIR)/$(CONFIG)/bad_ssl_cert_test: openssl_dep_error

else



$(BINDIR)/$(CONFIG)/bad_ssl_cert_test: $(BAD_SSL_CERT_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
	$(E) "[LD]      Linking $@"
	$(Q) mkdir -p `dirname $@`
	$(Q) $(LD) $(LDFLAGS) $(BAD_SSL_CERT_TEST_OBJS) $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a $(LDLIBS) $(LDLIBS_SECURE) -o $(BINDIR)/$(CONFIG)/bad_ssl_cert_test

endif

$(OBJDIR)/$(CONFIG)/test/core/bad_ssl/bad_ssl_test.o:  $(LIBDIR)/$(CONFIG)/libgrpc_test_util.a $(LIBDIR)/$(CONFIG)/libgrpc.a $(LIBDIR)/$(CONFIG)/libgpr_test_util.a $(LIBDIR)/$(CONFIG)/libgpr.a
deps_bad_ssl_cert_test: $(BAD_SSL_CERT_TEST_OBJS:.o=.dep)

ifneq ($(NO_SECURE),true)
ifneq ($(NO_DEPS),true)
-include $(BAD_SSL_CERT_TEST_OBJS:.o=.dep)
endif
endif






ifneq ($(OPENSSL_DEP),)
# This is to ensure the embedded OpenSSL is built beforehand, properly
# installing headers to their final destination on the drive. We need this
# otherwise parallel compilation will fail if a source is compiled first.
src/core/httpcli/httpcli_security_connector.c: $(OPENSSL_DEP)
src/core/security/base64.c: $(OPENSSL_DEP)
src/core/security/client_auth_filter.c: $(OPENSSL_DEP)
src/core/security/credentials.c: $(OPENSSL_DEP)
src/core/security/credentials_metadata.c: $(OPENSSL_DEP)
src/core/security/credentials_posix.c: $(OPENSSL_DEP)
src/core/security/credentials_win32.c: $(OPENSSL_DEP)
src/core/security/google_default_credentials.c: $(OPENSSL_DEP)
src/core/security/handshake.c: $(OPENSSL_DEP)
src/core/security/json_token.c: $(OPENSSL_DEP)
src/core/security/jwt_verifier.c: $(OPENSSL_DEP)
src/core/security/secure_endpoint.c: $(OPENSSL_DEP)
src/core/security/security_connector.c: $(OPENSSL_DEP)
src/core/security/security_context.c: $(OPENSSL_DEP)
src/core/security/server_auth_filter.c: $(OPENSSL_DEP)
src/core/security/server_secure_chttp2.c: $(OPENSSL_DEP)
src/core/surface/init_secure.c: $(OPENSSL_DEP)
src/core/surface/secure_channel_create.c: $(OPENSSL_DEP)
src/core/tsi/fake_transport_security.c: $(OPENSSL_DEP)
src/core/tsi/ssl_transport_security.c: $(OPENSSL_DEP)
src/core/tsi/transport_security.c: $(OPENSSL_DEP)
src/cpp/client/secure_credentials.cc: $(OPENSSL_DEP)
src/cpp/common/auth_property_iterator.cc: $(OPENSSL_DEP)
src/cpp/common/secure_auth_context.cc: $(OPENSSL_DEP)
src/cpp/common/secure_channel_arguments.cc: $(OPENSSL_DEP)
src/cpp/common/secure_create_auth_context.cc: $(OPENSSL_DEP)
src/cpp/server/secure_server_credentials.cc: $(OPENSSL_DEP)
src/csharp/ext/grpc_csharp_ext.c: $(OPENSSL_DEP)
test/core/bad_client/bad_client.c: $(OPENSSL_DEP)
test/core/bad_ssl/server.c: $(OPENSSL_DEP)
test/core/end2end/data/server1_cert.c: $(OPENSSL_DEP)
test/core/end2end/data/server1_key.c: $(OPENSSL_DEP)
test/core/end2end/data/test_root_cert.c: $(OPENSSL_DEP)
test/core/end2end/end2end_tests.c: $(OPENSSL_DEP)
test/core/end2end/tests/call_creds.c: $(OPENSSL_DEP)
test/core/security/oauth2_utils.c: $(OPENSSL_DEP)
test/core/util/reconnect_server.c: $(OPENSSL_DEP)
test/core/util/test_tcp_server.c: $(OPENSSL_DEP)
test/cpp/interop/client.cc: $(OPENSSL_DEP)
test/cpp/interop/client_helper.cc: $(OPENSSL_DEP)
test/cpp/interop/interop_client.cc: $(OPENSSL_DEP)
test/cpp/interop/server.cc: $(OPENSSL_DEP)
test/cpp/interop/server_helper.cc: $(OPENSSL_DEP)
test/cpp/qps/client_async.cc: $(OPENSSL_DEP)
test/cpp/qps/client_sync.cc: $(OPENSSL_DEP)
test/cpp/qps/driver.cc: $(OPENSSL_DEP)
test/cpp/qps/perf_db_client.cc: $(OPENSSL_DEP)
test/cpp/qps/qps_worker.cc: $(OPENSSL_DEP)
test/cpp/qps/report.cc: $(OPENSSL_DEP)
test/cpp/qps/server_async.cc: $(OPENSSL_DEP)
test/cpp/qps/server_sync.cc: $(OPENSSL_DEP)
test/cpp/qps/timer.cc: $(OPENSSL_DEP)
test/cpp/util/benchmark_config.cc: $(OPENSSL_DEP)
test/cpp/util/cli_call.cc: $(OPENSSL_DEP)
test/cpp/util/create_test_channel.cc: $(OPENSSL_DEP)
test/cpp/util/string_ref_helper.cc: $(OPENSSL_DEP)
test/cpp/util/subprocess.cc: $(OPENSSL_DEP)
test/cpp/util/test_config.cc: $(OPENSSL_DEP)
endif

.PHONY: all strip tools dep_error openssl_dep_error openssl_dep_message git_update stop buildtests buildtests_c buildtests_cxx test test_c test_cxx install install_c install_cxx install-headers install-headers_c install-headers_cxx install-shared install-shared_c install-shared_cxx install-static install-static_c install-static_cxx strip strip-shared strip-static strip_c strip-shared_c strip-static_c strip_cxx strip-shared_cxx strip-static_cxx dep_c dep_cxx bins_dep_c bins_dep_cxx clean