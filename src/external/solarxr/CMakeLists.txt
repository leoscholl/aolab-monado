# Copyright 2024, rcelyte
# SPDX-License-Identifier: BSL-1.0

include(FetchContent)

set(FLATCC_TEST
    OFF
    CACHE INTERNAL ""
	)
set(FLATCC_DEBUG_CLANG_SANITIZE
    OFF
    CACHE INTERNAL ""
	)
set(FLATCC_REFLECTION
    OFF
    CACHE INTERNAL ""
	)
set(FLATCC_ALLOW_WERROR
    OFF
    CACHE INTERNAL ""
	)
set(CMAKE_POLICY_DEFAULT_CMP0048 NEW)
FetchContent_Declare(
	FlatCC
	DOWNLOAD_EXTRACT_TIMESTAMP true
	URL_HASH SHA256=2533c2f1061498499f15acc7e0937dcf35bc68e685d237325124ae0d6c600c2b
	URL https://github.com/dvidelabs/flatcc/archive/refs/tags/v0.6.1.tar.gz
	)
FetchContent_MakeAvailable(FlatCC)

add_custom_command(
	OUTPUT solarxr_protocol.h
	COMMAND
		flatcc_cli -a --outfile=solarxr_protocol.h
		${CMAKE_CURRENT_SOURCE_DIR}/solarxr_protocol.fbs
	)

add_library(solarxr_protocol STATIC solarxr_protocol.h)
set_target_properties(solarxr_protocol PROPERTIES LINKER_LANGUAGE C)
target_include_directories(solarxr_protocol SYSTEM INTERFACE ${CMAKE_CURRENT_BINARY_DIR} ${FlatCC_SOURCE_DIR}/include)
target_link_libraries(solarxr_protocol INTERFACE flatccrt)
