#!/bin/sh
set -eu

if [ "$(uname -s)" != "Darwin" ]; then
	echo "This gate is configured for the native macOS Plex development host." >&2
	exit 2
fi

: "${CARGO_TARGET_DIR:=/private/tmp/plex-cadrum-native-sanitizers}"
: "${OCCT_ROOT:=$(pwd)/target/occt-8_0_0_rev5-aarch64_apple_darwin}"
export CARGO_TARGET_DIR
export OCCT_ROOT
export CADRUM_NATIVE_SANITIZERS=address,undefined
export ASAN_OPTIONS=detect_leaks=0:halt_on_error=1
export UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1

cargo test --test hardening --test cancellation --test concurrency --test identity_sweeps
