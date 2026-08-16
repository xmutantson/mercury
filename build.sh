#!/bin/bash
# Mercury build script (cross-platform: MSYS2/MinGW64, Linux, macOS)
# Usage: ./build.sh [mode] [clean]
#   Modes: release (default), debug, asan, ubsan, o0, o1, o2, o3
#   clean: removes all object files first
#
# Examples:
#   ./build.sh              # Release build (-O3)
#   ./build.sh debug        # Debug build (-O3 -g)
#   ./build.sh asan         # AddressSanitizer build (-O1 -fsanitize=address)
#   ./build.sh o0           # No optimization (stable, for crash investigation)
#   ./build.sh clean        # Clean only
#   ./build.sh debug clean  # Clean then debug build

set -e
cd "$(dirname "$0")"

MODE="${1:-release}"
CLEAN=""

# Parse arguments
for arg in "$@"; do
    case "$arg" in
        clean) CLEAN=1 ;;
        release|debug|asan|ubsan|o0|o1|o2|o3) MODE="$arg" ;;
    esac
done

# Build directory (separate per mode to avoid cross-mode stale objects)
BUILDDIR="build/${MODE}"
mkdir -p "$BUILDDIR"

# Clean function
do_clean() {
    echo "Cleaning..."
    rm -f mercury mercury.exe mercury_*.exe
    rm -rf build/
    echo "Clean done."
}

if [ "$CLEAN" = "1" ] && [ "$MODE" = "clean" ]; then
    do_clean
    exit 0
fi

if [ "$CLEAN" = "1" ]; then
    do_clean
    mkdir -p "$BUILDDIR"
fi

# Set optimization flags based on mode
case "$MODE" in
    release|o3)
        OPT="-O3"
        DBG="-g"
        SUFFIX=""
        EXTRA_CFLAGS=""
        EXTRA_LDFLAGS=""
        ;;
    debug)
        OPT="-O0"
        DBG="-g"
        SUFFIX="_debug"
        EXTRA_CFLAGS=""
        EXTRA_LDFLAGS=""
        ;;
    asan)
        OPT="-O1"
        DBG="-g -fsanitize=address -fno-omit-frame-pointer"
        SUFFIX="_asan"
        EXTRA_CFLAGS=""
        EXTRA_LDFLAGS="-fsanitize=address"
        ;;
    ubsan)
        OPT="-O3"
        DBG="-g -fsanitize=undefined -fsanitize-undefined-trap-on-error -fno-omit-frame-pointer"
        SUFFIX="_ubsan"
        EXTRA_CFLAGS=""
        EXTRA_LDFLAGS=""
        ;;
    o0)
        OPT="-O0"
        DBG=""
        SUFFIX="_o0"
        EXTRA_CFLAGS=""
        EXTRA_LDFLAGS=""
        ;;
    o1)
        OPT="-O1"
        DBG="-g"
        SUFFIX="_o1"
        EXTRA_CFLAGS=""
        EXTRA_LDFLAGS=""
        ;;
    o2)
        OPT="-O2"
        DBG="-g"
        SUFFIX="_o2"
        EXTRA_CFLAGS=""
        EXTRA_LDFLAGS=""
        ;;
    *)
        echo "Unknown mode: $MODE"
        echo "Usage: $0 [release|debug|asan|o0|o1|o2|o3] [clean]"
        exit 1
        ;;
esac

echo "=== Building Mercury ($MODE) ==="

# Dependency check (Linux/macOS only — Windows uses vendored libs)
check_deps() {
    local missing=""
    if ! command -v g++ &>/dev/null; then missing="$missing g++"; fi
    if ! command -v gcc &>/dev/null; then missing="$missing gcc"; fi
    if ! command -v pkg-config &>/dev/null; then missing="$missing pkg-config"; fi
    if ! pkg-config --exists glfw3 2>/dev/null; then missing="$missing glfw3"; fi

    # Check for audio headers
    if [ "$1" = "linux" ]; then
        if ! pkg-config --exists libpulse 2>/dev/null; then missing="$missing libpulse"; fi
        if ! pkg-config --exists alsa 2>/dev/null; then missing="$missing alsa"; fi
    fi

    if [ -n "$missing" ]; then
        echo ""
        echo "ERROR: Missing dependencies:$missing"
        echo ""
        # Detect distro and suggest install command
        if [ -f /etc/arch-release ]; then
            echo "  sudo pacman -S base-devel glfw-x11 libpulse alsa-lib pkg-config"
        elif [ -f /etc/debian_version ]; then
            echo "  sudo apt install build-essential libglfw3-dev libpulse-dev libasound2-dev pkg-config"
        elif [ -f /etc/fedora-release ]; then
            echo "  sudo dnf install gcc gcc-c++ glfw-devel pulseaudio-libs-devel alsa-lib-devel pkg-config"
        elif command -v brew &>/dev/null; then
            echo "  brew install glfw pkg-config"
        else
            echo "  Install: g++ gcc pkg-config glfw3-dev libpulse-dev libasound-dev"
        fi
        echo ""
        exit 1
    fi
}

# Compiler settings (overridable via env for cross-compilation)
CXX="${CXX:-g++}"
CC="${CC:-gcc}"
# Optional: IDLE_GATE_TRACE=1 ./build.sh o3 — enables the idle-scan cadence
# Step-0 instrumentation (idle-loop FIR-run rate + raw-passband RMS distribution,
# diagnostic only, behind #ifdef IDLE_GATE_TRACE). Without the env var the
# binary is byte-identical to baseline.
TRACE_CFLAGS=""
if [ "${IDLE_GATE_TRACE:-0}" = "1" ]; then
    TRACE_CFLAGS="$TRACE_CFLAGS -DIDLE_GATE_TRACE"
    echo "  (IDLE_GATE_TRACE instrumentation ENABLED)"
fi

# --- Deterministic git build-id baked into the binary banner ---------------
# Writes include/common/build_id.h with #define MERCURY_BUILD_ID "<shortrev>[-dirty]".
# This makes the on-Pi banner human-readable (git identity, not just an md5) AND keeps
# the build DETERMINISTIC: SAME SOURCE -> SAME id -> SAME md5 on every Pi. We bake ONLY
# the short rev + a dirty flag — NO build time, NO host name (embedding those is exactly
# what breaks reproducible-md5 across the two Pis / --build-on-both).
#
# Resolution order (first that yields a non-empty id wins):
#   1. $MERCURY_BUILD_ID  — caller-supplied (the deploy script computes it ONCE on the
#      host from the real source worktree and forwards it; the on-Pi tarball has no .git,
#      so this is how both Pis bake the SAME id for the same commit).
#   2. git rev-parse in this source tree (native host builds with a .git present).
#   3. "nogit" fallback — keeps the id (and thus the md5) deterministic when neithe
#      the env nor a .git is available.
BUILD_ID="${MERCURY_BUILD_ID:-}"
if [ -z "$BUILD_ID" ]; then
    if command -v git &>/dev/null && git -C . rev-parse --git-dir &>/dev/null; then
        _gid_rev="$(git -C . rev-parse --short HEAD 2>/dev/null)"
        if [ -n "$_gid_rev" ]; then
            if [ -n "$(git -C . status --porcelain 2>/dev/null)" ]; then
                BUILD_ID="${_gid_rev}-dirty"
            else
                BUILD_ID="${_gid_rev}"
            fi
        fi
    fi
fi
[ -z "$BUILD_ID" ] && BUILD_ID="nogit"
_BUILD_ID_HEADER="include/common/build_id.h"
_BUILD_ID_CONTENT="// Auto-generated by build.sh — DO NOT EDIT, DO NOT COMMIT.
#ifndef MERCURY_BUILD_ID
#define MERCURY_BUILD_ID \"${BUILD_ID}\"
#endif"
# Only rewrite the header when the id changes, so an unchanged id does not bump the
# header mtime and force a needless full recompile of everything that includes it.
if [ ! -f "$_BUILD_ID_HEADER" ] || [ "$(cat "$_BUILD_ID_HEADER" 2>/dev/null)" != "$_BUILD_ID_CONTENT" ]; then
    printf '%s\n' "$_BUILD_ID_CONTENT" > "$_BUILD_ID_HEADER"
fi
echo "  (build id: $BUILD_ID)"

CXXFLAGS="$OPT $DBG $EXTRA_CFLAGS $TRACE_CFLAGS -Wall -Wextra -Wno-format -Wno-unused -std=c++14 -I./include -I./source/audioio/ffaudio -I./source/compression -I./source/crypto -I./source/crypto/mlkem -pthread -DMERCURY_GUI_ENABLED -I./third_party/imgui -I./third_party/imgui/backends ${EXTRA_CFLAGS_ENV:-}"
CFLAGS="$OPT $DBG $EXTRA_CFLAGS -Wall -Wno-unused -I./source/audioio/ffbase/ -I./source/audioio/ffaudio/ -I./include -I./source/compression -I./source/crypto -I./source/crypto/mlkem -pthread -std=c17 ${EXTRA_CFLAGS_ENV:-}"

# Platform-specific flags
if [[ "$OSTYPE" == "msys"* ]] || [[ "$OSTYPE" == "mingw"* ]] || [[ "$OSTYPE" == "cygwin"* ]]; then
    PLATFORM="windows"
    CXXFLAGS="$CXXFLAGS -I./third_party/glfw/include"
    LDFLAGS="-L./third_party/glfw/lib -lglfw3 -lopengl32 -lgdi32 -lole32 -ldsound -ldxguid -lws2_32 -lbcrypt -static-libgcc -static-libstdc++ -static -l:libwinpthread.a $EXTRA_LDFLAGS"
    # Standalone test binaries must link static too: a dynamic link picks up
    # whatever libstdc++-6.dll is first on PATH (e.g. Git's mingw64), and the
    # cross-toolchain ABI mismatch segfaults the test run.
    TEST_LDFLAGS="-static -static-libgcc -static-libstdc++"
elif [[ "$OSTYPE" == "darwin"* ]]; then
    PLATFORM="macos"
    CXXFLAGS="$CXXFLAGS $(pkg-config --cflags glfw3)"
    LDFLAGS="$(pkg-config --libs glfw3) -framework OpenGL -framework CoreFoundation -framework CoreAudio $EXTRA_LDFLAGS"
else
    PLATFORM="linux"
    if [ "${MERCURY_CROSS_BUILD:-0}" = "1" ]; then
        # Cross-compile: host's pkg-config doesn't know about the target sysroot.
        # Caller supplies --sysroot=... via EXTRA_CFLAGS_ENV; derive SYSROOT here
        # for the link step. If MERCURY_SYSROOT is set, prefer it; otherwise parse.
        if [ -n "${MERCURY_SYSROOT:-}" ]; then
            SYSROOT="$MERCURY_SYSROOT"
        else
            # Extract --sysroot=PATH from EXTRA_CFLAGS_ENV
            SYSROOT=$(echo "${EXTRA_CFLAGS_ENV:-}" | sed -n 's/.*--sysroot=\([^ ]*\).*/\1/p')
        fi
        if [ -z "$SYSROOT" ]; then
            echo "ERROR: MERCURY_CROSS_BUILD=1 but no sysroot found (set MERCURY_SYSROOT or --sysroot=... in EXTRA_CFLAGS_ENV)"
            exit 1
        fi
        echo "  (cross-compile: sysroot=$SYSROOT)"
        CXXFLAGS="$CXXFLAGS -I${SYSROOT}/usr/include"
        # --as-needed: drop NEEDED entries for shared libs whose symbols Mercury
        # doesn't actually reference (libpulse pulls in libdbus/libsndfile/etc
        # transitively; we don't call them, so don't link them).
        # --allow-shlib-undefined: tolerate unresolved symbols inside
        # indirectly-pulled .so files when those libs themselves get dropped.
        # --unresolved-symbols=ignore-in-shared-libs: belt-and-braces in case
        # any indirect symbol survives the --as-needed pass.
        LDFLAGS="--sysroot=${SYSROOT} -L${SYSROOT}/usr/lib/aarch64-linux-gnu -Wl,--as-needed -Wl,--allow-shlib-undefined -Wl,--unresolved-symbols=ignore-in-shared-libs -lglfw -lGL -lpulse -lasound -lpthread -lrt $EXTRA_LDFLAGS"
    else
        CXXFLAGS="$CXXFLAGS $(pkg-config --cflags glfw3)"
        LDFLAGS="$(pkg-config --libs glfw3) -lGL -lpulse -lasound -lpthread -lrt $EXTRA_LDFLAGS"

        # SOLUTION F (fix/ldpc-decode-accel): ARM CPU tuning for the on-Pi build.
        # `build.sh o3` runs natively on the aarch64 Pi 5 (Cortex-A76; confirmed
        # via tools/mercury_deploy_rpi.py:467/695 "both Pi 5 aarch64"). The stock
        # build is plain -O3 with NO -mcpu/-mtune, so the SPA LDPC hot loop
        # (tanh/atanh-heavy) is generated for a generic ARMv8 baseline. -mcpu=native
        # lets gcc emit Cortex-A76-tuned scheduling + the A76 FP/SIMD ISA, which is
        # the cheapest win on the iteration-bound decode.
        #
        # We do NOT add -ffast-math: it drops the NaN/Inf guards the SPA temp-clamp
        # (ldpc_decoder_SPA.cc temp==+/-1 saturation -> 2*atanh) and the OFDM
        # variance/LLR clamps rely on. -fno-math-errno is safe (we never read errno
        # from libm) and lets math calls be treated as pure -> better scheduling.
        #
        # Native ONLY (skipped for MERCURY_CROSS_BUILD, handled above): a cross host
        # may not be the target uarch, and -mcpu=native there would mistune. Guarded
        # by a compile probe so a toolchain that rejects -mcpu=native falls back to
        # the explicit Cortex-A76 target, then to no tuning (never breaks the build).
        # This block is aarch64-Linux-only => the Windows/macOS render is untouched.
        ARCH_M=$(uname -m 2>/dev/null || echo unknown)
        if [ "$ARCH_M" = "aarch64" ] || [ "$ARCH_M" = "arm64" ]; then
            cpu_probe() { echo 'int main(){return 0;}' | "$CXX" "$1" -x c++ - -o /dev/null 2>/dev/null; }
            ARM_TUNE=""
            if cpu_probe "-mcpu=native"; then
                ARM_TUNE="-mcpu=native"
            elif cpu_probe "-mcpu=cortex-a76"; then
                ARM_TUNE="-mcpu=cortex-a76"
            fi
            ARM_MATH=""
            if cpu_probe "-fno-math-errno"; then
                ARM_MATH="-fno-math-errno"
            fi
            if [ -n "$ARM_TUNE$ARM_MATH" ]; then
                echo "  (aarch64 tuning: ${ARM_TUNE:-none} ${ARM_MATH})"
                CXXFLAGS="$CXXFLAGS $ARM_TUNE $ARM_MATH"
                CFLAGS="$CFLAGS $ARM_TUNE $ARM_MATH"
            fi
        fi
    fi
fi

# Check dependencies before build (skip on Windows — uses vendored libs).
# Skip on cross-build too: the host won't have the target's dev libs natively;
# they live in the sysroot only.
if [ "$PLATFORM" != "windows" ] && [ "${MERCURY_CROSS_BUILD:-0}" != "1" ]; then
    check_deps "$PLATFORM"
fi

if [ "$PLATFORM" = "windows" ]; then
    OUTPUT="mercury${SUFFIX}.exe"
elif [ "${MERCURY_CROSS_BUILD:-0}" = "1" ]; then
    # Distinguish cross-built artifact from a host-native build sitting alongside it.
    OUTPUT="mercury_aarch64${SUFFIX}"
else
    OUTPUT="mercury${SUFFIX}"
fi

# Parallel job count
NPROC=$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)

# --- Dependency-tracked parallel compilation ---
# Uses gcc -MMD to generate .d files alongside .o files.
# On rebuild, only files whose source or headers changed are recompiled.

PIDS=()
FAIL=0
OBJ_FILES=""

# Wait for background jobs, enforcing max parallelism
wait_slot() {
    while [ ${#PIDS[@]} -ge "$NPROC" ]; do
        # Wait for any one child to finish
        local new_pids=()
        for pid in "${PIDS[@]}"; do
            if kill -0 "$pid" 2>/dev/null; then
                new_pids+=("$pid")
            else
                wait "$pid" || FAIL=1
            fi
        done
        if [ ${#new_pids[@]} -ge "$NPROC" ]; then
            # All still running — wait for one
            wait -n 2>/dev/null || {
                # wait -n not available (older bash) — wait for all
                for pid in "${PIDS[@]}"; do wait "$pid" || FAIL=1; done
                PIDS=()
                return
            }
            # Reap finished
            new_pids=()
            for pid in "${PIDS[@]}"; do
                if kill -0 "$pid" 2>/dev/null; then
                    new_pids+=("$pid")
                else
                    wait "$pid" || FAIL=1
                fi
            done
        fi
        PIDS=("${new_pids[@]}")
    done
}

wait_all() {
    for pid in "${PIDS[@]}"; do
        wait "$pid" || FAIL=1
    done
    PIDS=()
    if [ "$FAIL" = "1" ]; then
        echo "*** Compilation failed"
        exit 1
    fi
}

# needs_rebuild SRC OBJ — true if obj is missing, src is newer, or any dep header changed
needs_rebuild() {
    local src="$1" obj="$2"
    [ ! -f "$obj" ] && return 0
    [ "$src" -nt "$obj" ] && return 0
    # Check gcc-generated dependency file
    local dep="${obj%.o}.d"
    if [ -f "$dep" ]; then
        # .d file lists obj: src header1 header2 ...
        # Check if any dependency is newer than obj
        while IFS= read -r line; do
            # Remove backslash continuations and the target prefix
            line="${line%\\}"
            line="${line#*: }"
            for f in $line; do
                [ -f "$f" ] && [ "$f" -nt "$obj" ] && return 0
            done
        done < "$dep"
    else
        # No dep file — must rebuild to generate it
        return 0
    fi
    return 1
}

# compile_cc SRC OBJ [extra flags...] — compile one C++ file
compile_cc() {
    local src="$1" obj="$2"; shift 2
    local dep="${obj%.o}.d"
    $CXX $CXXFLAGS -MMD -MF "$dep" "$@" -c -o "$obj" "$src"
}

# compile_c SRC OBJ [extra flags...] — compile one C file
compile_c() {
    local src="$1" obj="$2"; shift 2
    local dep="${obj%.o}.d"
    $CC $CFLAGS -MMD -MF "$dep" "$@" -c -o "$obj" "$src"
}

# Map source path to build dir obj path: source/foo/bar.cc -> build/MODE/source/foo/bar.o
obj_path() {
    echo "${BUILDDIR}/${1%.*}.o"
}

# Source files
CPP_SOURCES="
source/main.cc
source/datalink_layer/arq_commander.cc
source/datalink_layer/arq_common.cc
source/datalink_layer/l1_tx_journal.cc
source/datalink_layer/l1_block_ack.cc
source/datalink_layer/arq_responder.cc
source/datalink_layer/l1_block_codec.cc
source/datalink_layer/test_bigblock_arq_unit.cc
source/datalink_layer/test_rx_drain.cc
source/datalink_layer/test_restage_requeue.cc
source/datalink_layer/test_stream_offset.cc
source/datalink_layer/test_decompress_false_accept.cc
source/datalink_layer/test_kx_data_tamper.cc
source/datalink_layer/test_compact_confirm_rx.cc
source/datalink_layer/test_recovery_ack_capture_exercise.cc
source/datalink_layer/test_rsp_timeout_format.cc
source/datalink_layer/b2f_handler.cc
source/datalink_layer/channel_state_lookup.cc
source/datalink_layer/datalink_config.cc
source/datalink_layer/fifo_buffer.cc
source/datalink_layer/rate_optimizer.cc
source/datalink_layer/tcp_socket.cc
source/datalink_layer/timer.cc
source/physical_layer/awgn.cc
source/physical_layer/crc16_modbus_rtu.cc
source/physical_layer/data_container.cc
source/physical_layer/dist_matcher.cc
source/physical_layer/error_rate.cc
source/physical_layer/fir_filter.cc
source/physical_layer/interleaver.cc
source/physical_layer/interpolator.cc
source/physical_layer/ldpc.cc
source/physical_layer/ldpc_decode_pool.cc
source/physical_layer/test_decode_marathon.cc
source/physical_layer/ldpc_decoder_GBF.cc
source/physical_layer/ldpc_decoder_SPA.cc
source/physical_layer/mercury_met_2_16.cc
source/physical_layer/mercury_normal_1_16.cc
source/physical_layer/mercury_normal_14_16.cc
source/physical_layer/mercury_normal_2_16.cc
source/physical_layer/mercury_normal_3_16.cc
source/physical_layer/mercury_normal_4_16.cc
source/physical_layer/mercury_normal_5_16.cc
source/physical_layer/mercury_normal_6_16.cc
source/physical_layer/mercury_normal_8_16.cc
source/physical_layer/mercury_normal_10_16.cc
source/physical_layer/mercury_normal_12_16.cc
source/physical_layer/misc.cc
source/physical_layer/ofdm.cc
source/physical_layer/precook_copy_from.cc
source/physical_layer/physical_config.cc
source/physical_layer/plot.cc
source/physical_layer/psk.cc
source/physical_layer/mfsk.cc
source/physical_layer/mfsk_ctrl_codec.cc
source/physical_layer/mfsk_ctrl_codec_tests.cc
source/physical_layer/telecom_system.cc
source/common/os_interop.cc
source/common/ring_buffer_posix.cc
source/common/shm_posix.cc
source/common/sim_clock.cc
source/common/sim_clock_tests.cc
source/gui/gui_main.cc
source/gui/ini_parser.cc
source/gui/widgets/waterfall.cc
source/gui/dialogs/setup_dialog.cc
source/gui/dialogs/soundcard_dialog.cc
source/compression/mercury_compress.cc
source/compression/winlink_dict.cc
source/compression/test_winlink_dict.cc
source/compression/lzhuf_buffer.cc
source/crypto/mercury_crypto.cc
source/crypto/test_aead_nonce.cc
source/crypto/test_mlkem_hybrid.cc
"

# Auto-discover test-helper sources added under source/ (compiled INTO mercury).
# Exclude standalone test programs that define their own main() -- those are not
# part of the mercury link and would collide with source/main.cc.
while IFS= read -r test_src; do
    if ! printf '%s\n' "$CPP_SOURCES" | grep -Fxq "$test_src"; then
        CPP_SOURCES="${CPP_SOURCES}${test_src}
"
    fi
done < <(find source -type f -name 'test_*.cc' -print | sort | while IFS= read -r f; do
    grep -qE 'int[[:space:]]+main[[:space:]]*\(' "$f" || printf '%s\n' "$f"
done)

# Compression library C sources (PPMd8 from LZMA SDK, zstd amalgamated, LZHUF for B2F)
COMPRESSION_C_SOURCES="
source/compression/ppmd/Ppmd8.c
source/compression/ppmd/Ppmd8Dec.c
source/compression/ppmd/Ppmd8Enc.c
source/compression/zstd/zstd.c
source/compression/lzhuf/lzhuf.c
"

# Crypto C sources (monocypher + ML-KEM-768)
CRYPTO_C_SOURCES="
source/crypto/monocypher.c
source/crypto/mlkem/mlkem_native.c
"

IMGUI_SOURCES="
third_party/imgui/imgui.cpp
third_party/imgui/imgui_draw.cpp
third_party/imgui/imgui_tables.cpp
third_party/imgui/imgui_widgets.cpp
third_party/imgui/backends/imgui_impl_glfw.cpp
third_party/imgui/backends/imgui_impl_opengl3.cpp
"

# Platform-specific audio backends
AUDIO_C_SOURCES="source/audioio/audioio.c"
if [ "$PLATFORM" = "windows" ]; then
    AUDIO_C_SOURCES="$AUDIO_C_SOURCES
source/audioio/ffaudio/ffaudio/dsound.c
source/audioio/ffaudio/ffaudio/wasapi.c
"
elif [ "$PLATFORM" = "macos" ]; then
    AUDIO_C_SOURCES="$AUDIO_C_SOURCES
source/audioio/ffaudio/ffaudio/coreaudio.c
"
else
    AUDIO_C_SOURCES="$AUDIO_C_SOURCES
source/audioio/ffaudio/ffaudio/alsa.c
source/audioio/ffaudio/ffaudio/pulse.c
"
fi

# Create build subdirectories
for src in $CPP_SOURCES $IMGUI_SOURCES $AUDIO_C_SOURCES $COMPRESSION_C_SOURCES $CRYPTO_C_SOURCES; do
    mkdir -p "${BUILDDIR}/$(dirname "$src")"
done

# --- Compile all sources in parallel ---
COMPILED=0

echo "Compiling ($NPROC parallel jobs)..."

# C++ sources
for src in $CPP_SOURCES; do
    obj=$(obj_path "$src")
    OBJ_FILES="$OBJ_FILES $obj"
    if needs_rebuild "$src" "$obj"; then
        echo "  $src"
        wait_slot
        compile_cc "$src" "$obj" &
        PIDS+=($!)
        ((COMPILED++)) || true
    fi
done

# ImGui sources (.cpp)
for src in $IMGUI_SOURCES; do
    obj=$(obj_path "$src")
    OBJ_FILES="$OBJ_FILES $obj"
    if needs_rebuild "$src" "$obj"; then
        echo "  $src"
        wait_slot
        compile_cc "$src" "$obj" &
        PIDS+=($!)
        ((COMPILED++)) || true
    fi
done

# Audio sources
AUDIO_OBJ_FILES=""
for src in $AUDIO_C_SOURCES; do
    obj=$(obj_path "$src")
    AUDIO_OBJ_FILES="$AUDIO_OBJ_FILES $obj"
    if needs_rebuild "$src" "$obj"; then
        echo "  $src"
        wait_slot
        if [[ "$src" == *audioio.c ]]; then
            # audioio.c includes C++ headers, must compile as C++
            compile_cc "$src" "$obj" &
        else
            compile_c "$src" "$obj" &
        fi
        PIDS+=($!)
        ((COMPILED++)) || true
    fi
done

# Compression C sources
COMPRESS_OBJ_FILES=""
for src in $COMPRESSION_C_SOURCES; do
    obj=$(obj_path "$src")
    COMPRESS_OBJ_FILES="$COMPRESS_OBJ_FILES $obj"
    if needs_rebuild "$src" "$obj"; then
        echo "  $src"
        EXTRA_C=""
        if [[ "$src" == *lzhuf.c ]]; then
            EXTRA_C="-DLZHUF -DB2F"
        fi
        wait_slot
        compile_c "$src" "$obj" -Wno-extra -Wno-sign-compare -Wno-implicit-fallthrough $EXTRA_C &
        PIDS+=($!)
        ((COMPILED++)) || true
    fi
done

# Crypto C sources
CRYPTO_OBJ_FILES=""
for src in $CRYPTO_C_SOURCES; do
    obj=$(obj_path "$src")
    CRYPTO_OBJ_FILES="$CRYPTO_OBJ_FILES $obj"
    if needs_rebuild "$src" "$obj"; then
        echo "  $src"
        EXTRA_C=""
        if [[ "$src" == *mlkem_native.c ]]; then
            EXTRA_C="-I./source/crypto/mlkem -DMLK_CONFIG_PARAMETER_SET=768"
        fi
        wait_slot
        compile_c "$src" "$obj" -Wno-extra -Wno-sign-compare $EXTRA_C &
        PIDS+=($!)
        ((COMPILED++)) || true
    fi
done

# Wait for all compilations to finish
wait_all

run_l1_block_codec_test() {
    local test_bin="${BUILDDIR}/test_l1_block_codec"
    echo "Building and running L1 block codec test..."
    $CXX $CXXFLAGS source/datalink_layer/l1_block_codec.cc \
        source/datalink_layer/test_l1_block_codec.cc -o "$test_bin" ${TEST_LDFLAGS:-}
    "$test_bin"
}

run_l1_block_codec_test

run_l1_stage3_test() {
    local test_bin="${BUILDDIR}/test_l1_stage3"
    echo "Building and running L1 Stage-3 production-path test..."
    $CXX $CXXFLAGS source/datalink_layer/l1_block_codec.cc \
        source/datalink_layer/l1_tx_journal.cc \
        source/datalink_layer/l1_block_ack.cc \
        source/datalink_layer/test_l1_stage3.cc -o "$test_bin" -pthread ${TEST_LDFLAGS:-}
    "$test_bin"
}

run_l1_stage3_test

if [ "$COMPILED" -eq 0 ]; then
    # Check if output exists and is up to date
    if [ -f "$OUTPUT" ]; then
        echo "  (nothing changed)"
        echo "=== Build complete: $OUTPUT ==="
        ls -la "$OUTPUT"
        exit 0
    fi
fi

echo "  $COMPILED files compiled"

# Create audioio.a
ar rc "${BUILDDIR}/audioio.a" $AUDIO_OBJ_FILES

# Link
echo "Linking $OUTPUT..."
$CXX -o "$OUTPUT" $OBJ_FILES $COMPRESS_OBJ_FILES $CRYPTO_OBJ_FILES "${BUILDDIR}/audioio.a" $LDFLAGS

# The journal contract test has its own main and is intentionally outside the
# modem link. Run it on every native build so reset-row coverage cannot drift.
if [ "$PLATFORM" != "windows" ] && [ "${MERCURY_CROSS_BUILD:-0}" != "1" ]; then
    L1_TEST_BIN="${BUILDDIR}/test_l1_tx_journal"
    echo "Building and running L1 ownership journal contract test..."
    $CXX $CXXFLAGS -I./include source/datalink_layer/l1_tx_journal.cc \
        source/datalink_layer/test_l1_tx_journal.cc -o "$L1_TEST_BIN" -pthread
    "$L1_TEST_BIN"
fi

echo "=== Build complete: $OUTPUT ==="
ls -la "$OUTPUT"

if [ "$MODE" = "release" ]; then
    echo ""
    if [ "$PLATFORM" = "windows" ]; then
        echo "*** REMINDER: Install to Program Files with:"
        echo "    cp $OUTPUT \"/c/Program Files/Mercury/\""
    else
        echo "*** REMINDER: Install with:"
        echo "    sudo install -m 755 $OUTPUT /usr/local/bin/mercury"
    fi
fi
