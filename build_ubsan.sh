#!/bin/bash
# build_ubsan.sh — Mercury UBSan investigation build (O3-UB-hunt).
# Compiles the modem with -fsanitize=undefined and links the local ubsan_stub.c
# (no libubsan ships with this GCC). Produces mercury_ubsan.exe that prints
# [UBSAN] file:line:col for every UB hit and a [UBSAN-SUMMARY] tally at exit.
#
# Usage:  ./build_ubsan.sh [O1|O3]   (default O1 — fast; O3 to match release UB exposure)
# Env:    UB_NULLPTR=1   add -fsanitize=null,nonnull-attribute,returns-nonnull-attribute (default on)
set -e
cd "$(dirname "$0")"

OPTLVL="${1:-O1}"
case "$OPTLVL" in
    O1) OPT="-O1" ;;
    O3) OPT="-O3" ;;
    *)  echo "usage: $0 [O1|O3]"; exit 1 ;;
esac

GXASSERT="${GLIBCXX_ASSERT:-0}"
[ "$GXASSERT" = "1" ] && DIRSFX="_glibcxx" || DIRSFX=""
BUILDDIR="build/ubsan_${OPTLVL}${DIRSFX}"
mkdir -p "$BUILDDIR"

CXX="${CXX:-g++}"
CC="${CC:-gcc}"

# UBSan flags. We deliberately DROP -fsanitize=alignment for the C compression/
# crypto TUs only if it floods (handled below); keep full set for modem code.
# -fno-omit-frame-pointer for readable stacks. We let GCC RECOVER (default) so the
# stub logs and continues — one run finds all sites.
SAN="-fsanitize=undefined -fno-omit-frame-pointer"
# null/nonnull are not in the default 'undefined' group on some GCC; add them.
SAN="$SAN -fsanitize=bounds -fsanitize=null -fsanitize=nonnull-attribute -fsanitize=returns-nonnull-attribute"
# vptr/function checks need the full libubsan runtime (dynamic_type_cache) which
# we don't have, and they are NOT the -O3 UB class we hunt. Disable them.
SAN="$SAN -fno-sanitize=vptr"

# GLIBCXX_ASSERT=1 adds libstdc++ bounds assertions (std::vector::operator[],
# std::string, iterators) — catches HEAP container OOB that UBSan's -fsanitize=
# bounds cannot see (it only sees statically-sized arrays). The original Feb O3
# crash was a heap-buffer OOB, so this is the complementary detector. Suffix the
# binary so it does not clobber the pure-UBSan one.
HARDEN=""
BINSFX=""
if [ "${GLIBCXX_ASSERT:-0}" = "1" ]; then
    HARDEN="-D_GLIBCXX_ASSERTIONS"
    BINSFX="_glibcxx"
    echo "  (+ _GLIBCXX_ASSERTIONS heap-container bounds checks)"
fi
BASE_INC="-I./include -I./source/audioio/ffaudio -I./source/compression -I./source/crypto -I./source/crypto/mlkem -I./third_party/imgui -I./third_party/imgui/backends -I./third_party/glfw/include"
CXXFLAGS="$OPT -g $SAN $HARDEN -Wall -Wextra -Wno-format -Wno-unused -std=c++14 $BASE_INC -pthread -DMERCURY_GUI_ENABLED"
CFLAGS_C="$OPT -g $SAN -Wall -Wno-unused -I./source/audioio/ffbase/ -I./source/audioio/ffaudio/ -I./include -I./source/compression -I./source/crypto -I./source/crypto/mlkem -pthread -std=c17"

LDFLAGS="-L./third_party/glfw/lib -lglfw3 -lopengl32 -lgdi32 -lole32 -ldsound -ldxguid -lws2_32 -lbcrypt -static-libgcc -static-libstdc++ -static -l:libwinpthread.a"

CPP_SOURCES="
source/main.cc
source/datalink_layer/arq_commander.cc
source/datalink_layer/arq_common.cc
source/datalink_layer/arq_responder.cc
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
source/physical_layer/error_rate.cc
source/physical_layer/fir_filter.cc
source/physical_layer/interleaver.cc
source/physical_layer/interpolator.cc
source/physical_layer/ldpc.cc
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
source/compression/lzhuf_buffer.cc
source/crypto/mercury_crypto.cc
"

# Optional: a test_*.cc file may exist for in-process bigblock; include if present.
for extra in source/datalink_layer/test_bigblock_arq_unit.cc; do
    [ -f "$extra" ] && CPP_SOURCES="$CPP_SOURCES
$extra"
done

# Third-party C libs (PPMd/zstd/lzhuf, monocypher, mlkem). UBSan on these can flood
# with alignment/overflow noise that is NOT the modem bug, so build them WITHOUT
# the modem's full sanitizer set — only keep the cheap subset off to reduce noise.
# We still want bounds on them, but drop alignment (packed buffer libs).
C_SAN="-fsanitize=undefined -fsanitize=bounds -fno-sanitize=alignment -fno-omit-frame-pointer"

COMPRESSION_C_SOURCES="
source/compression/ppmd/Ppmd8.c
source/compression/ppmd/Ppmd8Dec.c
source/compression/ppmd/Ppmd8Enc.c
source/compression/zstd/zstd.c
source/compression/lzhuf/lzhuf.c
"
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
AUDIO_C_SOURCES="
source/audioio/audioio.c
source/audioio/ffaudio/ffaudio/dsound.c
source/audioio/ffaudio/ffaudio/wasapi.c
"

NPROC=$(nproc 2>/dev/null || echo 8)
PIDS=(); FAIL=0; OBJ_FILES=""
wait_slot(){ while [ ${#PIDS[@]} -ge "$NPROC" ]; do wait -n 2>/dev/null || { for p in "${PIDS[@]}"; do wait "$p" || FAIL=1; done; PIDS=(); }; local np=(); for p in "${PIDS[@]}"; do kill -0 "$p" 2>/dev/null && np+=("$p") || { wait "$p" || FAIL=1; }; done; PIDS=("${np[@]}"); done; }
op(){ echo "${BUILDDIR}/${1%.*}.o"; }
needs(){ [ ! -f "$2" ] && return 0; [ "$1" -nt "$2" ] && return 0; local d="${2%.o}.d"; [ ! -f "$d" ] && return 0; local ok=1; while IFS= read -r l; do l="${l%\\}"; l="${l#*: }"; for f in $l; do [ -f "$f" ] && [ "$f" -nt "$2" ] && return 0; done; done < "$d"; return 1; }

for src in $CPP_SOURCES $IMGUI_SOURCES; do mkdir -p "${BUILDDIR}/$(dirname "$src")"; done
for src in $AUDIO_C_SOURCES $COMPRESSION_C_SOURCES $CRYPTO_C_SOURCES; do mkdir -p "${BUILDDIR}/$(dirname "$src")"; done

echo "=== UBSan build ($OPTLVL, $NPROC jobs) ==="
COMPILED=0
for src in $CPP_SOURCES $IMGUI_SOURCES; do
    o=$(op "$src"); OBJ_FILES="$OBJ_FILES $o"
    if needs "$src" "$o"; then echo "  CXX $src"; wait_slot
        $CXX $CXXFLAGS -MMD -MF "${o%.o}.d" -c -o "$o" "$src" & PIDS+=($!); ((COMPILED++))||true; fi
done
AUDIO_OBJ=""
for src in $AUDIO_C_SOURCES; do
    o=$(op "$src"); AUDIO_OBJ="$AUDIO_OBJ $o"
    if needs "$src" "$o"; then echo "  CC  $src"; wait_slot
        if [[ "$src" == *audioio.c ]]; then
            $CXX $CXXFLAGS -MMD -MF "${o%.o}.d" -c -o "$o" "$src" & PIDS+=($!)
        else
            $CC $CFLAGS_C -MMD -MF "${o%.o}.d" -c -o "$o" "$src" & PIDS+=($!)
        fi
        ((COMPILED++))||true; fi
done
COMPRESS_OBJ=""
for src in $COMPRESSION_C_SOURCES; do
    o=$(op "$src"); COMPRESS_OBJ="$COMPRESS_OBJ $o"
    EXC=""; [[ "$src" == *lzhuf.c ]] && EXC="-DLZHUF -DB2F"
    if needs "$src" "$o"; then echo "  CC  $src"; wait_slot
        $CC $OPT -g $C_SAN -Wno-unused -I./include -I./source/compression -I./source/crypto -I./source/crypto/mlkem -pthread -std=c17 -Wno-extra -Wno-sign-compare -Wno-implicit-fallthrough $EXC -MMD -MF "${o%.o}.d" -c -o "$o" "$src" & PIDS+=($!); ((COMPILED++))||true; fi
done
CRYPTO_OBJ=""
for src in $CRYPTO_C_SOURCES; do
    o=$(op "$src"); CRYPTO_OBJ="$CRYPTO_OBJ $o"
    EXC=""; [[ "$src" == *mlkem_native.c ]] && EXC="-I./source/crypto/mlkem -DMLK_CONFIG_PARAMETER_SET=768"
    if needs "$src" "$o"; then echo "  CC  $src"; wait_slot
        $CC $OPT -g $C_SAN -Wno-unused -I./include -I./source/compression -I./source/crypto -I./source/crypto/mlkem -pthread -std=c17 -Wno-extra -Wno-sign-compare $EXC -MMD -MF "${o%.o}.d" -c -o "$o" "$src" & PIDS+=($!); ((COMPILED++))||true; fi
done
# UBSan stub (no sanitizer on the stub itself)
STUB_OBJ="${BUILDDIR}/ubsan_stub.o"
if needs ubsan_stub.c "$STUB_OBJ"; then echo "  CC  ubsan_stub.c"; $CC -O0 -g -c -o "$STUB_OBJ" ubsan_stub.c; fi

for p in "${PIDS[@]}"; do wait "$p" || FAIL=1; done; PIDS=()
[ "$FAIL" = "1" ] && { echo "*** compile failed"; exit 1; }
echo "  ($COMPILED files compiled)"

ar rc "${BUILDDIR}/audioio.a" $AUDIO_OBJ
OUTBIN="mercury_ubsan_${OPTLVL}${BINSFX}.exe"
echo "Linking ${OUTBIN} ..."
# Link WITHOUT -fsanitize on the link line (we provide the runtime via stub).
$CXX -o "${OUTBIN}" $OBJ_FILES $COMPRESS_OBJ $CRYPTO_OBJ "${BUILDDIR}/audioio.a" "$STUB_OBJ" $LDFLAGS
echo "=== built ${OUTBIN} ==="
ls -la "${OUTBIN}"
