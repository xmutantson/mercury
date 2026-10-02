"""Source-contract checks for production MFSK and OFDM SACK observations.

These inspect the actual receive lane without constructing a telemetry event or
claiming that compact confirmations supplied a selective bitmap.
"""

from pathlib import Path
import re
import unittest

from test_rro_arq_capture_window import block_after, cpp_code


ROOT = Path(__file__).resolve().parents[2]
ARQ_SOURCE = ROOT / "source/datalink_layer/arq_commander.cc"
COMMON_SOURCE = ROOT / "source/datalink_layer/arq_common.cc"
DEFINES_SOURCE = ROOT / "include/common/common_defines.h"


def compact(source):
    return re.sub(r"\s+", "", source)


def assert_explicit_sack_wiring(source, defines):
    code = cpp_code(source)
    body = block_after(code, r"void\s+cl_arq_controller::"
                       r"process_messages_rx_acks_data\s*\(\s*\)\s*\{")
    explicit = block_after(body, r"if\s*\(\s*!mfsk_handled_this_poll\s*\)\s*\{")
    assert body.count("record_arq_sack_window") == 1
    assert explicit.count("record_arq_sack_window") == 1
    assert re.search(r"#define\s+MFSK_SACK_BITMAP_BITS\s+30\b", defines)

    # Both newest-tail and recovered-phase decodes fill the same observed value.
    packed = compact(explicit)
    assert packed.count("tail_samples,&rx_bsi,&rx_bitmap,&rx_crc12,&mfsk_matched)") == 2
    assert "constuint8_twire_bsi=rx_bsi;" in packed
    crc_check = block_after(explicit, r"if\s*\(\s*decoded\s*\)\s*\{")
    crc_input = compact(crc_check)
    assert "crc_input[0]=(char)wire_bsi;" in crc_input
    for index, shift in ((1, 24), (2, 16), (3, 8)):
        assert f"crc_input[{index}]=(char)((rx_bitmap>>{shift})&0xFF);" in crc_input
    assert "crc_input[4]=(char)(rx_bitmap&0xFF);" in crc_input
    assert "expected_crc12=CRC12_calc(crc_input,5);" in crc_input
    failed_crc = block_after(crc_check,
                             r"if\s*\(\s*rx_crc12\s*!=\s*expected_crc12\s*\)\s*\{")
    assert "decoded=false;" in compact(failed_crc)
    assert "record_arq_sack_window" not in crc_check
    crc_end = explicit.index(crc_check) + len(crc_check)
    validated = block_after(explicit[crc_end:], r"if\s*\(\s*decoded\s*\)\s*\{")
    assert validated.count("record_arq_sack_window") == 1

    validated_code = compact(validated)
    owner_call = ("target_owned=resolve_and_validate_generation_ack("
                  "rx_bsi,data_batch_size,true,is_clean_confirmation,"
                  "bitmap_ok,bitmap_all,&resolved);")
    assert owner_call in validated_code
    assert "resolved_target=resolved.target;" in validated_code
    invalid_owner = block_after(validated,
                               r"if\s*\(\s*!target_owned\s*\)\s*\{")
    assert "decoded=false;" in compact(invalid_owner)
    accepted = block_after(validated,
                          r"if\s*\(\s*decoded\s*&&\s*target_owned\s*\)\s*\{")
    assert accepted.count("record_arq_sack_window") == 1
    assert validated.index(owner_call.split("=")[0]) < validated.index(accepted)
    assert validated.index("resolved_target = resolved.target;") < validated.index(accepted)

    observed = block_after(
        accepted, r"if\s*\(\s*data_batch_size\s*>\s*0\s*&&\s*"
        r"data_batch_size\s*<=\s*MFSK_SACK_BITMAP_BITS\s*&&\s*"
        r"rro::Telemetry::instance\s*\(\s*\)\.enabled\s*\(\s*\)\s*\)\s*\{")
    observed_code = compact(observed)
    assert ("constunsignedcharrro_sack_bitmap[4]={"
            "static_cast<unsignedchar>(rx_bitmap&0xFFu),"
            "static_cast<unsignedchar>((rx_bitmap>>8)&0xFFu),"
            "static_cast<unsignedchar>((rx_bitmap>>16)&0xFFu),"
            "static_cast<unsignedchar>((rx_bitmap>>24)&0xFFu)};") in observed_code
    assert ("rro::Telemetry::instance().record_arq_sack_window("
            "resolved_target,data_batch_size,rro_sack_bitmap,"
            "(data_batch_size+7)/8);") in observed_code
    assert not re.search(r"\b(?:new|malloc|calloc|realloc|MUTEX_LOCK)\b", accepted)

    # The publication follows the capture snapshot's release; it adds no lock.
    hook = explicit.index("record_arq_sack_window")
    unlocks = list(re.finditer(r"MUTEX_UNLOCK\s*\(\s*&capture_prep_mutex\s*\)\s*;",
                              explicit[:hook]))
    assert unlocks
    assert "MUTEX_LOCK" not in explicit[unlocks[-1].end():hook]

    resolver = block_after(code, r"bool\s+cl_arq_controller::"
                           r"resolve_and_validate_generation_ack\s*\([^)]*\)\s*const\s*\{")
    resolver_code = compact(resolver)
    assert "if(!out||!integrity_ok)returnfalse;" in resolver_code
    assert ("resolved.target=generation_ack_resolve_target("
            "wire_field,cumulative_ack_enabled);") in resolver_code
    assert ("if(!generation_ack_target_owned(resolved.target,span,"
            "!clean,&resolved.shadow_owned))returnfalse;") in resolver_code

    for name in ("cmd_compact_confirm_crc_valid",
                 "cmd_compact_confirm_sack_window_accept",
                 "cmd_compact_confirm_live_accept"):
        compact_body = block_after(code, r"bool\s+cl_arq_controller::" + name
                                   + r"\s*\([^)]*\)\s*\{")
        assert "record_arq_sack_window" not in compact_body


def assert_ofdm_received_sack_wiring(source, commander, defines):
    code = cpp_code(source)
    decode = block_after(code, r"bool\s+cl_arq_controller::decode_sack_v2_frame"
                         r"\s*\([^)]*\)\s*\{")
    received_code = compact(decode)
    assert decode.count("record_arq_sack_window") == 1
    assert "unsignedcharpayload[1+1+(MAX_SACK_BATCH_SIZE+7)/8+1];" in received_code
    assert "payload[b]=(unsignedchar)messages_rx_buffer.data[b];" in received_code
    assert "computed_crc=CRC8_calc((char*)payload,2+bitmap_bytes);" in received_code
    failed_crc = block_after(decode,
                             r"if\s*\(\s*rx_crc\s*!=\s*computed_crc\s*\)\s*\{")
    assert "returnfalse;" in compact(failed_crc)
    hook = decode.index("record_arq_sack_window")
    assert decode.index(failed_crc) + len(failed_crc) < hook
    assert ("if(telemetry.enabled())telemetry.record_arq_sack_window("
            "(int)generation_ack_resolve_target(payload[0],cumulative_ack_enabled),"
            "nframes,&payload[2],bitmap_bytes);") in received_code
    assert "resolve_and_validate_generation_ack" not in decode
    assert "generation_ack_target_owned" not in decode
    assert "intbitmap_bytes=(nframes+7)/8;" in received_code

    # Publication remains inside the CRC-valid decoder, before caller ownership.
    body = block_after(cpp_code(commander), r"void\s+cl_arq_controller::"
                       r"process_messages_rx_acks_data\s*\(\s*\)\s*\{")
    ofdm = block_after(body, r"if\s*\(\s*messages_rx_buffer\.status\s*==\s*RECEIVED"
                      r"\s*&&\s*messages_rx_buffer\.type\s*==\s*SACK_RSP\s*\)\s*\{")
    assert ofdm.index("decode_sack_v2_frame") < ofdm.index("resolve_and_validate_generation_ack")
    assert "record_arq_sack_window" not in ofdm

    # Check the actual unsigned canonical helper, including its legacy fallback.
    define_code = cpp_code(defines)
    helper = block_after(define_code, r"inline\s+unsigned\s+char\s+"
                         r"generation_ack_resolve_target\s*\([^)]*\)\s*\{")
    assert ("if(generation_canon_enabled()&&cap_on)"
            "return(unsignedchar)(((unsigned)wire_field+1u)&0xFFu);"
            "returnwire_field;") == compact(helper)


class ProductionArqSackWiringTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = ARQ_SOURCE.read_text(encoding="utf-8")
        cls.common = COMMON_SOURCE.read_text(encoding="utf-8")
        cls.defines = DEFINES_SOURCE.read_text(encoding="utf-8")

    def reject_change(self, old, new):
        changed = self.source.replace(old, new, 1)
        self.assertNotEqual(changed, self.source)
        with self.assertRaises(AssertionError):
            assert_explicit_sack_wiring(changed, self.defines)

    def test_real_decode_crc_owner_and_span_dominate_one_observation(self):
        assert_explicit_sack_wiring(self.source, self.defines)

    def test_crc_failure_cannot_remain_decoded(self):
        start = self.source.index("if(rx_crc12 != expected_crc12)",
                                  self.source.index("void cl_arq_controller::process_messages_rx_acks_data()"))
        end = self.source.index("rx_bsi = role_demand_restore_ack_bsi", start)
        changed = (self.source[:start]
                   + self.source[start:end].replace("decoded = false;", "decoded = true;", 1)
                   + self.source[end:])
        self.assertNotEqual(changed, self.source)
        with self.assertRaises(AssertionError):
            assert_explicit_sack_wiring(changed, self.defines)

    def test_owner_gate_cannot_be_removed(self):
        self.reject_change("if(decoded && target_owned)\n\t\t\t\t\t\t\t{",
                           "if(decoded)\n\t\t\t\t\t\t\t{")

    def test_owner_span_validation_cannot_be_replaced_by_integrity_alone(self):
        self.reject_change("target_owned = resolve_and_validate_generation_ack(\n"
                           "\t\t\t\t\t\t\t\trx_bsi, data_batch_size, /*integrity_ok=*/true,",
                           "target_owned = resolve_and_validate_generation_ack(\n"
                           "\t\t\t\t\t\t\t\trx_bsi, 1, /*integrity_ok=*/true,")

    def test_identity_cannot_bypass_protocol_resolver(self):
        self.reject_change("resolved.target = generation_ack_resolve_target(\n"
                           "\t\twire_field, cumulative_ack_enabled);",
                           "resolved.target = wire_field;")

    def test_exact_received_bit_order_not_an_all_ack_fill(self):
        self.reject_change("static_cast<unsigned char>((rx_bitmap >> 8) & 0xFFu)",
                           "static_cast<unsigned char>(0xFFu)")

    def test_width_cannot_claim_unobserved_slots(self):
        self.reject_change("&& data_batch_size <= MFSK_SACK_BITMAP_BITS",
                           "&& data_batch_size <= MAX_SACK_BATCH_SIZE")

    def test_identity_is_resolved_target_not_wire_or_demand_octet(self):
        self.reject_change("resolved_target, data_batch_size, rro_sack_bitmap,",
                           "wire_bsi, data_batch_size, rro_sack_bitmap,")

    def test_byte_count_tracks_the_same_observed_span(self):
        self.reject_change("(data_batch_size + 7) / 8);", "sizeof(rro_sack_bitmap));")

    def test_disabled_guard_cannot_be_removed(self):
        self.reject_change("&& rro::Telemetry::instance().enabled())\n"
                           "\t\t\t\t\t\t\t\t{",
                           ")\n\t\t\t\t\t\t\t\t{")

    def test_duplicate_publication_is_rejected(self):
        call = ("rro::Telemetry::instance().record_arq_sack_window(\n"
                "\t\t\t\t\t\t\t\t\t\tresolved_target, data_batch_size, rro_sack_bitmap,\n"
                "\t\t\t\t\t\t\t\t\t\t(data_batch_size + 7) / 8);")
        self.reject_change(call, call + "\n" + call)

    def test_compact_implicit_clean_must_not_publish_a_bitmap(self):
        self.reject_change("uint8_t cc_bsi = 0;",
                           "uint8_t cc_bsi = 0;\n"
                           "rro::Telemetry::instance().record_arq_sack_window(0, 30, nullptr, 4);")

    def test_ofdm_crc_valid_received_bitmap_uses_canonical_identity(self):
        assert_ofdm_received_sack_wiring(self.common, self.source, self.defines)

    def test_ofdm_raw_predecessor_cannot_be_the_bitmap_identity(self):
        changed = self.common.replace(
            "(int)generation_ack_resolve_target(payload[0], cumulative_ack_enabled)",
            "(int)payload[0]", 1)
        self.assertNotEqual(changed, self.common)
        with self.assertRaises(AssertionError):
            assert_ofdm_received_sack_wiring(changed, self.source, self.defines)

    def test_ofdm_bad_crc_cannot_reach_received_observation(self):
        start = self.common.index("bool cl_arq_controller::decode_sack_v2_frame")
        end = self.common.index("// CRC pass", start)
        changed = (self.common[:start]
                   + self.common[start:end].replace("return false;", "return true;")
                   + self.common[end:])
        self.assertNotEqual(changed, self.common)
        with self.assertRaises(AssertionError):
            assert_ofdm_received_sack_wiring(changed, self.source, self.defines)

    def test_ofdm_received_observation_does_not_claim_owner_acceptance(self):
        changed = self.common.replace(
            "(int)generation_ack_resolve_target(payload[0], cumulative_ack_enabled)",
            "(int)resolve_and_validate_generation_ack(payload[0], nframes, true, true, true, true, nullptr)", 1)
        self.assertNotEqual(changed, self.common)
        with self.assertRaises(AssertionError):
            assert_ofdm_received_sack_wiring(changed, self.source, self.defines)

    def test_canonical_wrap_and_legacy_identity_source_contract(self):
        assert_ofdm_received_sack_wiring(self.common, self.source, self.defines)
        # The exact helper expression above establishes these unsigned cases.
        self.assertEqual((255 + 1) & 0xFF, 0)
        self.assertEqual((6 + 1) & 0xFF, 7)
        helper = block_after(cpp_code(self.defines), r"inline\s+unsigned\s+char\s+"
                             r"generation_ack_resolve_target\s*\([^)]*\)\s*\{")
        self.assertTrue(compact(helper).endswith("returnwire_field;"))


if __name__ == "__main__":
    unittest.main()
