#!/usr/bin/env python3
# CONNECT-SEED FUSION patcher: fold the connect-evidenced seed (DUTY-R robust exit)
# into the SWITCH_BANDWIDTH frame so the RSP loads it on the deferred WB switch and the
# CMD on the ACK, collapsing the separate robust SET_CONFIG cross (slice D, ~10.4 s).
# Default-ON; MERCURY_CONNECT_FUSE_DEFEAT / master MERCURY_DUTY_FASTSTART_DEFEAT restores BASE.
import sys, io

ROOT = "/dev/shm/connlever"
T = "\t"

def rd(p):
    with io.open(p, "r", encoding="utf-8", newline="") as f:
        return f.read()

def wr(p, s):
    with io.open(p, "w", encoding="utf-8", newline="") as f:
        f.write(s)

def repl(s, old, new, n_expect, tag):
    c = s.count(old)
    if c != n_expect:
        raise SystemExit("PATCH-FAIL [%s]: expected %d occurrence(s) of anchor, found %d" % (tag, n_expect, c))
    return s.replace(old, new, n_expect)

def repl_after(s, anchor, old, new, tag):
    i = s.find(anchor)
    if i < 0:
        raise SystemExit("PATCH-FAIL [%s]: anchor not found" % tag)
    j = s.find(old, i)
    if j < 0:
        raise SystemExit("PATCH-FAIL [%s]: old not found after anchor" % tag)
    return s[:j] + new + s[j+len(old):]

def B(*lines):
    # each line = (ntabs, text); returns tab-indented block with trailing newlines
    return "".join(T*n + t + "\n" for (n, t) in lines)

# ---------------------------------------------------------------- arq.h
p = ROOT + "/include/datalink_layer/arq.h"
s = rd(p)

# Edit 1: member decls after duty_palt_defeat
old = "  bool duty_palt_defeat;\n"
new = old + (
"  // CONNECT-SEED FUSION (climb-duty, connect floor) - fold the connect-evidenced seed\n"
"  // (DUTY-R robust exit) into the SWITCH_BANDWIDTH frame so the RSP loads it on the WB\n"
"  // switch and the CMD on the ACK, collapsing the separate ~10 s robust SET_CONFIG cross.\n"
"  // Ships DEFAULT-ON. MERCURY_CONNECT_FUSE_DEFEAT=1 (or master MERCURY_DUTY_FASTSTART_DEFEAT=1)\n"
"  // restores the two-frame cross so the fire-proof runs FIX vs DEFEAT on ONE binary.\n"
"  bool connect_fuse_defeat;\n"
"  int connect_fuse_seed_tx;   // CMD: seed embedded in the SWITCH_BANDWIDTH we sent (CONFIG_NONE=none)\n"
"  int connect_fuse_seed_rx;   // RSP: seed carried by the SWITCH_BANDWIDTH we received (CONFIG_NONE=none)\n"
)
s = repl(s, old, new, 1, "arq.h-members")

# Edit 2: pure helpers before robust_connect_exit_target
old = "  int robust_connect_exit_target() const {\n"
new = (
"  // CONNECT-SEED FUSION gate + pure seed selector. connect_fuse_active(): the fusion is\n"
"  // live unless defeated. connect_fuse_seed_select(connect_seed): the seed the\n"
"  // SWITCH_BANDWIDTH carries - the max (by ladder index) of the caller's connect_seed and\n"
"  // the connect-evidenced robust exit target. CONFIG_NONE when defeated or no seed applies.\n"
"  // Pure: production (the WB-upgrade queue) and the directed test drive the SAME logic.\n"
"  bool connect_fuse_active() const { return !connect_fuse_defeat; }\n"
"  int connect_fuse_seed_select(int connect_seed) const {\n"
"    if(!connect_fuse_active()) return CONFIG_NONE;\n"
"    int fs = connect_seed;\n"
"    int rce = robust_connect_exit_target();\n"
"    if(rce != CONFIG_NONE &&\n"
"       (fs == CONFIG_NONE || config_ladder_index(rce) > config_ladder_index(fs)))\n"
"      fs = rce;\n"
"    return fs;\n"
"  }\n"
) + old
s = repl(s, old, new, 1, "arq.h-helpers")

# Edit 3: test + helper decls after test_climb_confirm_batch decl
old = "  int test_climb_confirm_batch();\n"
new = old + (
"  int test_connect_fuse();\n"
"  void apply_connect_seed_cross(int seed_cfg);\n"
)
s = repl(s, old, new, 1, "arq.h-testdecl")
wr(p, s)
print("arq.h OK")

# ---------------------------------------------------------------- arq_common.cc
p = ROOT + "/source/datalink_layer/arq_common.cc"
s = rd(p)
old = (
T+T+'const char* dp = std::getenv("MERCURY_DUTY_PALT_DEFEAT");\n'
+T+T+'duty_palt_defeat = duty_master_defeat || (dp && *dp && atoi(dp) != 0);\n'
)
new = old + (
T+T+'// CONNECT-SEED FUSION (climb-duty, connect floor) - ships DEFAULT-ON.\n'
+T+T+'// MERCURY_CONNECT_FUSE_DEFEAT=1 (or master MERCURY_DUTY_FASTSTART_DEFEAT=1) restores the\n'
+T+T+'// two-frame SWITCH_BANDWIDTH+SET_CONFIG cross so the fire-proof runs FIX vs DEFEAT on ONE\n'
+T+T+'// binary. Env-latched ONCE here (production path) like the R / P-alt knobs above.\n'
+T+T+'const char* cf = std::getenv("MERCURY_CONNECT_FUSE_DEFEAT");\n'
+T+T+'connect_fuse_defeat = duty_master_defeat || (cf && *cf && atoi(cf) != 0);\n'
+T+T+'connect_fuse_seed_tx = CONFIG_NONE;\n'
+T+T+'connect_fuse_seed_rx = CONFIG_NONE;\n'
)
s = repl(s, old, new, 1, "arq_common-latch")
wr(p, s)
print("arq_common.cc OK")

# ---------------------------------------------------------------- arq_commander.cc
p = ROOT + "/source/datalink_layer/arq_commander.cc"
s = rd(p)

# Edit 5: WB-upgrade embed
old = (
T*5+"add_message_control(SWITCH_BANDWIDTH);\n"
+T*5+"this->connection_status=TRANSMITTING_CONTROL;\n"
)
new = (
T*5+"add_message_control(SWITCH_BANDWIDTH);\n"
+T*5+"// CONNECT-SEED FUSION: the connect-evidenced seed is deterministic here, so\n"
+T*5+"// embed it in the SWITCH_BANDWIDTH frame (data[2]) - the RSP loads it on the\n"
+T*5+"// deferred WB switch and the CMD on the ACK, collapsing the separate robust\n"
+T*5+"// SET_CONFIG cross (~10 s at CONFIG_100 WB). Defeated -> length stays 2 (BASE).\n"
+T*5+"connect_fuse_seed_tx = CONFIG_NONE;\n"
+T*5+"{\n"
+T*6+"int fs = connect_fuse_seed_select(connect_seed_target());\n"
+T*6+"if(fs != CONFIG_NONE && messages_control.status != FREE &&\n"
+T*6+"   messages_control.data[0] == SWITCH_BANDWIDTH)\n"
+T*6+"{\n"
+T*7+"messages_control.data[2] = (char)fs;\n"
+T*7+"messages_control.length = 3;\n"
+T*7+"connect_fuse_seed_tx = fs;\n"
+T*7+'printf("[BW-NEG] FUSE: embedding connect-seed config %d in SWITCH_BANDWIDTH (collapse SET_CONFIG cross)\\n", fs);\n'
+T*7+"fflush(stdout);\n"
+T*6+"}\n"
+T*5+"}\n"
+T*5+"this->connection_status=TRANSMITTING_CONTROL;\n"
)
s = repl(s, old, new, 1, "cmd-embed")

# Edit 6: fused apply branch (disambiguate to the SWITCH_BANDWIDTH-accepted handler)
anchor = "else if(this->link_status==CONNECTED && messages_control.data[0]==SWITCH_BANDWIDTH)"
old = (
T*3+"{\n"
+T*4+"int seed_cfg = connect_seed_target();\n"
)
new = (
T*3+"if(connect_fuse_seed_tx != CONFIG_NONE)\n"
+T*3+"{\n"
+T*4+"// CONNECT-SEED FUSION apply (CMD): the RSP loaded this seed as part of the WB\n"
+T*4+"// switch (carried in the SWITCH_BANDWIDTH frame), so apply it here too via the\n"
+T*4+"// SAME cross steps - the separate SET_CONFIG frame airtime is eliminated.\n"
+T*4+"int seed = connect_fuse_seed_tx;\n"
+T*4+"connect_fuse_seed_tx = CONFIG_NONE;\n"
+T*4+"apply_connect_seed_cross(seed);\n"
+T*3+"}\n"
+T*3+"else\n"
+T*3+"{\n"
+T*4+"int seed_cfg = connect_seed_target();\n"
)
s = repl_after(s, anchor, old, new, "cmd-fused-apply")

# Edit 7: helper definition before test_robust_connect_exit
old = "int cl_arq_controller::test_robust_connect_exit()\n"
helper = B(
(0, "void cl_arq_controller::apply_connect_seed_cross(int seed_cfg)"),
(0, "{"),
(1, "// CONNECT-SEED FUSION apply (CMD). Load the seed config the RSP already loaded as part"),
(1, "// of the fused SWITCH_BANDWIDTH switch, running the SAME apply steps the SET_CONFIG-ACK"),
(1, "// cross runs (gearshift-timer reset, geometry load, control-turnaround guard, per-config"),
(1, "// TX re-chunk) MINUS the eliminated SET_CONFIG frame airtime. Mirrors the proven"),
(1, "// SET_CONFIG-ACK apply body; the original branch is left untouched."),
(1, "gear_shift_timer.stop();"),
(1, "gear_shift_timer.reset();"),
(1, "negotiated_configuration = seed_cfg;"),
(1, "data_configuration = seed_cfg;"),
(1, "if(data_configuration != current_configuration)"),
(1, "{"),
(2, "messages_control_backup();"),
(2, "load_configuration(data_configuration, PHYSICAL_LAYER_ONLY, YES);"),
(2, "messages_control_restore();"),
(2, 'printf("[GEARSHIFT] FUSED connect-seed loaded config %d (SET_CONFIG cross collapsed)\\n", data_configuration);'),
(2, "fflush(stdout);"),
(2, "// R1-rescope: arm the post-control-turnaround wait so the first data frame does not"),
(2, "// key inside the peer's PHY-reinit window (same as the SET_CONFIG cross)."),
(2, "arm_control_turnaround_guard();"),
(2, "if(phy_reinit_settle_us > 0)"),
(2, "{"),
(3, "if(arq_sim_inproc_active())"),
(4, "pumped_settle_wait(phy_reinit_settle_us / 1000);"),
(3, "else"),
(4, "usleep(phy_reinit_settle_us);"),
(2, "}"),
(2, "// Re-fill TX messages for the new config's message sizes (per-config re-chunk)."),
(2, "for(int i=0;i<nMessages;i++)"),
(3, "messages_tx[i].status=FREE;"),
(2, "int data_read_size;"),
(2, "for(int i=0;i<get_nTotal_messages();i++)"),
(2, "{"),
(3, "data_read_size=fifo_buffer_backup.pop(message_TxRx_byte_buffer,max_data_length+max_header_length);"),
(3, "if(data_read_size!=0)"),
(4, "fifo_buffer_tx.push(message_TxRx_byte_buffer,data_read_size);"),
(3, "else"),
(4, "break;"),
(2, "}"),
(2, "fifo_buffer_backup.flush();"),
(1, "}"),
(1, "this->connection_status=TRANSMITTING_DATA;"),
(0, "}"),
(0, ""),
)
new = helper + old
s = repl(s, old, new, 1, "cmd-helper-def")

# Edit 8: test function before test_climb_confirm_batch
old = "int cl_arq_controller::test_climb_confirm_batch()\n"
testfn = B(
(0, "int cl_arq_controller::test_connect_fuse()"),
(0, "{"),
(1, "// CONNECT-SEED FUSION - drives the REAL pure selector connect_fuse_seed_select() (the"),
(1, "// SAME method the WB-upgrade queue calls to embed the seed in SWITCH_BANDWIDTH)."),
(1, "// PASS-AFTER (fusion live): at the robust connect floor it returns CONFIG_0 -> the seed"),
(1, "// is carried by the switch frame and the separate SET_CONFIG cross is collapsed."),
(1, "// FAIL-BEFORE (connect_fuse_defeat / MERCURY_CONNECT_FUSE_DEFEAT): CONFIG_NONE -> the"),
(1, "// two-frame cross is retained (BASE). GUARDS: R defeated / already-OFDM -> CONFIG_NONE."),
(1, "int failed = 0;"),
(1, "auto check = [&](bool cond, const char* name, int got, int want) {"),
(2, 'if(cond) { printf("[TEST-CONNECT-FUSE] PASS: %s (got=%d want=%d)\\n", name, got, want); }'),
(2, 'else { printf("[TEST-CONNECT-FUSE] FAIL: %s (got=%d want=%d)\\n", name, got, want); failed++; }'),
(2, "fflush(stdout);"),
(1, "};"),
(1, "int saved_cfg  = current_configuration;"),
(1, "int saved_ceil = supershift_proven_ceiling;"),
(1, "bool saved_fd  = connect_fuse_defeat;"),
(1, "bool saved_rd  = duty_r_defeat;"),
(0, ""),
(1, "// FAIL-BEFORE: fusion defeated at the robust floor -> NO seed carried (two-frame cross)"),
(1, "connect_fuse_defeat = true;  duty_r_defeat = false;"),
(1, "current_configuration = ROBUST_0;  supershift_proven_ceiling = -1;"),
(1, "int fb = connect_fuse_seed_select(CONFIG_NONE);"),
(1, 'check(fb == CONFIG_NONE, "FAIL-BEFORE: fusion defeated at ROBUST_0 -> NO seed (SET_CONFIG cross retained)", fb, CONFIG_NONE);'),
(0, ""),
(1, "// PASS-AFTER: fusion live at the robust floor -> carry CONFIG_0 in the switch frame"),
(1, "connect_fuse_defeat = false;  duty_r_defeat = false;"),
(1, "current_configuration = ROBUST_0;  supershift_proven_ceiling = -1;"),
(1, "int pa = connect_fuse_seed_select(CONFIG_NONE);"),
(1, 'check(pa == CONFIG_0, "PASS-AFTER: fusion live at ROBUST_0 -> carry CONFIG_0 (collapse cross)", pa, CONFIG_0);'),
(0, ""),
(1, "// GUARD: R defeated -> no robust-exit seed even with fusion live -> CONFIG_NONE"),
(1, "connect_fuse_defeat = false;  duty_r_defeat = true;"),
(1, "current_configuration = ROBUST_0;  supershift_proven_ceiling = -1;"),
(1, "int g1 = connect_fuse_seed_select(CONFIG_NONE);"),
(1, 'check(g1 == CONFIG_NONE, "GUARD R defeated -> no seed to fuse", g1, CONFIG_NONE);'),
(0, ""),
(1, "// GUARD: already OFDM (not robust) -> robust exit NONE -> CONFIG_NONE"),
(1, "connect_fuse_defeat = false;  duty_r_defeat = false;"),
(1, "current_configuration = CONFIG_0;  supershift_proven_ceiling = -1;"),
(1, "int g2 = connect_fuse_seed_select(CONFIG_NONE);"),
(1, 'check(g2 == CONFIG_NONE, "GUARD already-OFDM (CONFIG_0) -> nothing to fuse", g2, CONFIG_NONE);'),
(0, ""),
(1, "current_configuration     = saved_cfg;"),
(1, "supershift_proven_ceiling = saved_ceil;"),
(1, "connect_fuse_defeat       = saved_fd;"),
(1, "duty_r_defeat             = saved_rd;"),
(1, 'printf("[TEST-CONNECT-FUSE] %s (%d failures)\\n", failed==0 ? "ALL PASS" : "FAILURES PRESENT", failed);'),
(1, "fflush(stdout);"),
(1, "return failed == 0 ? 0 : 1;"),
(0, "}"),
(0, ""),
)
new = testfn + old
s = repl(s, old, new, 1, "cmd-testfn")
wr(p, s)
print("arq_commander.cc OK")

# ---------------------------------------------------------------- arq_responder.cc
p = ROOT + "/source/datalink_layer/arq_responder.cc"
s = rd(p)

# Edit 9: accept read data[2]
old = (
T*4+"wb_upgrade_pending = true;\n"
+T*4+"connection_status = ACKNOWLEDGING_CONTROL;\n"
)
new = (
T*4+"wb_upgrade_pending = true;\n"
+T*4+"// CONNECT-SEED FUSION: a length>=3 SWITCH_BANDWIDTH carries a seed config in data[2].\n"
+T*4+"// Stage it so the deferred WB switch (after this ACK) loads it directly, collapsing\n"
+T*4+"// the separate SET_CONFIG cross. Defeated peers send length 2 -> no seed (BASE).\n"
+T*4+"connect_fuse_seed_rx = CONFIG_NONE;\n"
+T*4+"if(!connect_fuse_defeat && messages_control.length >= 3)\n"
+T*4+"{\n"
+T*5+"int s_seed = (int)(unsigned char)messages_control.data[2];\n"
+T*5+"if(is_ofdm_config(s_seed) || is_robust_config(s_seed))\n"
+T*5+"{\n"
+T*6+"connect_fuse_seed_rx = s_seed;\n"
+T*6+'printf("[BW-NEG] FUSE: SWITCH_BANDWIDTH carries connect-seed config %d\\n", s_seed);\n'
+T*6+"fflush(stdout);\n"
+T*5+"}\n"
+T*4+"}\n"
+T*4+"connection_status = ACKNOWLEDGING_CONTROL;\n"
)
s = repl(s, old, new, 1, "rsp-accept")

# Edit 10: deferred WB switch load
old = (
T*3+"switch_narrowband_mode(NO);\n"
+T*3+"// After WB config loads, the NB ftr=264 is still active but the\n"
)
new = (
T*3+"switch_narrowband_mode(NO);\n"
+T*3+"// CONNECT-SEED FUSION: load the seed config carried by the SWITCH_BANDWIDTH frame\n"
+T*3+"// now (after the WB switch), collapsing the separate SET_CONFIG cross. Same\n"
+T*3+"// load_configuration() + ring setup the SET_CONFIG cross runs; the ftr reset below\n"
+T*3+"// then reflects the loaded config geometry.\n"
+T*3+"if(connect_fuse_seed_rx != CONFIG_NONE)\n"
+T*3+"{\n"
+T*4+"data_configuration = connect_fuse_seed_rx;\n"
+T*4+"connect_fuse_seed_rx = CONFIG_NONE;\n"
+T*4+"if(data_configuration != current_configuration &&\n"
+T*4+"   (is_ofdm_config(data_configuration) || is_robust_config(data_configuration)))\n"
+T*4+"{\n"
+T*5+"load_configuration(data_configuration, PHYSICAL_LAYER_ONLY, YES);\n"
+T*5+"if(inband_rate_feature_enabled() && is_ofdm_config(data_configuration))\n"
+T*6+"inband_finalize_ofdm_adopt_ring(data_configuration);\n"
+T*5+'printf("[BW-NEG] FUSE: loaded connect-seed config %d on WB switch (SET_CONFIG cross collapsed)\\n", data_configuration);\n'
+T*5+"fflush(stdout);\n"
+T*4+"}\n"
+T*3+"}\n"
+T*3+"// After WB config loads, the NB ftr=264 is still active but the\n"
)
s = repl(s, old, new, 1, "rsp-deferred-load")
wr(p, s)
print("arq_responder.cc OK")

# ---------------------------------------------------------------- main.cc register test
p = ROOT + "/source/main.cc"
s = rd(p)
old = (
"                cl_arq_controller test_pa;\n"
"                failed += test_pa.test_climb_confirm_batch();\n"
"            }\n"
)
new = old + (
"            // CONNECT-SEED FUSION (climb-duty, connect floor) - the connect-evidenced seed\n"
"            // is folded into the SWITCH_BANDWIDTH frame so the RSP loads it on the WB switch\n"
"            // and the CMD on the ACK, collapsing the separate robust SET_CONFIG cross. Drives\n"
"            // the REAL selector connect_fuse_seed_select(). PASS-AFTER carries CONFIG_0;\n"
"            // FAIL-BEFORE (connect_fuse_defeat / MERCURY_CONNECT_FUSE_DEFEAT) CONFIG_NONE.\n"
"            {\n"
"                cl_arq_controller test_cf;\n"
"                failed += test_cf.test_connect_fuse();\n"
"            }\n"
)
s = repl(s, old, new, 1, "main-register")
wr(p, s)
print("main.cc OK")
print("ALL PATCHES APPLIED")
