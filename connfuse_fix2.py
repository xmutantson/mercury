#!/usr/bin/env python3
# FIX for the RSP desync: the RSP's recovered messages_control.length for SWITCH_BANDWIDTH is 2
# (code-derived, not the transmitted 3), so the `length >= 3` gate blocked the seed read -> RSP
# loaded WB CONFIG_100 while the CMD loaded cfg0 -> desync -> BREAK. SET_CONFIG proves data[2] is
# buffer-delivered on a length=3 frame and is read UNCONDITIONALLY. So: (1) RSP reads data[2] with
# no length gate; (2) CMD always occupies data[2] on the FIX path (seed, or 0xFF sentinel) so the
# RSP never reads a stale byte; BASE (defeat) leaves the frame 2-byte (byte-identical DUTY base).
import io
ROOT = "/dev/shm/connlever"; T = "\t"
def rd(p):
    with io.open(p,"r",encoding="utf-8",newline="") as f: return f.read()
def wr(p,s):
    with io.open(p,"w",encoding="utf-8",newline="") as f: f.write(s)
def repl(s,old,new,tag):
    if s.count(old)!=1: raise SystemExit("FAIL[%s]: found %d"%(tag,s.count(old)))
    return s.replace(old,new,1)

# --- CMD embed: always occupy data[2] on the FIX path (connect_fuse_active), sentinel 0xFF if no seed
p = ROOT+"/source/datalink_layer/arq_commander.cc"; s = rd(p)
old = (
T*5+"connect_fuse_seed_tx = CONFIG_NONE;\n"
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
)
new = (
T*5+"connect_fuse_seed_tx = CONFIG_NONE;\n"
+T*5+"if(connect_fuse_active() && messages_control.status != FREE &&\n"
+T*5+"   messages_control.data[0] == SWITCH_BANDWIDTH)\n"
+T*5+"{\n"
+T*6+"int fs = connect_fuse_seed_select(connect_seed_target());\n"
+T*6+"// Always occupy data[2] on the FIX path so the RSP never reads a stale byte: a valid\n"
+T*6+"// config = the fused seed; 0xFF = no seed (fall back to the two-frame cross). length=3\n"
+T*6+"// matches SET_CONFIG so data[2] is buffer-delivered to the RSP (its length field is not).\n"
+T*6+"messages_control.data[2] = (char)(fs != CONFIG_NONE ? fs : 0xFF);\n"
+T*6+"messages_control.length = 3;\n"
+T*6+"if(fs != CONFIG_NONE)\n"
+T*6+"{\n"
+T*7+"connect_fuse_seed_tx = fs;\n"
+T*7+'printf("[BW-NEG] FUSE: embedding connect-seed config %d in SWITCH_BANDWIDTH (collapse SET_CONFIG cross)\\n", fs);\n'
+T*7+"fflush(stdout);\n"
+T*6+"}\n"
+T*5+"}\n"
)
s = repl(s, old, new, "cmd-embed-fix"); wr(p, s); print("arq_commander.cc OK")

# --- RSP accept: drop the length>=3 gate; read data[2] unconditionally (like SET_CONFIG)
p = ROOT+"/source/datalink_layer/arq_responder.cc"; s = rd(p)
old = T*4+"if(!connect_fuse_defeat && messages_control.length >= 3)\n"
new = (
T*4+"// data[2] is buffer-copied on RX regardless of the recovered length field (SET_CONFIG\n"
+T*4+"// reads data[2] the same way). A FIX-arm CMD always sets data[2] (seed or 0xFF sentinel).\n"
+T*4+"if(!connect_fuse_defeat)\n"
)
s = repl(s, old, new, "rsp-accept-fix"); wr(p, s); print("arq_responder.cc OK")
print("FIX2 APPLIED")
