import io
import socket
import threading
import time

import arq_realaudio as arq


def test_control_disconnected_is_a_real_peer_release_endpoint():
    client, server = socket.socketpair()
    state = arq.State()
    state.cmd_connected = True
    state.rsp_connected = True
    state.connected = True
    stop = threading.Event()
    log = io.StringIO()
    started = time.monotonic()
    reader = threading.Thread(
        target=arq.control_output,
        args=(client, "CMD", log, started, state, stop))
    reader.start()
    server.sendall(b"BUFFER 0\rDISCONNECTED\r")
    deadline = time.monotonic() + 2.0
    while "CMD" not in state.disconnected_at_by_peer and time.monotonic() < deadline:
        time.sleep(0.01)
    stop.set()
    server.close()
    reader.join(timeout=2)
    client.close()

    assert state.disconnected_at_by_peer["CMD"] >= 0
    assert state.disconnect_source_by_peer["CMD"] == "control:DISCONNECTED"
    assert state.cmd_connected is False
    assert "[CMD-CTRL] DISCONNECTED" in log.getvalue()
