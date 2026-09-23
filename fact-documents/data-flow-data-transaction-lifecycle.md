# DATA transaction lifecycle

## Invariant

An accepted host DATA connection belongs to one radio session. Before a radio teardown resets stream identity, Mercury closes that DATA peer and purges every queued byte owned by the transaction. No byte accepted by the old transaction may be transmitted or delivered in a later radio session.

The command socket remains connected and reports the established `DISCONNECTED` status. The DATA server listener remains open so the application can establish a new transaction.

## Cross-layer audit

### 1. Producers

- `process_main()` accepts host bytes from `tcp_socket_data` into `fifo_buffer_tx`.
- The commander moves transmit bytes into `messages_tx`, `fifo_buffer_backup`, and the retransmit queue.
- The responder moves decoded bytes through `messages_rx` or `messages_rx_prev` into `fifo_buffer_rx`; socket backpressure may leave a tail in `rx_deliver_pending`.
- Radio teardown callers converge on `reset_session_state()`. `commander_clean_reconnect()` is the implicit-reconnect entry point that previously restored backup bytes after that reset.

### 2. Consumers

- Commander data builders consume `fifo_buffer_tx`, `fifo_buffer_backup`, `messages_tx`, and the retransmit queue for RF transmission.
- `process_buffer_data_responder()` consumes `fifo_buffer_rx` and `rx_deliver_pending` into the host DATA connection.
- `process_main()` accepts a new DATA peer after `cl_tcp_socket::close_connection()` returns the server socket to listening state.

### 3. Valid states

- `IDLE` or `LISTENING`: a DATA peer may be pre-opened for the next radio session. Idle initialization must not close it.
- Any other link state entering `reset_session_state()`: a radio attempt or established session is ending. The current DATA peer and its byte stores must be terminalized.
- After terminalization: the DATA peer is not `TCP_STATUS_ACCEPTED`; transmit, backup, receive, pending-tail, message-slot, and retransmit stores contain no old application bytes.

### 4. Required invariants

- DATA EOF/reset is observable before a later session can deliver application bytes.
- A zero delivered high-water is not permission to preserve the DATA transaction.
- `reset_session_state()` may re-anchor radio and stream cursors only after the old transaction's byte stores are purged.
- A new session starts from a new DATA connection and a new application write.
- `LISTEN ON` remains compatible with hosts that open DATA before enabling the listener.

### 5. Change made

- `terminalize_data_transaction_on_radio_teardown()` closes the accepted DATA peer while retaining the server listener, flushes all three FIFOs and the pending delivery tail, frees TX/current-RX/previous-RX message ownership, and clears the retransmit queue.
- `reset_session_state()` calls that helper for every non-idle/non-listening session boundary after the L1 journal receives its terminal ownership event.
- `commander_clean_reconnect()` no longer copies backup bytes across the boundary.
- `MERCURY_DATA_EPOCH_CLOSE_DEFEAT=1` exists only as the fail-before test arm and restores the old persistent/suffix behavior.
