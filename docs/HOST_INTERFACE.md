# Mercury host interface

Mercury exposes two TCP sockets. The base port is the command socket and the next port is the DATA socket. Commands and status tokens on the command socket end with carriage return. The DATA socket is an unframed byte stream and carries no status text.

## DATA transaction lifetime

One accepted DATA connection is one application transaction. Its lifetime is bounded by one radio session.

Every radio-session teardown terminates that transaction, including a clean peer disconnect, link timeout, exhausted recovery, local abort, failed handshake after data was accepted, an automatic reconnect, or an active-session `LISTEN ON`, `LISTEN OFF`, or replacement `CONNECT` command. Mercury performs these actions before a later session may use application data:

1. Close the accepted DATA connection. The host observes EOF or a connection-reset error. Mercury keeps the DATA listening socket open.
2. Discard application bytes queued for transmit, in-flight backup bytes, received bytes not yet delivered, and a pending short-write tail.
3. Report exactly one `DISCONNECTED` status on the command socket. A failed pending inbound or outbound attempt may report `CANCELPENDING` first. A replacement session does not enter `CONNECTING` until this link-end token has been emitted.

EOF/reset on DATA is the terminal DATA signal. Mercury does not put a text event into the DATA byte stream because such an event would be indistinguishable from application content.

The application owns resume. After the terminal signal, it opens a new DATA connection and writes again from an offset established by its application protocol. Mercury does not preserve or replay accepted-but-unconfirmed bytes across the radio teardown. A fresh radio session cannot deliver bytes to the old DATA connection.

This rule applies even when the previous session delivered zero bytes. Keeping the old connection and later delivering a suffix is forbidden.

`LISTEN ON` while already idle is initialization, not a teardown. It may preserve an empty DATA connection opened in advance for the next session. The same command received while a link or connection attempt is active first applies the teardown rule above.

## Relationship to the VARA host convention

The VARA Native TNC command document defines TCP 8300 as the command port and TCP 8301 as the byte-stream bridge after `CONNECTED`. It uses `DISCONNECTED` to report that the radio link ended and defines no modem-level application offset or resume command. An application must therefore retain its own buffer and decide what to send in a later session. Mercury mirrors the `CONNECTED`/`DISCONNECTED` command vocabulary and the application-owned buffer boundary. Mercury also closes the DATA peer at that boundary so the end of a DATA transaction cannot be mistaken for continuation.

Reference: [VARA Protocol Native TNC Commands, 13 February 2022](https://github.com/n8jja/Pat-Vara/raw/main/VARA%20Protocol%20Native%20TNC%20Commands.pdf).

## Opening the next transaction

After DATA EOF/reset and command `DISCONNECTED`, connect a fresh DATA socket. A listener may accept that socket while Mercury is idle or reconnecting, but bytes on it belong only to the next radio session. Send a new `CONNECT` command when the application is the caller, or leave `LISTEN ON` active when it is the receiver. The new session starts at Mercury's fresh wire-batch origin, so the application's new offset-zero write is eligible for delivery. Do not continue writing through a socket that belonged to the ended session.
