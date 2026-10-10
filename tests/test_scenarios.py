"""Scenario tests for SDLView with a simulated ECU on a fake serial port (no hardware needed).

Run from the repository root:
    python tests/test_scenarios.py

Covers the safety-relevant paths of the idle command (lost acks, unconfirmed stops, closing
the window) and robustness against partial replies, a pulled cable and bad user input.
Exits with code 1 if any check fails.
"""

import pathlib
import queue
import sys
import time
import tkinter as tk
import types

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import sdlview  # noqa: E402

REAL_SLEEP = time.sleep


class FakeECU:
    """Stands in for serial.Serial: echoes every write, then answers like ECU 1453.

    Class attributes switch on failure modes for the next scenario (see reset()).
    """

    log: list[bytes] = []
    drop_idle_ack = False   # the idle frame is applied, but its ack is lost
    drop_stop_acks = 0      # number of stop frames that get no ack
    partial_reply = False   # the next 0x13 reply is only one byte
    die_after = None        # raise SerialException after N writes (cable pulled)

    def __init__(self, *_args, **_kwargs):
        self.buf = b""
        self.writes = 0
        self.target_rpm = None

    def reset_input_buffer(self):
        self.buf = b""

    def close(self):
        pass

    def write(self, msg: bytes) -> int:
        self.writes += 1
        if FakeECU.die_after is not None and self.writes > FakeECU.die_after:
            raise sdlview.serial.SerialException("device disconnected")
        FakeECU.log.append(msg)
        REAL_SLEEP(0.02)  # the real ECU needs ~50-60 ms per request
        self.buf += msg  # echo from the half-duplex cable
        header, data = msg[0], msg[2:-1]
        reply = None
        if header == 0x10:
            reply = bytes([0x14, 0x53])
        elif header == 0x13:
            if FakeECU.partial_reply:
                FakeECU.partial_reply = False
                self.buf += bytes([header])
                return len(msg)
            values = {a: 0 for a in range(0x30)}
            rpm_raw = int(900 * 5.1)
            values[0x04], values[0x05] = rpm_raw // 256, rpm_raw % 256
            values[0x06] = 154 if self.target_rpm is None else round(self.target_rpm / 7.84375)
            values[0x02] = 0x20  # stored code 23
            reply = bytes(values[a] for a in data)
        elif header == 0x15:
            if data[0] == 0xC0:
                self.target_rpm = data[1] * 7.84375
                if FakeECU.drop_idle_ack:
                    return len(msg)
            elif data == bytes(8):
                if FakeECU.drop_stop_acks > 0:
                    FakeECU.drop_stop_acks -= 1
                    return len(msg)
                self.target_rpm = None
            reply = b""
        if reply is not None:
            body = bytes([header, 3 + len(reply)]) + reply
            self.buf += body + bytes([sdlview.checksum(body)])
        return len(msg)

    def read(self, n: int) -> bytes:
        out, self.buf = self.buf[:n], self.buf[n:]
        return out


# Patch the app: fake serial port, and near-instant waits on error paths.
sdlview.serial.Serial = FakeECU
sdlview.time = types.SimpleNamespace(sleep=lambda _s: REAL_SLEEP(0.001), time=time.time,
                                     perf_counter=time.perf_counter)

results: list[tuple[str, bool]] = []


def check(name: str, ok: bool, detail: str = "") -> None:
    results.append((name, ok))
    print(("PASS " if ok else "FAIL ") + name + (f"  [{detail}]" if detail else ""))


def reset(**failure_modes) -> None:
    FakeECU.log = []
    FakeECU.drop_idle_ack = False
    FakeECU.drop_stop_acks = 0
    FakeECU.partial_reply = False
    FakeECU.die_after = None
    for name, value in failure_modes.items():
        setattr(FakeECU, name, value)


def stop_frames_sent() -> int:
    return sum(1 for m in FakeECU.log if m[0] == 0x15 and m[2:-1] == bytes(8))


def run_link(commands: list, seconds: float = 0.3) -> list:
    """Run a Link against the fake ECU, send commands, stop it, and return all its messages."""
    out: queue.Queue = queue.Queue()
    link = sdlview.Link(out, "FAKE")
    link.start()
    REAL_SLEEP(0.05)
    for cmd in commands:
        link.cmds.put(cmd)
    REAL_SLEEP(seconds)
    link.halt.set()
    link.join(5)
    messages = []
    while not out.empty():
        messages.append(out.get())
    return messages


def last(messages: list, kind: str) -> tuple:
    return [m for m in messages if m[0] == kind][-1]


def run_gui(script) -> None:
    """Start the app against the fake ECU and run script(root, app) once it is connected."""
    root = tk.Tk()
    app = sdlview.App(root)
    app.start_link("FAKE")
    root.after(500, lambda: script(root, app))
    root.mainloop()


# --- link layer ---------------------------------------------------------------------------

reset(drop_idle_ack=True)
msgs = run_link([("idle", 1200)])
check("lost idle ack: stop is still sent on close", stop_frames_sent() >= 1, f"stops={stop_frames_sent()}")
check("lost idle ack: close reports the stop as confirmed", last(msgs, "closed")[3] is False)

reset(drop_stop_acks=2)
msgs = run_link([("idle", 1200), ("stop",)])
check("unacknowledged stop is retried", stop_frames_sent() >= 3, f"stops={stop_frames_sent()}")
state = last(msgs, "idle_state")
check("after the retried stop the idle state is clear", state[1] is None and state[2] is False, str(state))

reset(drop_stop_acks=999)
msgs = run_link([("idle", 1200)], seconds=0.1)
check("stop never acknowledged: close reports stop_failed", last(msgs, "closed")[3] is True)

reset(partial_reply=True)
msgs = run_link([], seconds=0.2)
samples = sum(1 for m in msgs if m[0] == "sample")
check("one-byte partial reply does not end the session",
      samples > 0 and last(msgs, "closed")[2] is False, f"samples={samples}")

reset(die_after=6)
msgs = run_link([("idle", 1200)], seconds=0.2)
closed = last(msgs, "closed")
check("pulled cable: closed with an error", closed[2] is True and "lost" in closed[1].lower(), str(closed))

# --- GUI ------------------------------------------------------------------------------------

reset()
seen: dict = {}


def empty_spinbox_and_close(root, app):
    app.idle_confirmed = True
    app.idle_rpm.set(1300)
    app.send_idle()
    app.ect_limit.set("")  # user clears the spinbox (used to stop the poll loop)

    def inspect():
        seen["samples"] = len(app.sample_times)
        seen["state"] = app.idle_state_var.get()
        app.on_close()  # must wait for the link thread to send the stop

    root.after(800, inspect)


run_gui(empty_spinbox_and_close)
check("poll loop keeps running with an empty spinbox", seen["samples"] > 2, str(seen))
check("active target is shown in the GUI", seen["state"].startswith("Target active"), seen["state"])
check("closing sends the stop frame before exit", stop_frames_sent() >= 1, f"stops={stop_frames_sent()}")

reset(drop_stop_acks=999)
events: list = []
sdlview.messagebox.showwarning = lambda title, _text: events.append(("warning", title))


def close_with_unconfirmed_stop(root, app):
    original_destroy = root.destroy
    root.destroy = lambda: (events.append(("destroy",)), original_destroy())
    app.idle_confirmed = True
    app.send_idle()
    root.after(600, app.on_close)


run_gui(close_with_unconfirmed_stop)
check("unconfirmed stop on close: warning shown before the window closes",
      events[:2] == [("warning", "Idle command"), ("destroy",)], str(events))
check("unconfirmed stop on close: three attempts", stop_frames_sent() >= 3, f"stops={stop_frames_sent()}")

reset(drop_stop_acks=999)
buttons: dict = {}


def send_blocked_while_stop_pending(root, app):
    app.idle_confirmed = True
    app.send_idle()
    root.after(400, app.send_stop)

    def while_pending():
        buttons["pending"] = str(app.idle_send.cget("state"))
        buttons["label"] = app.idle_state_var.get()
        FakeECU.drop_stop_acks = 0  # let the next retry succeed

        def after_confirmed():
            buttons["after"] = str(app.idle_send.cget("state"))
            app.on_close()

        root.after(800, after_confirmed)

    root.after(1200, while_pending)


run_gui(send_blocked_while_stop_pending)
check("Send target is disabled while a stop is unconfirmed",
      buttons.get("pending") == "disabled" and "NOT CONFIRMED" in buttons.get("label", ""), str(buttons))
check("Send target is enabled again once the stop is confirmed", buttons.get("after") == "normal", str(buttons))

passed = sum(ok for _, ok in results)
print(f"\n{passed}/{len(results)} passed")
sys.exit(0 if passed == len(results) else 1)
