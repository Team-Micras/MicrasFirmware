#!/usr/bin/env python3
"""Talk to Micras over its link (micras_comm protocol v1) from a terminal.

Two transports:
  --ble ADDR   the robot, through `blecheck.exe bridge ADDR - 0` (Windows, called from WSL), which
               paces the writes the way the HM-19 needs them
  --sim        the simulator's monitor bridge (`micras_sim --monitor`), at --url (ws://localhost:8080)

Commands:
  schema                         list every variable with its id, type and access
  read NAME...                   read variables once, whether they stream or not
  write NAME VALUE               write a variable (only the writable ones)
  command CODE|NAME [ARG]        send a command (explore, solve, calibrate, save, reset, stop, resume,
                                 maintain, or a number); maintain takes the procedure: walls, drive,
                                 gyroscope, sensors, polarity
  stream NAME... [--rate HZ] [--seconds S] [--out FILE.csv] [--until-state STATE]
                                 stream up to 16 variables as one group, one CSV row per sample,
                                 with the robot's sequence number and timestamp
  watch NAME... [--rate HZ]      print the values of a group as they arrive

NAME may be a glob, such as 'wall/*'. The schema is cached per hash in ~/.cache/micras_link.
"""

from __future__ import annotations

import argparse
import base64
import csv
import fnmatch
import json
import os
import select
import socket
import struct
import subprocess
import sys
import threading
import time
from pathlib import Path
from urllib.parse import urlparse

BLECHECK = "/mnt/c/Users/cosme/AppData/Local/Temp/micras_ble/blecheck.exe"
DEFAULT_ADDRESS = "B0:10:A0:BF:6B:55"
PROTOCOL_VERSION = 1
MAX_GROUP_VARIABLES = 16
MAX_SAMPLE_BYTES = 193

HELLO, SCHEMA_REQUEST, GROUP_DEFINE, GROUP_ENABLE, CREDIT, WRITE, READ, COMMAND, PING = range(1, 10)
HELLO_ACK, SCHEMA_PAGE, GROUP_ACK = 0x81, 0x82, 0x83
SAMPLE, WRITE_ACK, VALUE, COMMAND_ACK, PONG, LOG, ERROR = 0x85, 0x86, 0x87, 0x88, 0x89, 0x8A, 0x8F

TYPES = {
    0: ("bool", "<?"),
    1: ("u8", "<B"),
    2: ("i8", "<b"),
    3: ("u16", "<H"),
    4: ("i16", "<h"),
    5: ("u32", "<I"),
    6: ("i32", "<i"),
    7: ("u64", "<Q"),
    8: ("i64", "<q"),
    9: ("f32", "<f"),
    10: ("f64", "<d"),
    11: ("blob", None),
}

WRITE_STATUS = ["OK", "NO_SUCH_ID", "READ_ONLY", "NEEDS_IDLE", "WRONG_SIZE"]
COMMAND_RESULT = ["OK", "UNKNOWN", "REFUSED"]
ERROR_CODES = ["UNKNOWN_TYPE", "MALFORMED", "NO_SUCH_GROUP", "GROUP_TOO_LARGE", "NOT_STREAMABLE", "NO_SUCH_VARIABLE"]
SEVERITIES = ["DEBUG", "INFO", "WARNING", "ERROR"]

COMMANDS = {"explore": 0, "solve": 1, "calibrate": 2, "save": 3, "reset": 4, "stop": 5, "resume": 6, "maintain": 7}

PROCEDURES = {"walls": 0, "drive": 1, "gyroscope": 2, "sensors": 3, "polarity": 4}

STATES = [
    "INIT",
    "IDLE",
    "WAIT_FOR_RUN",
    "RUN",
    "PLAN",
    "SAVE",
    "WAIT_FOR_CALIBRATE",
    "CALIBRATE",
    "WAIT_FOR_IDENTIFY",
    "IDENTIFY",
    "WAIT_FOR_GYROSCOPE",
    "CALIBRATE_GYROSCOPE",
    "ERROR",
    "CHECK_SENSORS",
    "CHECK_POLARITY",
]


def fletcher16(data: bytes) -> int:
    low = high = 0

    for byte in data:
        low = (low + byte) % 255
        high = (high + low) % 255

    return high << 8 | low


def cobs_encode(data: bytes) -> bytes:
    out = bytearray([0])
    code_index = 0
    code = 1

    for byte in data:
        if byte == 0:
            out[code_index] = code
            code_index = len(out)
            out.append(0)
            code = 1
            continue

        out.append(byte)
        code += 1

        if code == 0xFF:
            out[code_index] = code
            code_index = len(out)
            out.append(0)
            code = 1

    out[code_index] = code
    return bytes(out)


def cobs_decode(data: bytes) -> bytes | None:
    out = bytearray()
    index = 0

    while index < len(data):
        code = data[index]

        if code == 0 or index + code > len(data):
            return None

        out += data[index + 1 : index + code]
        index += code

        if code < 0xFF and index < len(data):
            out.append(0)

    return bytes(out)


def encode_frame(message_type: int, payload: bytes = b"") -> bytes:
    plain = bytes([message_type]) + payload
    return cobs_encode(plain + struct.pack("<H", fletcher16(plain))) + b"\x00"


def state_name(state_id: int) -> str:
    return STATES[state_id] if 0 <= state_id < len(STATES) else str(state_id)


class BleBridge:
    """The robot, through the native Windows bridge over its standard input and output."""

    def __init__(self, address: str, log, attempts: int = 3):
        self.log = log

        for attempt in range(attempts):
            if self._start(address):
                return

            self.close()

            if attempt + 1 < attempts:
                self.log("[bridge] retrying after a scan, which puts the module in the cache of Windows")
                subprocess.run([BLECHECK, "scan", address, "3"], capture_output=True, timeout=30)

        raise ConnectionError("the bridge did not connect to the robot")

    def _start(self, address: str) -> bool:
        self.process = subprocess.Popen(
            [BLECHECK, "bridge", address, "-", "0"],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            bufsize=0,
        )
        self.ready = threading.Event()
        self.failed = threading.Event()
        threading.Thread(target=self._read_errors, daemon=True).start()

        return self.ready.wait(40) and not self.failed.is_set()

    def _read_errors(self):
        for raw in iter(self.process.stderr.readline, b""):
            line = raw.decode(errors="replace").rstrip("\r\n")
            self.log(f"[bridge] {line}")

            if "notifications" in line:
                if "Success" not in line:
                    self.failed.set()

                self.ready.set()

        self.failed.set()
        self.ready.set()

    def send(self, data: bytes):
        self.process.stdin.write(data)
        self.process.stdin.flush()

    def receive(self, timeout: float) -> bytes:
        ready, _, _ = select.select([self.process.stdout], [], [], timeout)

        if not ready:
            return b""

        data = os.read(self.process.stdout.fileno(), 4096)

        if not data:
            raise ConnectionError("the bridge closed")

        return data

    def close(self):
        try:
            self.process.stdin.close()
        except OSError:
            pass

        try:
            self.process.wait(5)
        except subprocess.TimeoutExpired:
            self.process.kill()


class WebSocketClient:
    """The simulator's monitor bridge: binary WebSocket messages that carry the raw byte stream."""

    def __init__(self, url: str):
        parsed = urlparse(url)
        self.sock = socket.create_connection((parsed.hostname, parsed.port or 80), timeout=5)
        key = base64.b64encode(os.urandom(16)).decode()
        request = (
            f"GET {parsed.path or '/'} HTTP/1.1\r\nHost: {parsed.hostname}:{parsed.port}\r\n"
            f"Upgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Key: {key}\r\n"
            "Sec-WebSocket-Version: 13\r\n\r\n"
        )
        self.sock.sendall(request.encode())
        response = b""

        while b"\r\n\r\n" not in response:
            chunk = self.sock.recv(1024)

            if not chunk:
                raise ConnectionError("the simulator closed the connection during the handshake")

            response += chunk

        header, self.pending = response.split(b"\r\n\r\n", 1)

        if b" 101 " not in header.split(b"\r\n")[0]:
            raise ConnectionError(header.decode(errors="replace"))

        self.sock.settimeout(None)

    def send(self, data: bytes):
        mask = os.urandom(4)
        length = len(data)
        header = bytearray([0x82])

        if length < 126:
            header.append(0x80 | length)
        elif length < 65536:
            header += bytes([0x80 | 126]) + struct.pack(">H", length)
        else:
            header += bytes([0x80 | 127]) + struct.pack(">Q", length)

        self.sock.sendall(bytes(header) + mask + bytes(b ^ mask[i % 4] for i, b in enumerate(data)))

    def _read_exactly(self, size: int) -> bytes:
        while len(self.pending) < size:
            chunk = self.sock.recv(65536)

            if not chunk:
                raise ConnectionError("the simulator closed the connection")

            self.pending += chunk

        data, self.pending = self.pending[:size], self.pending[size:]
        return data

    def receive(self, timeout: float) -> bytes:
        if len(self.pending) < 2:
            ready, _, _ = select.select([self.sock], [], [], timeout)

            if not ready:
                return b""

        first, second = self._read_exactly(2)
        length = second & 0x7F

        if length == 126:
            (length,) = struct.unpack(">H", self._read_exactly(2))
        elif length == 127:
            (length,) = struct.unpack(">Q", self._read_exactly(8))

        mask = self._read_exactly(4) if second & 0x80 else None
        payload = self._read_exactly(length)

        if mask:
            payload = bytes(b ^ mask[i % 4] for i, b in enumerate(payload))

        opcode = first & 0x0F

        if opcode == 0x8:
            raise ConnectionError("the simulator closed the connection")

        if opcode == 0x9:
            self.sock.sendall(bytes([0x8A, 0x80]) + os.urandom(4))
            return b""

        return payload if opcode in (0x0, 0x1, 0x2) else b""

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass


class Variable:
    def __init__(self, identifier: int, type_code: int, access: int, name: str):
        self.id = identifier
        self.type_code = type_code
        self.access = access
        self.name = name

    @property
    def type_name(self) -> str:
        return TYPES[self.type_code][0]

    @property
    def size(self) -> int:
        fmt = TYPES[self.type_code][1]
        return struct.calcsize(fmt) if fmt else 0

    @property
    def streamable(self) -> bool:
        return bool(self.access & 1)

    @property
    def access_text(self) -> str:
        return "".join(flag if self.access & (1 << bit) else "-" for bit, flag in enumerate("swip"))

    def decode(self, data: bytes):
        fmt = TYPES[self.type_code][1]
        return data.hex() if fmt is None else struct.unpack(fmt, data[: self.size])[0]

    def encode(self, text: str) -> bytes:
        fmt = TYPES[self.type_code][1]

        if fmt is None:
            return bytes.fromhex(text)

        if self.type_code == 0:
            return struct.pack(fmt, text.lower() in ("1", "true", "on", "yes"))

        if self.type_code in (9, 10):
            return struct.pack(fmt, float(text))

        return struct.pack(fmt, int(text, 0))


class Session:
    def __init__(self, transport, log=print, credit_interval: float = 0.05):
        self.transport = transport
        self.log = log
        self.buffer = bytearray()
        self.unreturned = 0
        self.credit_interval = credit_interval
        self.last_credit = 0.0
        self.variables: list[Variable] = []
        self.by_name: dict[str, Variable] = {}
        self.handlers = {}
        self.dropped_frames = 0
        self.loop_time_us = 0

    def send(self, message_type: int, payload: bytes = b""):
        self.transport.send(encode_frame(message_type, payload))

    def poll(self, timeout: float = 0.02):
        data = self.transport.receive(timeout)

        if data:
            self.unreturned += len(data)
            self.buffer += data

        while True:
            end = self.buffer.find(0)

            if end < 0:
                break

            encoded = bytes(self.buffer[:end])
            del self.buffer[: end + 1]

            if encoded:
                self._dispatch(encoded)

        now = time.monotonic()

        if self.unreturned and now - self.last_credit >= self.credit_interval:
            self.send(CREDIT, struct.pack("<H", min(self.unreturned, 0xFFFF)))
            self.unreturned = 0
            self.last_credit = now

    def _dispatch(self, encoded: bytes):
        decoded = cobs_decode(encoded)

        if decoded is None or len(decoded) < 3 or fletcher16(decoded[:-2]) != struct.unpack("<H", decoded[-2:])[0]:
            self.dropped_frames += 1
            return

        message_type, payload = decoded[0], decoded[1:-2]

        if message_type == LOG and payload:
            self.log(f"[robot {SEVERITIES[min(payload[0], 3)]}] {payload[1:].decode(errors='replace')}")
        elif message_type == ERROR and len(payload) >= 3:
            code, context = payload[0], struct.unpack("<H", payload[1:3])[0]
            self.log(f"[robot error] {ERROR_CODES[code] if code < len(ERROR_CODES) else code} ({context})")

        handler = self.handlers.get(message_type)

        if handler:
            handler(payload)

    def request(self, message_type: int, payload: bytes, reply_type: int, accept=lambda payload: True,
                timeout: float = 1.5, attempts: int = 3) -> bytes:
        for _ in range(attempts):
            result = []
            self.handlers[reply_type] = lambda reply: result.append(reply) if accept(reply) else None
            self.send(message_type, payload)
            deadline = time.monotonic() + timeout

            while not result and time.monotonic() < deadline:
                self.poll()

            self.handlers.pop(reply_type, None)

            if result:
                return result[0]

        raise TimeoutError(f"no reply of type {reply_type:#04x}")

    def connect(self):
        reply = self.request(HELLO, b"", HELLO_ACK, timeout=2.0, attempts=5)
        version, schema_hash, count, loop_time_us, credit = struct.unpack("<BIHIH", reply[:13])

        if version != PROTOCOL_VERSION:
            raise ConnectionError(f"the robot speaks protocol {version}, this tool {PROTOCOL_VERSION}")

        self.loop_time_us = loop_time_us
        self.unreturned = 0
        self.log(f"connected: {count} variables, schema {schema_hash:08x}, loop {loop_time_us} us, credit {credit}")
        self._load_schema(schema_hash, count)

    def _load_schema(self, schema_hash: int, count: int):
        cache = Path.home() / ".cache" / "micras_link" / f"{schema_hash:08x}.json"

        if cache.exists():
            entries = json.loads(cache.read_text())
        else:
            entries = []

            while len(entries) < count:
                first = len(entries)
                page = self.request(
                    SCHEMA_REQUEST,
                    struct.pack("<H", first),
                    SCHEMA_PAGE,
                    accept=lambda reply, first=first: len(reply) >= 9 and struct.unpack("<H", reply[4:6])[0] == first,
                )
                page_count = page[8]
                index = 9

                for _ in range(page_count):
                    type_code, access, name_size = page[index : index + 3]
                    name = page[index + 3 : index + 3 + name_size].decode()
                    entries.append([type_code, access, name])
                    index += 3 + name_size

                if page_count == 0:
                    raise ConnectionError("the robot sent an empty schema page")

            cache.parent.mkdir(parents=True, exist_ok=True)
            cache.write_text(json.dumps(entries))

        self.variables = [Variable(i, t, a, n) for i, (t, a, n) in enumerate(entries)]
        self.by_name = {variable.name: variable for variable in self.variables}

    def resolve(self, patterns: list[str]) -> list[Variable]:
        chosen = []

        for pattern in patterns:
            matched = [v for v in self.variables if fnmatch.fnmatchcase(v.name, pattern)]

            if not matched:
                raise KeyError(f"no variable matches {pattern}")

            chosen += [v for v in matched if v not in chosen]

        return chosen

    def read(self, variable: Variable):
        reply = self.request(
            READ,
            struct.pack("<H", variable.id),
            VALUE,
            accept=lambda reply: len(reply) >= 2 and struct.unpack("<H", reply[:2])[0] == variable.id,
        )
        return variable.decode(reply[2:])

    def write(self, variable: Variable, text: str) -> str:
        reply = self.request(
            WRITE,
            struct.pack("<H", variable.id) + variable.encode(text),
            WRITE_ACK,
            accept=lambda reply: len(reply) >= 3 and struct.unpack("<H", reply[:2])[0] == variable.id,
        )
        return WRITE_STATUS[reply[2]] if reply[2] < len(WRITE_STATUS) else str(reply[2])

    def command(self, code: int, argument: int = 0) -> str:
        reply = self.request(
            COMMAND,
            struct.pack("<BI", code, argument),
            COMMAND_ACK,
            accept=lambda reply: len(reply) >= 2 and reply[0] == code,
            attempts=1,
            timeout=6.0,
        )
        return COMMAND_RESULT[reply[1]] if reply[1] < len(COMMAND_RESULT) else str(reply[1])

    def define_group(self, group: int, variables: list[Variable], rate: float) -> float:
        if len(variables) > MAX_GROUP_VARIABLES:
            raise ValueError(f"a group holds at most {MAX_GROUP_VARIABLES} variables")

        if sum(v.size for v in variables) > MAX_SAMPLE_BYTES:
            raise ValueError(f"a group holds at most {MAX_SAMPLE_BYTES} bytes of values")

        for variable in variables:
            if not variable.streamable:
                raise ValueError(f"{variable.name} does not stream; read it instead")

        period = max(1, round(1e6 / (rate * self.loop_time_us)))
        payload = struct.pack("<BHB", group, period, len(variables)) + b"".join(
            struct.pack("<H", v.id) for v in variables
        )
        self.request(GROUP_DEFINE, payload, GROUP_ACK, accept=lambda reply: reply[0] == group)
        self.request(GROUP_ENABLE, bytes([group, 1]), GROUP_ACK, accept=lambda reply: reply[0] == group)
        return 1e6 / (period * self.loop_time_us)

    def disable_group(self, group: int):
        try:
            self.request(GROUP_ENABLE, bytes([group, 0]), GROUP_ACK, accept=lambda reply: reply[0] == group)
        except TimeoutError:
            pass

    def on_samples(self, group: int, variables: list[Variable], callback):
        offsets = []
        offset = 7

        for variable in variables:
            offsets.append(offset)
            offset += variable.size

        def handle(payload: bytes):
            if len(payload) < offset or payload[0] != group:
                return

            sequence, timestamp = struct.unpack("<HI", payload[1:7])
            values = [v.decode(payload[o : o + v.size]) for v, o in zip(variables, offsets)]
            callback(sequence, timestamp, values)

        self.handlers[SAMPLE] = handle


def open_session(args) -> Session:
    log = lambda text: print(text, file=sys.stderr, flush=True)
    transport = WebSocketClient(args.url) if args.sim else BleBridge(args.ble, log)
    session = Session(transport, log)

    try:
        session.connect()
    except Exception:
        transport.close()
        raise

    return session


def format_value(variable: Variable, value) -> str:
    if variable.name == "fsm/state":
        return f"{value} ({state_name(value)})"

    if isinstance(value, float):
        return f"{value:.6g}"

    return str(value)


def run_schema(session: Session, args):
    for variable in session.variables:
        print(f"{variable.id:3d}  {variable.type_name:5s} {variable.access_text}  {variable.name}")


def run_read(session: Session, args):
    for variable in session.resolve(args.names):
        print(f"{variable.name} = {format_value(variable, session.read(variable))}")


def run_write(session: Session, args):
    (variable,) = session.resolve([args.name])
    print(f"{variable.name} <- {args.value}: {session.write(variable, args.value)}")


def run_command(session: Session, args):
    code = COMMANDS[args.code] if args.code in COMMANDS else int(args.code, 0)
    print(f"command {args.code} ({code}, {args.argument}): {session.command(code, args.argument)}")


def run_stream(session: Session, args, printing: bool = False):
    variables = session.resolve(args.names)
    state_variable = session.by_name.get("fsm/state")
    stop_state = None

    if args.until_state:
        stop_state = STATES.index(args.until_state) if args.until_state in STATES else int(args.until_state)

        if state_variable and state_variable not in variables:
            variables.append(state_variable)

    rows = []
    last_sequence = [None]
    gaps = [0]
    finished = [False]
    left_stop_state = [False]

    def on_sample(sequence, timestamp, values):
        if last_sequence[0] is not None:
            gaps[0] += (sequence - last_sequence[0] - 1) & 0xFFFF

        last_sequence[0] = sequence
        rows.append([sequence, timestamp, *values])

        if printing:
            print(" ".join(f"{v.name}={format_value(v, x)}" for v, x in zip(variables, values)), flush=True)

        if stop_state is not None:
            state = values[variables.index(state_variable)]

            if state != stop_state:
                left_stop_state[0] = True
            elif left_stop_state[0]:
                finished[0] = True

    session.on_samples(0, variables, on_sample)
    achieved = session.define_group(0, variables, args.rate)
    session.log(f"streaming {len(variables)} variables at {achieved:.1f} Hz")

    if args.command:
        code = COMMANDS[args.command] if args.command in COMMANDS else int(args.command, 0)
        session.log(f"command {args.command} {args.argument}: {session.command(code, args.argument)}")

    deadline = time.monotonic() + args.seconds if args.seconds else None

    try:
        while not finished[0] and (deadline is None or time.monotonic() < deadline):
            session.poll()
    except KeyboardInterrupt:
        pass
    finally:
        session.disable_group(0)

    session.log(f"{len(rows)} samples, {gaps[0]} lost by sequence, {session.dropped_frames} corrupt frames")

    if args.out:
        with open(args.out, "w", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(["sequence", "time_us", *[v.name for v in variables]])
            writer.writerows(rows)

        session.log(f"wrote {args.out}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    target = parser.add_mutually_exclusive_group()
    target.add_argument("--ble", default=DEFAULT_ADDRESS, help="address of the robot's module")
    target.add_argument("--sim", action="store_true", help="the simulator's monitor bridge instead of the robot")
    parser.add_argument("--url", default="ws://localhost:8080", help="address of the simulator's monitor bridge")
    commands = parser.add_subparsers(dest="action", required=True)

    commands.add_parser("schema")
    read = commands.add_parser("read")
    read.add_argument("names", nargs="+")
    write = commands.add_parser("write")
    write.add_argument("name")
    write.add_argument("value")
    command = commands.add_parser("command")
    command.add_argument("code")
    command.add_argument(
        "argument", nargs="?", type=lambda text: PROCEDURES[text] if text in PROCEDURES else int(text, 0), default=0
    )

    for name in ("stream", "watch"):
        stream = commands.add_parser(name)
        stream.add_argument("names", nargs="+")
        stream.add_argument("--rate", type=float, default=20.0)
        stream.add_argument("--seconds", type=float, default=0.0)
        stream.add_argument("--out")
        stream.add_argument("--until-state", help="stop once fsm/state reaches this state")
        stream.add_argument("--command", help="command sent once the stream runs")
        stream.add_argument(
            "--argument", type=lambda text: PROCEDURES[text] if text in PROCEDURES else int(text, 0), default=0
        )

    args = parser.parse_args()
    session = open_session(args)

    try:
        if args.action == "schema":
            run_schema(session, args)
        elif args.action == "read":
            run_read(session, args)
        elif args.action == "write":
            run_write(session, args)
        elif args.action == "command":
            run_command(session, args)
        else:
            run_stream(session, args, printing=args.action == "watch")
    finally:
        session.transport.close()


if __name__ == "__main__":
    main()
