#!/usr/bin/env python3
"""
WPILog Parser — reusable CLI for parsing AdvantageKit / WPILib .wpilog files.

Usage:
    python wpilog.py entries  <file>  [--filter KEYWORD]
    python wpilog.py extract  <file>  --signals SIG [SIG ...] [--start S] [--end E]
    python wpilog.py timeline <file>  --signals SIG [SIG ...] [--start S] [--end E]
    python wpilog.py summary  <file>

Times are in seconds (float). Signals can be exact names or substring filters.
"""

import argparse
import array
import mmap
import struct
import sys
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, SupportsBytes, Tuple

# ── Binary format constants ──────────────────────────────────────────────────

_FLOAT = struct.Struct("<f")
_DOUBLE = struct.Struct("<d")

CTRL_START = 0
CTRL_FINISH = 1
CTRL_SET_METADATA = 2


# ── Low-level record types ──────────────────────────────────────────────────

@dataclass
class StartRecord:
    entry: int
    name: str
    type: str
    metadata: str


@dataclass
class RawRecord:
    entry: int
    timestamp: int  # microseconds
    data: bytes

    # ── control helpers ──────────────────────────────────────────────────
    @property
    def is_control(self) -> bool:
        return self.entry == 0

    def _ctrl_type(self) -> int:
        return self.data[0]

    @property
    def is_start(self) -> bool:
        return self.is_control and len(self.data) >= 17 and self._ctrl_type() == CTRL_START

    @property
    def is_finish(self) -> bool:
        return self.is_control and len(self.data) == 5 and self._ctrl_type() == CTRL_FINISH

    @property
    def is_set_metadata(self) -> bool:
        return self.is_control and len(self.data) >= 9 and self._ctrl_type() == CTRL_SET_METADATA

    def get_start_data(self) -> StartRecord:
        entry = int.from_bytes(self.data[1:5], "little", signed=False)
        name, pos = self._read_str(5)
        typ, pos = self._read_str(pos)
        meta, _ = self._read_str(pos)
        return StartRecord(entry, name, typ, meta)

    def get_finish_entry(self) -> int:
        return int.from_bytes(self.data[1:5], "little", signed=False)

    # ── value decoders ───────────────────────────────────────────────────
    def as_boolean(self) -> bool:
        return self.data[0] != 0

    def as_int(self) -> int:
        return int.from_bytes(self.data, "little", signed=True)

    def as_float(self) -> float:
        return _FLOAT.unpack(self.data)[0]

    def as_double(self) -> float:
        return _DOUBLE.unpack(self.data)[0]

    def as_string(self) -> str:
        return self.data.decode("utf-8", errors="replace")

    def as_string_array(self) -> List[str]:
        count = int.from_bytes(self.data[:4], "little", signed=False)
        pos = 4
        out: List[str] = []
        for _ in range(count):
            s, pos = self._read_str(pos)
            out.append(s)
        return out

    def as_double_array(self) -> list:
        arr = array.array("d")
        arr.frombytes(self.data)
        return arr.tolist()

    def as_float_array(self) -> list:
        arr = array.array("f")
        arr.frombytes(self.data)
        return arr.tolist()

    def as_int_array(self) -> list:
        arr = array.array("q")
        arr.frombytes(self.data)
        return arr.tolist()

    def as_boolean_array(self) -> List[bool]:
        return [b != 0 for b in self.data]

    def _read_str(self, pos: int) -> Tuple[str, int]:
        size = int.from_bytes(self.data[pos:pos + 4], "little", signed=False)
        end = pos + 4 + size
        return self.data[pos + 4:end].decode("utf-8", errors="replace"), end


# ── DataLog reader (iterator over raw records) ──────────────────────────────

class DataLogReader:
    """Memory-mapped reader for .wpilog files."""

    def __init__(self, buf: SupportsBytes):
        self.buf = buf

    def is_valid(self) -> bool:
        return len(self.buf) >= 12 and self.buf[:6] == b"WPILOG" and self.version >= 0x0100

    @property
    def version(self) -> int:
        return int.from_bytes(self.buf[6:8], "little", signed=False)

    @property
    def extra_header(self) -> str:
        size = int.from_bytes(self.buf[8:12], "little", signed=False)
        return self.buf[12:12 + size].decode("utf-8", errors="replace")

    def __iter__(self):
        extra = int.from_bytes(self.buf[8:12], "little", signed=False)
        return _RecordIterator(self.buf, 12 + extra)


class _RecordIterator:
    def __init__(self, buf, pos):
        self.buf = buf
        self.pos = pos

    def __iter__(self):
        return self

    @staticmethod
    def _read_var(buf, pos, length):
        val = 0
        for i in range(length):
            val |= buf[pos + i] << (i * 8)
        return val

    def __next__(self) -> RawRecord:
        if len(self.buf) < self.pos + 4:
            raise StopIteration
        hdr = self.buf[self.pos]
        entry_len = (hdr & 0x3) + 1
        size_len = ((hdr >> 2) & 0x3) + 1
        ts_len = ((hdr >> 4) & 0x7) + 1
        header_len = 1 + entry_len + size_len + ts_len
        if len(self.buf) < self.pos + header_len:
            raise StopIteration
        p = self.pos + 1
        entry = self._read_var(self.buf, p, entry_len);           p += entry_len
        size = self._read_var(self.buf, p, size_len);              p += size_len
        timestamp = self._read_var(self.buf, p, ts_len)
        data_start = self.pos + header_len
        if len(self.buf) < data_start + size:
            raise StopIteration
        rec = RawRecord(entry, timestamp, bytes(self.buf[data_start:data_start + size]))
        self.pos = data_start + size
        return rec


# ── High-level helpers ───────────────────────────────────────────────────────

def decode_value(rec: RawRecord, type_str: str) -> Any:
    """Decode a data record's payload into a Python value."""
    try:
        if type_str == "double":
            return rec.as_double()
        elif type_str == "int64":
            return rec.as_int()
        elif type_str == "float":
            return rec.as_float()
        elif type_str == "boolean":
            return rec.as_boolean()
        elif type_str in ("string", "json"):
            return rec.as_string()
        elif type_str == "string[]":
            return rec.as_string_array()
        elif type_str == "double[]":
            return rec.as_double_array()
        elif type_str == "float[]":
            return rec.as_float_array()
        elif type_str == "int64[]":
            return rec.as_int_array()
        elif type_str == "boolean[]":
            return rec.as_boolean_array()
        elif type_str.startswith("struct:") or type_str.startswith("structschema:"):
            return f"<struct {len(rec.data)}B>"
        elif type_str.startswith("proto:"):
            return f"<protobuf {len(rec.data)}B>"
        else:
            return f"<raw {len(rec.data)}B>"
    except Exception as e:
        return f"<decode error: {e}>"


def open_log(path: str) -> DataLogReader:
    f = open(path, "rb")
    mm = mmap.mmap(f.fileno(), 0, access=mmap.ACCESS_READ)
    reader = DataLogReader(mm)
    if not reader.is_valid():
        print(f"Error: {path} is not a valid .wpilog file", file=sys.stderr)
        sys.exit(1)
    return reader


def build_entry_map(reader: DataLogReader) -> Dict[int, StartRecord]:
    """First pass: collect all Start records → {entry_id: StartRecord}."""
    entries: Dict[int, StartRecord] = {}
    for rec in reader:
        if rec.is_start:
            sd = rec.get_start_data()
            entries[sd.entry] = sd
        elif rec.is_finish:
            eid = rec.get_finish_entry()
            entries.pop(eid, None)
    return entries


def match_signals(entries: Dict[int, StartRecord], patterns: List[str]) -> Dict[int, StartRecord]:
    """Return entries whose name contains any of the given substrings (case-insensitive)."""
    matched: Dict[int, StartRecord] = {}
    for eid, sr in entries.items():
        lower = sr.name.lower()
        for p in patterns:
            if p.lower() in lower:
                matched[eid] = sr
                break
    return matched


# ── CLI commands ─────────────────────────────────────────────────────────────

def cmd_entries(args):
    reader = open_log(args.file)
    entries = build_entry_map(reader)
    # group by first path component for readability
    names = sorted(entries.values(), key=lambda s: s.name.lower())
    if args.filter:
        kw = args.filter.lower()
        names = [s for s in names if kw in s.name.lower()]

    max_name = max((len(s.name) for s in names), default=10)
    max_type = max((len(s.type) for s in names), default=6)
    fmt = f"  {{:<{max_name}}}  {{:<{max_type}}}  id={{}}"
    print(f"{'Signal':<{max_name}}  {'Type':<{max_type}}  ID")
    print("-" * (max_name + max_type + 10))
    for s in names:
        print(fmt.format(s.name, s.type, s.entry))
    print(f"\n{len(names)} entries", end="")
    if args.filter:
        print(f" matching '{args.filter}'", end="")
    print()


def cmd_extract(args):
    reader = open_log(args.file)
    entries = build_entry_map(reader)
    matched = match_signals(entries, args.signals)
    if not matched:
        print("No matching signals found. Use 'entries' command to list available signals.", file=sys.stderr)
        sys.exit(1)

    t_min = int(args.start * 1e6) if args.start is not None else 0
    t_max = int(args.end * 1e6) if args.end is not None else float("inf")
    target_ids = set(matched.keys())

    print(f"Extracting {len(matched)} signal(s) in [{args.start or 0:.3f}s .. {args.end or '∞'}s]:\n")
    for eid, sr in sorted(matched.items(), key=lambda x: x[1].name):
        print(f"  {sr.name}  ({sr.type})")
    print()

    # second pass — collect data
    rows: List[Tuple[float, str, str, Any]] = []  # (time_s, name, type, value)
    for rec in reader:
        if rec.is_control:
            continue
        if rec.entry not in target_ids:
            continue
        if rec.timestamp < t_min or rec.timestamp > t_max:
            continue
        sr = matched[rec.entry]
        val = decode_value(rec, sr.type)
        rows.append((rec.timestamp / 1e6, sr.name, sr.type, val))

    rows.sort(key=lambda r: r[0])
    if not rows:
        print("(no data in specified time range)")
        return

    max_name = max(len(r[1]) for r in rows)
    for t, name, typ, val in rows:
        print(f"  {t:12.6f}  {name:<{max_name}}  {val}")
    print(f"\n{len(rows)} samples")


def cmd_timeline(args):
    """Interleaved multi-signal view — the main diagnostic tool.

    Forward-fills all signal values so every row shows the complete state.
    Pre-scans before the time window to capture initial values.
    Marks changed values with * in the leftmost column position.
    """
    reader = open_log(args.file)
    entries = build_entry_map(reader)
    matched = match_signals(entries, args.signals)
    if not matched:
        print("No matching signals found. Use 'entries' command to list available signals.", file=sys.stderr)
        sys.exit(1)

    t_min = int(args.start * 1e6) if args.start is not None else 0
    t_max = int(args.end * 1e6) if args.end is not None else float("inf")
    target_ids = set(matched.keys())

    # Assign short labels for column headers
    sig_names = sorted(set(sr.name for sr in matched.values()))
    labels = {name: name.rsplit("/", 1)[-1] for name in sig_names}
    # Deduplicate labels by using two last path components when collisions occur
    label_counts: Dict[str, int] = {}
    for name in sig_names:
        label_counts[labels[name]] = label_counts.get(labels[name], 0) + 1
    for name in sig_names:
        if label_counts[labels[name]] > 1:
            parts = name.rsplit("/", 2)
            labels[name] = "/".join(parts[-2:]) if len(parts) >= 2 else name

    # Format value compactly
    def fmt_val(v):
        if isinstance(v, bool):
            return "T" if v else "F"
        if isinstance(v, float):
            return f"{v:.4f}"
        if isinstance(v, int):
            return str(v)
        if isinstance(v, list):
            if len(v) <= 4:
                return "[" + ",".join(fmt_val(x) for x in v) + "]"
            return f"[{len(v)} items]"
        s = str(v)
        return s if len(s) <= 50 else s[:47] + "..."

    # Single pass: pre-scan for initial state (before t_min) and collect in-window events
    state: Dict[str, str] = {name: "—" for name in sig_names}
    events: List[Tuple[float, str, Any]] = []
    id_to_name = {eid: sr.name for eid, sr in matched.items()}

    for rec in reader:
        if rec.is_control:
            continue
        if rec.entry not in target_ids:
            continue
        name = id_to_name[rec.entry]
        if rec.timestamp < t_min:
            # Pre-scan: track last value before the window
            sr = matched[rec.entry]
            state[name] = fmt_val(decode_value(rec, sr.type))
        elif rec.timestamp <= t_max:
            sr = matched[rec.entry]
            val = decode_value(rec, sr.type)
            events.append((rec.timestamp / 1e6, name, val))
        # Don't break early — records may not be strictly ordered

    events.sort(key=lambda e: e[0])
    if not events:
        print("(no data in specified time range)")
        if any(v != "—" for v in state.values()):
            print("\nLast known state before window:")
            for name in sig_names:
                print(f"  {labels[name]}: {state[name]}")
        return

    # Compute column widths from headers AND data
    col_widths = {name: len(labels[name]) for name in sig_names}
    temp_state = dict(state)
    for _, name, val in events:
        fv = fmt_val(val)
        col_widths[name] = max(col_widths[name], len(fv) + 1)  # +1 for * marker
        temp_state[name] = fv
    # Also account for initial state values
    for name in sig_names:
        col_widths[name] = max(col_widths[name], len(state[name]) + 1)

    # Print header
    time_hdr = "Time(s)"
    parts = [f"{time_hdr:>12}"]
    for name in sig_names:
        w = col_widths[name]
        parts.append(f"{labels[name]:>{w}}")
    header = "  ".join(parts)
    print(header)
    print("-" * len(header))

    for t, name, val in events:
        formatted = fmt_val(val)
        old = state[name]
        changed = old != formatted
        state[name] = formatted

        row_parts = [f"{t:12.6f}"]
        for sn in sig_names:
            w = col_widths[sn]
            if sn == name:
                marker = "*" if changed else " "
                row_parts.append(f"{marker}{state[sn]:>{w - 1}}")
            else:
                # Forward-fill: show current state dimly (no marker)
                row_parts.append(f" {state[sn]:>{w - 1}}")
        print("  ".join(row_parts))

    print(f"\n{len(events)} events across {len(sig_names)} signals")
    print(f"Time range: {events[0][0]:.6f}s .. {events[-1][0]:.6f}s")


def cmd_summary(args):
    reader = open_log(args.file)
    print(f"File:     {args.file}")
    print(f"Version:  {reader.version >> 8}.{reader.version & 0xFF}")
    print(f"Producer: {reader.extra_header}")
    print()

    entries = build_entry_map(reader)

    # Second pass to get time range and record counts
    first_ts = None
    last_ts = None
    data_count = 0
    ctrl_count = 0
    for rec in reader:
        if rec.is_control:
            ctrl_count += 1
        else:
            data_count += 1
            if first_ts is None or rec.timestamp < first_ts:
                first_ts = rec.timestamp
            if last_ts is None or rec.timestamp > last_ts:
                last_ts = rec.timestamp

    duration = (last_ts - first_ts) / 1e6 if first_ts and last_ts else 0
    print(f"Entries:       {len(entries)}")
    print(f"Data records:  {data_count:,}")
    print(f"Ctrl records:  {ctrl_count}")
    print(f"Time span:     {duration:.3f}s ({duration / 60:.1f} min)")
    if first_ts:
        print(f"First ts:      {first_ts / 1e6:.6f}s")
        print(f"Last ts:       {last_ts / 1e6:.6f}s")

    # Type breakdown
    types: Dict[str, int] = {}
    for sr in entries.values():
        types[sr.type] = types.get(sr.type, 0) + 1
    print("\nEntry types:")
    for t, c in sorted(types.items(), key=lambda x: -x[1]):
        print(f"  {t:20s}  {c}")


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    p = argparse.ArgumentParser(
        prog="wpilog",
        description="Parse and query WPILib/AdvantageKit .wpilog files.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python wpilog.py entries  log.wpilog --filter shoot
  python wpilog.py extract  log.wpilog --signals Velocity Reference --start 30 --end 38
  python wpilog.py timeline log.wpilog --signals shooter command DS/Enabled --start 33 --end 35
  python wpilog.py summary  log.wpilog
""",
    )
    sub = p.add_subparsers(dest="command", required=True)

    # entries
    p_ent = sub.add_parser("entries", help="List all entry names in the log")
    p_ent.add_argument("file", help="Path to .wpilog file")
    p_ent.add_argument("--filter", "-f", help="Filter entries by keyword (case-insensitive)")

    # extract
    p_ext = sub.add_parser("extract", help="Extract values for specific signals")
    p_ext.add_argument("file", help="Path to .wpilog file")
    p_ext.add_argument("--signals", "-s", nargs="+", required=True, help="Signal name substrings")
    p_ext.add_argument("--start", type=float, default=None, help="Start time in seconds")
    p_ext.add_argument("--end", type=float, default=None, help="End time in seconds")

    # timeline
    p_tl = sub.add_parser("timeline", help="Cross-signal correlation timeline")
    p_tl.add_argument("file", help="Path to .wpilog file")
    p_tl.add_argument("--signals", "-s", nargs="+", required=True, help="Signal name substrings")
    p_tl.add_argument("--start", type=float, default=None, help="Start time in seconds")
    p_tl.add_argument("--end", type=float, default=None, help="End time in seconds")

    # summary
    p_sum = sub.add_parser("summary", help="Overview of the log file")
    p_sum.add_argument("file", help="Path to .wpilog file")

    args = p.parse_args()
    {
        "entries": cmd_entries,
        "extract": cmd_extract,
        "timeline": cmd_timeline,
        "summary": cmd_summary,
    }[args.command](args)


if __name__ == "__main__":
    main()
