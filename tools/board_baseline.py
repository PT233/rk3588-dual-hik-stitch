#!/usr/bin/env python3
"""Run a bounded camera self-check or an independent RTSP decode check."""

import argparse
import hashlib
import json
import math
import os
import platform
import re
import shutil
import subprocess
import sys
import time
from pathlib import Path
from urllib.parse import urlsplit


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def executable(name):
    path = shutil.which(name)
    if not path:
        raise ValueError(f"executable not found: {name}")
    return Path(path).resolve()


def read_optional(path):
    try:
        return path.read_text(encoding="utf-8", errors="replace").strip("\0\n ")
    except OSError:
        return None


def sample_host(pid, started, sensors):
    sample = {"elapsed_ms": round((time.monotonic() - started) * 1000)}
    stat = read_optional(Path(f"/proc/{pid}/stat"))
    if stat:
        fields = stat[stat.rfind(")") + 2:].split()
        if len(fields) > 21:
            sample["process_cpu_seconds"] = (int(fields[11]) + int(fields[12])) / os.sysconf("SC_CLK_TCK")
            sample["process_rss_bytes"] = int(fields[21]) * os.sysconf("SC_PAGE_SIZE")
    sample["thermal_millicelsius"] = {}
    for name, sensor in sensors.items():
        value = read_optional(sensor)
        if value and re.fullmatch(r"-?[0-9]+", value):
            sample["thermal_millicelsius"][name] = int(value)
    return sample


def run_bounded(command, out, timeout, redact_url=False):
    sensors = {
        f"{zone.name}:{read_optional(zone / 'type') or 'unknown'}": zone / "temp"
        for zone in sorted(Path("/sys/class/thermal").glob("thermal_zone*"))
    }
    started = time.monotonic()
    timed_out = False
    stderr = out / "stderr.log"
    # RTSP diagnostics can contain the caller's URL. Suppress them in the evidence bundle.
    with (out / "stdout.log").open("wb") as output, stderr.open("wb") as errors, \
            (out / "resources.jsonl").open("w", encoding="utf-8") as resources:
        process = subprocess.Popen(command, stdout=output, stderr=subprocess.DEVNULL if redact_url else errors)
        try:
            while process.poll() is None:
                resources.write(json.dumps(sample_host(process.pid, started, sensors)) + "\n")
                remaining = timeout - (time.monotonic() - started)
                if remaining <= 0:
                    timed_out = True
                    break
                try:
                    process.wait(timeout=min(remaining, 1))
                except subprocess.TimeoutExpired:
                    pass
        finally:
            if process.poll() is None:
                process.kill()
            process.wait()
    return {"exit_code": process.returncode, "timed_out": timed_out,
            "elapsed_ms": round((time.monotonic() - started) * 1000)}


def distribution(values):
    values = sorted(values)
    if not values:
        return None
    result = {"count": len(values), "max": values[-1]}
    for percent in (50, 95, 99):
        result[f"p{percent}"] = values[math.ceil(len(values) * percent / 100) - 1]
    return result


def summarize_trace(trace, pairs, warmup):
    with trace.open(encoding="utf-8") as source:
        rows = [json.loads(line) for line in source]
    if len(rows) != pairs or any(row["pair_index"] != i for i, row in enumerate(rows, 1)):
        raise ValueError("frame trace is incomplete or out of order")
    measured = rows[warmup:]
    intervals = [second["pair_monotonic_us"] - first["pair_monotonic_us"]
                 for first, second in zip(measured, measured[1:])]
    span = sum(intervals)
    return {
        "pairs": pairs, "warmup_excluded": warmup, "clock": rows[0]["clock"],
        "paired_fps": len(intervals) * 1_000_000 / span if span > 0 else None,
        "first_pair_sources": {side: rows[0][side] for side in ("left", "right")},
        "pair_interval_us": distribution(intervals),
        "absolute_pair_skew_us": distribution(abs(row["pair_skew_us"]) for row in measured),
        "stitch_us": distribution(row["stitch_us"] for row in measured),
        "oldest_receive_age_us": distribution(max(row[side]["receive_age_us"] for side in ("left", "right"))
                                               for row in measured),
    }


def capture(args, extra, out, report):
    if args.warmup < 0 or args.warmup >= args.frames - 1:
        raise ValueError("warmup must leave at least two measured pairs")
    reserved = {"--output", "--frames", "--check-pairs", "--trace-dir", "--run-timeout-ms",
                "--calibration", "--snapshot", "--list", "--help"}
    if reserved.intersection(extra):
        raise ValueError("camera arguments include an option managed by the baseline tool")
    app = executable(args.app)
    calibration = out / "calibration.input"
    shutil.copyfile(args.calibration, calibration)
    report.update({"source": "synthetic" if "--synthetic" in extra else "camera",
                   "capture_executable_sha256": sha256(app), "calibration_sha256": sha256(calibration),
                   "camera_args": extra})
    trace_dir = out / "trace"
    command = [str(app), "--calibration", str(calibration), "--check-pairs", str(args.frames),
               "--run-timeout-ms", str(int(args.timeout * 1000)), "--trace-dir", str(trace_dir), *extra]
    report["process"] = run_bounded(command, out, args.timeout)
    if report["process"]["exit_code"] or report["process"]["timed_out"]:
        raise ValueError("camera self-check failed or timed out; inspect stderr.log and partial trace")
    result = json.loads((out / "stdout.log").read_text(encoding="utf-8"))
    if result.get("status") != "passed" or result.get("pairs") != args.frames:
        raise ValueError("camera self-check did not confirm the requested frame pairs")
    report["self_check"] = result
    report["measurements"] = summarize_trace(trace_dir / "frames.jsonl", args.frames, args.warmup)
    logs = (out / "stderr.log").read_text(encoding="utf-8", errors="replace").splitlines()
    summary = next((line for line in reversed(logs) if line.startswith("paired=")), "")
    report["capture_counters"] = {key: int(value) for key, value in re.findall(r"(\w+)=([0-9]+)\b", summary)}


def rtsp(args, out, report):
    url = urlsplit(args.url)
    if url.scheme != "rtsp" or not url.hostname:
        raise ValueError("a valid rtsp:// URL is required")
    ffmpeg = executable(args.ffmpeg)
    report["ffmpeg_sha256"] = sha256(ffmpeg)
    command = [str(ffmpeg), "-nostdin", "-hide_banner", "-loglevel", "error", "-xerror",
               "-rtsp_transport", "tcp", "-i", args.url, "-map", "0:v:0", "-an",
               "-frames:v", str(args.frames), "-progress", "pipe:1", "-f", "null", "-"]
    report["process"] = run_bounded(command, out, args.timeout, redact_url=True)
    progress = (out / "stdout.log").read_text(encoding="utf-8", errors="replace")
    decoded = max((int(n) for n in re.findall(r"(?m)^frame=\s*(\d+)$", progress)), default=0)
    report["decoded_frames"] = decoded
    if report["process"]["exit_code"] or report["process"]["timed_out"] or decoded != args.frames:
        raise ValueError("RTSP decoder did not complete the requested frame count within the deadline")


def main(argv):
    extra = []
    if "--" in argv:
        index = argv.index("--")
        argv, extra = argv[:index], argv[index + 1:]
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_subparsers(dest="mode", required=True)
    capture_parser = modes.add_parser("capture", help="pass camera options after --")
    capture_parser.add_argument("--app", default="dual-hik-stitch")
    capture_parser.add_argument("--calibration", required=True, type=Path)
    capture_parser.add_argument("--warmup", type=int, default=1)
    rtsp_parser = modes.add_parser("rtsp", help="verify the running service using a video decoder")
    rtsp_parser.add_argument("--ffmpeg", default="ffmpeg")
    rtsp_parser.add_argument("--url", required=True)
    for mode in (capture_parser, rtsp_parser):
        mode.add_argument("--out", required=True, type=Path, help="a new directory")
        mode.add_argument("--frames", type=int, default=90)
        mode.add_argument("--timeout", type=float, default=20)
    args = parser.parse_args(argv)
    if not 2 <= args.frames <= 100000 or not 0.2 <= args.timeout <= 3600:
        parser.error("frames must be 2..100000 and timeout 0.2..3600 seconds")
    if args.mode == "rtsp" and extra:
        parser.error("extra camera arguments are only accepted for capture")
    out = args.out.resolve()
    try:
        out.mkdir()
    except OSError as error:
        print(f"cannot create a new result directory: {error}", file=sys.stderr)
        return 1
    report = {"schema_version": 1, "mode": args.mode, "status": "failed", "requested_frames": args.frames,
              "platform": {"system": platform.system(), "release": platform.release(),
                           "machine": platform.machine(), "board": read_optional(Path("/proc/device-tree/model"))}}
    try:
        if args.mode == "capture":
            capture(args, extra, out, report)
        else:
            rtsp(args, out, report)
        report["status"] = "passed"
    except (OSError, ValueError, KeyError, IndexError, TypeError) as error:
        report["error"] = str(error) if args.mode == "capture" else "RTSP check failed; inspect the process result"
        print(report["error"], file=sys.stderr)
    (out / "report.json").write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return 0 if report["status"] == "passed" else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
