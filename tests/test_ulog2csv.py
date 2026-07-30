"""
convert/convert.py tests.

The promise of the converter is that the CSV holds everything the .ulg held --
so these tests are mostly about what must NOT happen: no sample overwritten by
another at the same timestamp, no value rounded on the way out, no column
silently merged. Stand-in ULog objects are used instead of a real log: pyulog is
not installed on every machine, the logs are ~120 MB and gitignored, and the
part worth testing is the table construction, not pyulog's parser.
"""

import csv
import importlib.util
import os

import numpy as np
import pytest

_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                     os.pardir, "convert", "convert.py")
_spec = importlib.util.spec_from_file_location("ulog2csv", _PATH)
u2c = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(u2c)


class FakeTopic:
    """One entry of ULog.data_list: a name, an instance id and column arrays."""

    def __init__(self, name, data, multi_id=0):
        self.name = name
        self.multi_id = multi_id
        self.data = data


class FakeULog:
    def __init__(self, data_list, start=1_000_000):
        self.data_list = data_list
        self.start_timestamp = start
        self.last_timestamp = start + 1_000_000


def outputs(ts, ch0, ch1, multi_id=0):
    return FakeTopic("actuator_outputs", {
        "timestamp": np.array(ts, dtype=np.uint64),
        "output[0]": np.array(ch0, dtype=np.float32),
        "output[1]": np.array(ch1, dtype=np.float32),
    }, multi_id)


def battery(ts, volts, amps):
    return FakeTopic("battery_status", {
        "timestamp": np.array(ts, dtype=np.uint64),
        "voltage_v": np.array(volts, dtype=np.float32),
        "current_a": np.array(amps, dtype=np.float32),
    })


def write(ulog, tmp_path, topics=None, name="log.csv"):
    """Convert and read back as a list of dict rows, plus the header."""
    path = str(tmp_path / name)
    stats = u2c.write_wide_csv(ulog, path, topics)
    with open(path, newline="", encoding="utf-8") as f:
        rows = list(csv.reader(f))
    return rows[0], rows[1:], stats


def test_header_is_topic_dot_field_in_topic_order(tmp_path):
    ulog = FakeULog([battery([1_010_000], [16.72], [0.41]),
                     outputs([1_000_000], [1000], [1450])])
    header, _rows, _stats = write(ulog, tmp_path)
    assert header == ["t_s", "timestamp",
                      "actuator_outputs.output[0]", "actuator_outputs.output[1]",
                      "battery_status.voltage_v", "battery_status.current_a"]


def test_second_instance_gets_a_suffix(tmp_path):
    ulog = FakeULog([outputs([1_000_000], [1000], [1000]),
                     outputs([1_000_000], [1200], [1300], multi_id=1)])
    header, _rows, _stats = write(ulog, tmp_path)
    assert header[2:] == ["actuator_outputs.output[0]", "actuator_outputs.output[1]",
                          "actuator_outputs_1.output[0]", "actuator_outputs_1.output[1]"]


def test_different_timestamps_get_their_own_rows(tmp_path):
    ulog = FakeULog([outputs([1_000_000, 1_020_100], [1000, 1000], [1000, 1450]),
                     battery([1_010_000], [16.72], [0.41])])
    _header, rows, _stats = write(ulog, tmp_path)
    assert [r[0] for r in rows] == ["0", "0.01", "0.0201"]
    # The battery row leaves the actuator columns blank, and vice versa
    assert rows[0] == ["0", "1000000", "1000", "1000", "", ""]
    assert rows[1] == ["0.01", "1010000", "", "", "16.72", "0.41"]


def test_same_timestamp_shares_one_row(tmp_path):
    ulog = FakeULog([outputs([1_000_000], [1000], [1450]),
                     battery([1_000_000], [16.72], [0.41])])
    _header, rows, _stats = write(ulog, tmp_path)
    assert rows == [["0", "1000000", "1000", "1450", "16.72", "0.41"]]


def test_repeated_topic_at_one_timestamp_splits_into_two_rows(tmp_path):
    """Two messages of the same topic at the same instant must not collide."""
    ulog = FakeULog([outputs([1_000_000, 1_000_000], [1000, 1900], [1450, 1950])])
    _header, rows, _stats = write(ulog, tmp_path)
    assert rows == [["0", "1000000", "1000", "1450"],
                    ["0", "1000000", "1900", "1950"]]


def test_every_sample_survives(tmp_path):
    """Filled cells per column == messages on that topic. Nothing dropped."""
    n_out, n_bat = 500, 37
    ulog = FakeULog([
        outputs(np.arange(n_out) * 2500 + 1_000_000,
                np.linspace(1000, 2000, n_out), np.linspace(2000, 1000, n_out)),
        battery(np.arange(n_bat) * 25_000 + 1_000_000,
                np.linspace(16.8, 15.1, n_bat), np.linspace(0.4, 91.2, n_bat)),
    ])
    header, rows, stats = write(ulog, tmp_path)
    filled = {name: sum(1 for r in rows if r[i] != "")
              for i, name in enumerate(header)}
    assert filled["actuator_outputs.output[0]"] == n_out
    assert filled["actuator_outputs.output[1]"] == n_out
    assert filled["battery_status.voltage_v"] == n_bat
    assert filled["battery_status.current_a"] == n_bat
    assert stats["n_msgs"] == n_out + n_bat
    # Timestamps overlap every tenth sample, so rows < messages
    assert stats["n_rows"] == n_out


def test_rows_come_out_in_time_order(tmp_path):
    ulog = FakeULog([outputs([1_060_000, 1_000_000, 1_030_000],
                             [1200, 1000, 1100], [1200, 1000, 1100])])
    _header, rows, _stats = write(ulog, tmp_path)
    assert [int(r[1]) for r in rows] == [1_000_000, 1_030_000, 1_060_000]


def test_floats_are_not_rounded_or_widened(tmp_path):
    """float32 16.72 is 16.719999313354492 as a double -- neither form is wanted."""
    ulog = FakeULog([battery([1_000_000, 1_010_000],
                             [16.72, 0.000123456], [1e-20, 12345.6789])])
    _header, rows, _stats = write(ulog, tmp_path)
    assert rows[0][4] == "16.72"
    assert rows[1][4] == "0.000123456"
    assert rows[0][5] == "1.e-20"          # tiny values stay compact
    assert rows[1][5] == "12345.679"       # float32 holds no more than this
    # and every one of them reads back as the same float32
    for text, want in (("16.72", 16.72), ("0.000123456", 0.000123456),
                       ("1.e-20", 1e-20), ("12345.679", 12345.6789)):
        assert np.float32(text) == np.float32(want)


def test_nan_is_written_not_blanked(tmp_path):
    """A logged NaN is data; a blank cell means no sample at all."""
    ulog = FakeULog([battery([1_000_000], [np.nan], [0.41])])
    _header, rows, _stats = write(ulog, tmp_path)
    assert rows[0][4] == "nan"


def test_topics_filter_drops_columns_and_rows(tmp_path):
    ulog = FakeULog([outputs([1_000_000, 1_020_000], [1000, 1450], [1000, 1450]),
                     battery([1_010_000], [16.72], [0.41])])
    header, rows, stats = write(ulog, tmp_path, topics=["battery_status"])
    assert header == ["t_s", "timestamp",
                      "battery_status.voltage_v", "battery_status.current_a"]
    assert rows == [["0.01", "1010000", "16.72", "0.41"]]
    assert stats["n_topics"] == 1


def test_topic_without_timestamp_is_skipped_not_fatal(tmp_path):
    ulog = FakeULog([FakeTopic("weird", {"value": np.array([1.0])}),
                     battery([1_000_000], [16.72], [0.41])])
    header, rows, stats = write(ulog, tmp_path)
    assert "weird.value" not in header
    assert stats["n_topics"] == 1
    assert rows == [["0", "1000000", "16.72", "0.41"]]


def test_empty_log_writes_just_a_header(tmp_path):
    header, rows, stats = write(FakeULog([]), tmp_path)
    assert header == ["t_s", "timestamp"]
    assert rows == []
    assert stats["n_rows"] == 0


def test_find_ulogs_walks_subfolders_and_ignores_others(tmp_path):
    (tmp_path / "2026-07-24").mkdir()
    for rel in ("a.ulg", "2026-07-24/b.ULG", "notes.txt", "2026-07-24/c.csv"):
        (tmp_path / rel).write_bytes(b"")
    assert u2c.find_ulogs(str(tmp_path)) == [os.path.join("2026-07-24", "b.ULG"),
                                             "a.ulg"]


def test_cli_converts_a_folder(tmp_path, monkeypatch):
    """End to end through main(), with the pyulog parse stubbed out."""
    indir, outdir = tmp_path / "ulog", tmp_path / "csv"
    indir.mkdir()
    (indir / "log_1.ulg").write_bytes(b"")

    ulog = FakeULog([outputs([1_000_000], [1000], [1450]),
                     battery([1_000_000], [16.72], [0.41])])
    monkeypatch.setattr(u2c, "load_ulog", lambda path, topics=None: ulog)

    argv = ["--in", str(indir), "--out", str(outdir)]
    assert u2c.main(argv) == 0
    out = outdir / "log_1.csv"
    assert out.exists()
    assert out.read_text().splitlines()[1] == "0,1000000,1000,1450,16.72,0.41"

    # A second run leaves the up-to-date CSV alone
    stamp = out.stat().st_mtime_ns
    assert u2c.main(argv) == 0
    assert out.stat().st_mtime_ns == stamp
    assert u2c.main(argv + ["--force"]) == 0


def test_cli_reports_a_bad_log_without_stopping(tmp_path, monkeypatch):
    indir, outdir = tmp_path / "ulog", tmp_path / "csv"
    indir.mkdir()
    (indir / "bad.ulg").write_bytes(b"")
    (indir / "good.ulg").write_bytes(b"")

    def load(path, topics=None):
        if "bad" in path:
            raise ValueError("truncated header")
        return FakeULog([battery([1_000_000], [16.72], [0.41])])

    monkeypatch.setattr(u2c, "load_ulog", load)
    assert u2c.main(["--in", str(indir), "--out", str(outdir)]) == 1
    assert (outdir / "good.csv").exists()
    assert not (outdir / "bad.csv").exists()
