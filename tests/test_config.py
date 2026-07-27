"""
Plan loading and validation tests.

Validation is the last line of defence before a battery is spent, so each rule
is pinned to the failure it prevents rather than merely to its own existence.
"""

import json

import pytest

from tvcbench import config
from tvcbench.config import Plan, PlanError


def plan(**overrides):
    base = {"schema_version": 1}
    base.update(overrides)
    return Plan(base)


def problems(**overrides):
    with pytest.raises(PlanError) as exc:
        plan(**overrides)
    return exc.value.problems


class TestDefaultsAndMerging:
    def test_an_almost_empty_plan_is_valid(self):
        p = plan()
        assert p["repeats"] == 1
        assert p["dwell"]["mode"] == "fixed"

    def test_nested_overrides_merge_rather_than_replace(self):
        """Setting one hardware field must not wipe the rest of the section."""
        p = plan(hardware={"fc": {"baud": 57600}})
        assert p["hardware"]["fc"]["baud"] == 57600
        assert p["hardware"]["fc"]["device"] == "/dev/ttyAMA0"
        assert p["hardware"]["loadcell"]["baud"] == 115200

    def test_raw_is_kept_verbatim_for_the_manifest(self):
        raw = {"schema_version": 1, "repeats": 3}
        p = Plan(raw)
        assert p.raw == raw
        assert p.as_dict()["repeats"] == 3
        assert "hardware" not in p.raw          # untouched by the defaults merge

    def test_as_dict_is_a_copy(self):
        p = plan()
        p.as_dict()["repeats"] = 99
        assert p["repeats"] == 1

    def test_unknown_keys_are_carried_through(self):
        # Forward compatibility: an unrecognised key is preserved into the
        # manifest rather than silently dropped.
        assert plan(experiment_id="x7")["experiment_id"] == "x7"


class TestGrid:
    def test_expands_inclusive_bounds(self):
        p = plan(grid={"a": [1300, 1500, 100], "b": [1000, 1200, 100]})
        assert p.a_values == [1300, 1400, 1500]
        assert p.b_values == [1000, 1100, 1200]
        assert p.n_grid_points == 9

    def test_single_point_grid_is_allowed(self):
        p = plan(grid={"a": [1500, 1500, 100], "b": [1500, 1500, 100]})
        assert p.n_grid_points == 1

    def test_step_larger_than_span_still_yields_the_start(self):
        assert plan(grid={"a": [1300, 1400, 500], "b": [1000, 1000, 100]}).a_values \
            == [1300]

    @pytest.mark.parametrize("spec", [[900, 2000, 100], [1000, 2500, 100]])
    def test_out_of_range_bounds_rejected(self, spec):
        assert any("1000-2000us" in p for p in problems(grid={"a": spec,
                                                             "b": [1000, 2000, 100]}))

    def test_reversed_bounds_rejected(self):
        assert any("below start" in p
                   for p in problems(grid={"a": [1600, 1300, 100],
                                           "b": [1000, 2000, 100]}))

    def test_malformed_axis_rejected(self):
        assert any("[start, end, step]" in p
                   for p in problems(grid={"a": [1300, 1600], "b": [1000, 2000, 100]}))


class TestDwellValidation:
    def test_unknown_mode(self):
        assert any("dwell.mode" in p for p in problems(dwell={"mode": "vibes"}))

    def test_max_below_min(self):
        assert any("below" in p
                   for p in problems(dwell={"mode": "fixed", "min_s": 5, "max_s": 2}))

    def test_sem_target_needs_a_target(self):
        assert any("target_thrust_sem_n" in p
                   for p in problems(dwell={"mode": "sem_target",
                                            "target_thrust_sem_n": 0}))

    def test_settle_needs_its_thresholds(self):
        found = problems(dwell={"mode": "settle", "settle_rate_n_per_s": 0,
                                "settle_window_s": 0})
        assert any("settle_rate_n_per_s" in p for p in found)
        assert any("settle_window_s" in p for p in found)

    def test_unbounded_adaptive_dwell_is_rejected(self):
        """max_s is the only thing stopping a step that never settles."""
        assert any("never settles" in p
                   for p in problems(dwell={"mode": "sem_target", "max_s": 600}))

    def test_a_long_fixed_dwell_is_fine(self):
        # Nothing adaptive about it, so max_s is not load-bearing here.
        plan(dwell={"mode": "fixed", "fixed_s": 120, "max_s": 600})


class TestSafetyValidation:
    def test_resend_faster_than_expiry_is_required(self):
        """Otherwise commands lapse between sends and the step is holed."""
        assert any("stutter" in p
                   for p in problems(hardware={"fc": {"timeout_s": 0.1,
                                                      "resend_hz": 5.0}}))

    def test_defaults_satisfy_the_resend_invariant(self):
        plan()          # 1.0 s expiry against a 0.2 s resend interval

    def test_dropout_detection_cannot_be_disabled(self):
        assert any("dead force sensor" in p
                   for p in problems(limits={"loadcell_dropout_s": 0}))

    def test_heartbeat_timeout_must_be_positive(self):
        assert any("heartbeat_timeout_s" in p
                   for p in problems(limits={"heartbeat_timeout_s": 0}))

    def test_limits_default_to_disabled_but_dropout_stays_on(self):
        p = plan()
        assert p["limits"]["max_thrust_n"] == 0.0        # opt-in
        assert p["limits"]["loadcell_dropout_s"] > 0     # always on


class TestMiscValidation:
    def test_schema_version_mismatch_is_reported(self):
        with pytest.raises(PlanError, match="schema_version"):
            Plan({"schema_version": 99})

    def test_repeats_must_be_positive(self):
        assert any("repeats" in p for p in problems(repeats=0))

    def test_reference_bounds_checked_only_when_enabled(self):
        plan(reference={"a": 5, "b": 5, "every_n_steps": 0})       # disabled: ignored
        assert any("reference.a" in p
                   for p in problems(reference={"a": 5, "b": 1500,
                                                "every_n_steps": 4}))

    def test_chirp_bounds_checked_only_when_enabled(self):
        plan(chirp={"enabled": False, "high_us": 5})
        assert any("chirp.high_us" in p
                   for p in problems(chirp={"enabled": True, "high_us": 5}))

    def test_every_problem_is_reported_at_once(self):
        """One error per battery is a bad exchange rate."""
        found = problems(repeats=0, order={"mode": "nope"},
                         grid={"a": [900, 2000, 100], "b": [1000, 2000, 0]})
        assert len(found) >= 4


class TestLoading:
    def test_loads_json(self, tmp_path):
        path = tmp_path / "p.json"
        path.write_text(json.dumps({"schema_version": 1, "repeats": 4}))
        assert config.load(str(path))["repeats"] == 4

    def test_loads_yaml(self, tmp_path):
        yaml = pytest.importorskip("yaml")
        path = tmp_path / "p.yaml"
        path.write_text(yaml.safe_dump({"schema_version": 1, "repeats": 5}))
        assert config.load(str(path))["repeats"] == 5

    def test_loads_the_shipped_example(self):
        p = config.load("plans/coax_grid.yaml")
        assert p.n_grid_points == 44
        assert p["order"]["mode"] == "blocked_random"

    def test_empty_yaml_falls_back_to_defaults(self, tmp_path):
        pytest.importorskip("yaml")
        path = tmp_path / "empty.yaml"
        path.write_text("")
        assert config.load(str(path))["repeats"] == 1
