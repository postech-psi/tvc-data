"""
Sequence expansion tests.

The statistical properties are the point of this module, so they are asserted
directly: a repeat must be a *complete* replicate of the grid, repeats must be
ordered differently from each other, and the whole thing must be reproducible
from the one seed recorded in the manifest.
"""

import pytest

from tvcbench import sequence
from tvcbench.config import Plan
from tvcbench.sequence import (
    CHIRP,
    IDLE_POST,
    IDLE_PRE,
    POST_STOP,
    RAMP_BETWEEN,
    RAMP_DOWN,
    REFERENCE,
    STEP,
    TARE,
    WARMUP,
)

QUIET = {"tare": {"seconds": 0}, "warmup": {"seconds": 0},
         "idle": {"pre_s": 0, "post_s": 0}, "chirp": {"enabled": False},
         "post_stop": {"seconds": 0}}


def plan(**overrides):
    base = {"schema_version": 1, "grid": {"a": [1300, 1400, 100],
                                          "b": [1000, 1200, 100]}}
    base.update(overrides)
    return Plan(base)


def bare(**overrides):
    """A plan with only the measurement steps, so orderings are easy to read."""
    merged = dict(QUIET)
    merged.update(overrides)
    return plan(**merged)


def kinds(segments):
    return [s.kind for s in segments]


def measurement_kinds(segments):
    """Just the steps and references -- the ramp down is always appended."""
    return [s.kind for s in segments if s.kind in (STEP, REFERENCE)]


def steps_of(segments, sweep=None):
    return [(s.a_us, s.b_us) for s in segments
            if s.kind == STEP and (sweep is None or s.sweep == sweep)]


class TestStructure:
    def test_full_run_is_bracketed_by_tare_idle_and_ramp(self):
        segs, _ = sequence.expand(plan())
        k = kinds(segs)
        assert k[0] == TARE
        assert k[1] == WARMUP
        assert k[2] == IDLE_PRE
        assert k[-2:] == [IDLE_POST, POST_STOP]
        assert RAMP_DOWN in k
        # The ramp down comes after every measurement step.
        assert k.index(RAMP_DOWN) > max(i for i, x in enumerate(k) if x == STEP)

    def test_segment_ids_are_dense_and_ordered(self):
        """seg_id is the join key for every stream; gaps or repeats would break it."""
        segs, _ = sequence.expand(plan())
        assert [s.seg_id for s in segs] == list(range(len(segs)))

    def test_ramp_down_reaches_minimum(self):
        segs, _ = sequence.expand(plan())
        last_ramp = [s for s in segs if s.kind == RAMP_DOWN][-1]
        assert (last_ramp.a_us, last_ramp.b_us) == (1000, 1000)

    def test_ramp_between_bridges_consecutive_sweeps(self):
        segs, _ = sequence.expand(bare(repeats=2), seed=7)
        idx = kinds(segs).index(RAMP_BETWEEN)
        before = segs[idx - 1]
        after = [s for s in segs[idx:] if s.kind == STEP][0]
        ramp = [s for s in segs if s.kind == RAMP_BETWEEN]

        assert ramp[-1].a_us == after.a_us and ramp[-1].b_us == after.b_us
        assert before.kind == STEP        # ramps start where the last step ended

    def test_no_ramp_between_for_a_single_sweep(self):
        segs, _ = sequence.expand(bare(repeats=1))
        assert RAMP_BETWEEN not in kinds(segs)

    def test_warmup_is_the_only_unrecorded_kind(self):
        segs, _ = sequence.expand(plan())
        assert {s.kind for s in segs if not s.record} == {WARMUP}

    def test_zero_length_phases_are_omitted(self):
        segs, _ = sequence.expand(bare())
        assert not ({TARE, WARMUP, IDLE_PRE, IDLE_POST, CHIRP, POST_STOP}
                    & set(kinds(segs)))

    def test_post_stop_is_last_and_at_minimum(self):
        """It is the window with nothing commanding the outputs; minimum is where
        a lapsed actuator-test command leaves them."""
        segs, _ = sequence.expand(plan(post_stop={"seconds": 7}))
        last = segs[-1]
        assert last.kind == POST_STOP
        assert (last.a_us, last.b_us) == (1000, 1000)
        assert last.dwell_s == 7
        assert not last.adaptive
        assert last.record

    def test_post_stop_time_is_costed_into_the_plan(self):
        """`plan show` must not understate a run by the length of its tail."""
        with_tail = sequence.expand(bare(post_stop={"seconds": 10}))[0]
        without = sequence.expand(bare())[0]
        assert (sequence.duration_s(with_tail)[1]
                == pytest.approx(sequence.duration_s(without)[1] + 10))


class TestChirp:
    def test_alternates_high_and_low_at_both_ends(self):
        segs, _ = sequence.expand(
            plan(chirp={"enabled": True, "high_us": 1400, "cycles": 2,
                        "half_period_s": 0.5}))
        chirps = [(s.a_us, s.b_us) for s in segs if s.kind == CHIRP]
        assert chirps == [(1400, 1400), (1000, 1000)] * 4      # 2 cycles at each end

    def test_drives_both_rotors_together(self):
        segs, _ = sequence.expand(plan())
        assert all(s.a_us == s.b_us for s in segs if s.kind == CHIRP)

    def test_can_be_disabled(self):
        segs, _ = sequence.expand(plan(chirp={"enabled": False}))
        assert CHIRP not in kinds(segs)


class TestOrdering:
    def test_sequential_walks_a_outer_b_inner(self):
        segs, _ = sequence.expand(bare(order={"mode": "sequential"}))
        assert steps_of(segs) == [(1300, 1000), (1300, 1100), (1300, 1200),
                                  (1400, 1000), (1400, 1100), (1400, 1200)]

    def test_blocked_random_gives_each_repeat_a_complete_replicate(self):
        """
        The statistical guarantee: every command appears once per repeat.

        That is what balances the battery drain across levels -- a repeat missing
        points, or doubling them, would reintroduce the confound.
        """
        segs, _ = sequence.expand(bare(repeats=3, order={"mode": "blocked_random"}),
                                  seed=11)
        expected = sorted(sequence.grid_points(bare()))
        for sweep in (1, 2, 3):
            assert sorted(steps_of(segs, sweep)) == expected

    def test_blocked_random_reorders_between_repeats(self):
        segs, _ = sequence.expand(
            bare(repeats=4, grid={"a": [1300, 1600, 100], "b": [1000, 2000, 100]},
                 order={"mode": "blocked_random"}), seed=3)
        orders = [tuple(steps_of(segs, s)) for s in (1, 2, 3, 4)]
        assert len(set(orders)) == 4

    def test_random_reuses_one_order_for_every_repeat(self):
        segs, _ = sequence.expand(bare(repeats=3, order={"mode": "random"}), seed=5)
        assert steps_of(segs, 1) == steps_of(segs, 2) == steps_of(segs, 3)

    def test_random_actually_shuffles(self):
        big = bare(grid={"a": [1300, 1600, 100], "b": [1000, 2000, 100]},
                   order={"mode": "random"})
        segs, _ = sequence.expand(big, seed=5)
        assert steps_of(segs) != sequence.grid_points(big)


class TestReproducibility:
    def test_same_seed_gives_the_same_sequence(self):
        a, _ = sequence.expand(bare(repeats=2), seed=42)
        b, _ = sequence.expand(bare(repeats=2), seed=42)
        assert [s.as_dict() for s in a] == [s.as_dict() for s in b]

    def test_different_seeds_give_different_sequences(self):
        big = bare(grid={"a": [1300, 1600, 100], "b": [1000, 2000, 100]})
        assert steps_of(sequence.expand(big, seed=1)[0]) != \
            steps_of(sequence.expand(big, seed=2)[0])

    def test_zero_seed_resolves_to_a_recorded_one(self):
        """A run must be reproducible even when the plan asked for a fresh seed."""
        _, seed = sequence.expand(bare(order={"mode": "blocked_random", "seed": 0}))
        assert seed > 0
        replay, _ = sequence.expand(bare(), seed=seed)
        assert steps_of(replay) == steps_of(
            sequence.expand(bare(), seed=seed)[0])

    def test_explicit_seed_is_returned_unchanged(self):
        _, seed = sequence.expand(bare(order={"seed": 20260725}))
        assert seed == 20260725


class TestReferenceRevisits:
    def test_inserted_every_n_steps(self):
        segs, _ = sequence.expand(
            bare(grid={"a": [1300, 1300, 100], "b": [1000, 2000, 100]},
                 order={"mode": "sequential"},
                 reference={"a": 1500, "b": 1500, "every_n_steps": 4}))
        # 11 steps: references land after the 4th and 8th, not after the last.
        assert measurement_kinds(segs) == (
            [STEP] * 4 + [REFERENCE] + [STEP] * 4 + [REFERENCE] + [STEP] * 3)

    def test_reference_carries_the_configured_command(self):
        segs, _ = sequence.expand(
            bare(reference={"a": 1500, "b": 1450, "every_n_steps": 2}))
        ref = [s for s in segs if s.kind == REFERENCE][0]
        assert (ref.a_us, ref.b_us) == (1500, 1450)

    def test_never_appended_after_the_final_step_of_a_sweep(self):
        """The end-of-sweep ramp already covers that transition."""
        segs, _ = sequence.expand(
            bare(grid={"a": [1300, 1300, 100], "b": [1000, 1300, 100]},
                 order={"mode": "sequential"},
                 reference={"a": 1500, "b": 1500, "every_n_steps": 4}))
        assert measurement_kinds(segs) == [STEP] * 4

    def test_disabled_by_default(self):
        segs, _ = sequence.expand(bare())
        assert REFERENCE not in kinds(segs)


class TestDwellBounds:
    def test_fixed_mode_pins_all_three(self):
        segs, _ = sequence.expand(bare(dwell={"mode": "fixed", "fixed_s": 4.0}))
        step = [s for s in segs if s.kind == STEP][0]
        assert (step.dwell_s, step.min_dwell_s, step.max_dwell_s) == (4.0, 4.0, 4.0)
        assert not step.adaptive

    def test_sem_target_spreads_min_to_max(self):
        segs, _ = sequence.expand(
            bare(dwell={"mode": "sem_target", "min_s": 2.0, "max_s": 8.0,
                        "target_thrust_sem_n": 0.05}))
        step = [s for s in segs if s.kind == STEP][0]
        assert (step.min_dwell_s, step.max_dwell_s) == (2.0, 8.0)
        assert step.adaptive

    def test_fixed_phases_are_never_adaptive(self):
        segs, _ = sequence.expand(
            plan(dwell={"mode": "sem_target", "target_thrust_sem_n": 0.05}))
        assert not any(s.adaptive for s in segs if s.kind not in (STEP, REFERENCE))

    def test_duration_collapses_when_dwell_is_fixed(self):
        lo, nominal, hi = sequence.duration_s(sequence.expand(plan())[0])
        assert lo == nominal == hi

    def test_duration_spreads_under_adaptive_dwell(self):
        segs, _ = sequence.expand(
            plan(dwell={"mode": "sem_target", "min_s": 2.0, "max_s": 8.0,
                        "target_thrust_sem_n": 0.05}))
        lo, nominal, hi = sequence.duration_s(segs)
        assert lo < nominal < hi

    def test_duration_matches_the_sum_of_segments(self):
        segs, _ = sequence.expand(plan())
        assert sequence.duration_s(segs)[1] == pytest.approx(
            sum(s.dwell_s for s in segs))


class TestSummary:
    def test_counts_every_kind(self):
        segs, _ = sequence.expand(plan(repeats=2))
        counts = sequence.summarise(segs)
        assert counts[STEP] == 12          # 6 grid points x 2 repeats
        assert counts[TARE] == 1
        assert sum(counts.values()) == len(segs)


class TestBatteryEstimate:
    TABLE = {(1300, 1000): 1.0, (1300, 2000): 20.0}

    def test_exact_lookup_wins(self):
        assert sequence.lookup_current(self.TABLE, 1300, 1000) == 1.0

    def test_falls_back_to_the_nearest_measured_point(self):
        assert sequence.lookup_current(self.TABLE, 1300, 1950) == 20.0

    def test_no_table_means_no_estimate(self):
        assert sequence.lookup_current(None, 1300, 1000) is None
        assert sequence.estimate_charge_mah([], None) is None

    def test_integrates_current_over_planned_time(self):
        segs, _ = sequence.expand(
            bare(grid={"a": [1300, 1300, 100], "b": [1000, 1000, 100]},
                 order={"mode": "sequential"},
                 dwell={"mode": "fixed", "fixed_s": 3600.0}))
        # One step, one hour, one amp -> 1000 mAh, plus the sub-second ramp down.
        assert sequence.estimate_charge_mah(segs, self.TABLE) == pytest.approx(
            1000.0, rel=1e-3)

    def test_worst_case_uses_the_maximum_dwell(self):
        segs, _ = sequence.expand(
            bare(grid={"a": [1300, 1300, 100], "b": [1000, 1000, 100]},
                 dwell={"mode": "sem_target", "min_s": 2, "max_s": 8,
                        "target_thrust_sem_n": 0.05}))
        nominal = sequence.estimate_charge_mah(segs, self.TABLE)
        worst = sequence.estimate_charge_mah(segs, self.TABLE, use="max_dwell_s")
        assert worst > nominal

    def test_reads_the_projects_measured_map(self):
        table = sequence.current_map_from_csv("out/pwm_thrust_torque_map.csv")
        assert table and all(len(k) == 2 for k in table)
        assert all(c >= 0 for c in table.values())

    def test_missing_columns_yield_no_table(self, tmp_path):
        path = tmp_path / "junk.csv"
        path.write_text("x,y\n1,2\n")
        assert sequence.current_map_from_csv(str(path)) is None
