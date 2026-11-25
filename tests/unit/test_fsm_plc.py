import pytest

from control_lab.controllers import FSM
from control_lab.controllers.plc import CTU, F_TRIG, R_TRIG, TOF, TON, SRLatch, run_scans


def make_fsm():
    return FSM(
        "idle",
        {("idle", "start"): "run", ("run", "stop"): "idle", ("run", "fault"): "fault",
         ("fault", "reset"): "idle"},
        outputs={"idle": {"motor": 0}, "run": {"motor": 1}, "fault": {"motor": 0}},
    )


def test_fsm_transitions_and_outputs():
    f = make_fsm()
    assert f.fire("start") and f.state == "run" and f.output("motor") == 1
    assert f.fire("fault") and f.output("motor") == 0


def test_fsm_ignores_undefined_event_and_records_it():
    f = make_fsm()
    assert not f.fire("stop")
    assert f.state == "idle" and len(f.dropped) == 1


def test_fsm_structure_checks():
    f = make_fsm()
    assert f.unreachable_states() == set()
    assert f.deadlock_states() == set()
    assert f.can_reach("idle", "fault")
    g = FSM("a", {("a", "go"): "b", ("c", "go"): "a"})
    assert g.unreachable_states() == {"c"}
    assert g.deadlock_states() == {"b"}


def test_fsm_dot_and_table():
    f = make_fsm()
    assert "digraph" in f.to_dot()
    assert len(f.to_table()) == 4


def test_ton_timing():
    ton = TON(pt=1.0)
    out = [ton.update(True, 0.1) for _ in range(12)]
    assert out[:9] == [False] * 9
    assert out[10] is True
    assert ton.update(False, 0.1) is False


def test_tof_holds_after_falling_edge():
    tof = TOF(pt=0.5)
    assert tof.update(True, 0.1)
    held = [tof.update(False, 0.1) for _ in range(8)]
    assert held[0] is True and held[-1] is False


def test_edge_detectors():
    r, f = R_TRIG(), F_TRIG()
    seq = [False, True, True, False]
    assert [r.update(x) for x in seq] == [False, True, False, False]
    assert [f.update(x) for x in seq] == [False, False, False, True]


def test_ctu_counts_rising_edges_and_resets():
    c = CTU(pv=3)
    for x in [True, True, False, True, False, True]:  # three rising edges
        done = c.update(x)
    assert c.cv == 3 and done
    c.update(False, reset=True)
    assert c.cv == 0


def test_ctu_counts_edge_that_coincides_with_reset_release():
    c = CTU(pv=5)
    c.update(True)
    c.update(False, reset=True)
    c.update(True)
    assert c.cv == 1


def test_sr_latch_dominance():
    sr = SRLatch(reset_dominant=True)
    assert sr.update(True, False)
    assert not sr.update(True, True)


class Blinker:
    def __init__(self):
        self.on = TON(0.5)
        self.q = False

    def scan(self, inputs, dt):
        run = self.on.update(inputs["enable"], dt)
        self.q = run
        return {"q": self.q}


def test_run_scans_returns_time_history():
    recs = run_scans(Blinker(), lambda t: {"enable": t > 0.2}, 0.01, 1.5)
    assert recs[0]["t"] == 0.0 and len(recs) == 150
    first_true = next(r["t"] for r in recs if r["out"]["q"])
    assert first_true == pytest.approx(0.7, abs=0.03)
