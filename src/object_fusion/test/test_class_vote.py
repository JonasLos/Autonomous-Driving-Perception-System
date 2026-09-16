"""The per-id class vote: a one- or two-frame relabel must not resize the box."""
from object_fusion.class_vote import ClassVote

TRUCK, TRAIN, CAR = (8, "truck"), (6, "train"), (2, "car")


def run(vote, tid, labels, t0=0.0, dt=0.1, score=0.8):
    return [vote.vote(tid, t0 + i * dt, cid, name, score) for i, (cid, name) in enumerate(labels)]


def test_the_truck_that_became_a_train_stays_a_truck():
    """adps_2026-08-25_11-58-32: 39 frames truck, then 2 frames train at ~9 m."""
    v = ClassVote()
    out = run(v, "100", [TRUCK] * 39 + [TRAIN] * 2)
    assert out[-2:] == [TRUCK, TRUCK]
    assert v.overridden == 2


def test_a_genuine_reclassification_takes_over_once_it_holds_the_window():
    v = ClassVote(window_s=2.0)
    out = run(v, "7", [CAR] * 20 + [TRUCK] * 25)
    assert out[20] == CAR, "one frame of truck does not outvote 2 s of car"
    assert out[-1] == TRUCK, "after the window turns over, truck wins"


def test_first_label_and_ties_keep_the_current_label():
    v = ClassVote()
    assert v.vote("1", 0.0, *CAR, 0.5) == CAR
    assert v.vote("1", 0.1, *TRUCK, 0.5) == TRUCK, "tie: the current label is kept"


def test_score_weighting():
    v = ClassVote()
    v.vote("1", 0.0, *CAR, 0.3)
    v.vote("1", 0.1, *CAR, 0.3)
    assert v.vote("1", 0.2, *TRUCK, 0.9) == TRUCK


def test_ids_are_independent_and_empty_id_passes_through():
    v = ClassVote()
    run(v, "a", [TRUCK] * 10)
    assert v.vote("b", 1.0, *CAR, 0.8) == CAR
    assert v.vote("", 1.1, *TRAIN, 0.8) == TRAIN


def test_rewind_and_stale_ids_reset():
    v = ClassVote(forget_after_s=2.0)
    run(v, "1", [TRUCK] * 10, t0=100.0)
    assert v.vote("1", 10.0, *TRAIN, 0.8) == TRAIN, "bag rewind clears the history"
    v2 = ClassVote(forget_after_s=2.0)
    run(v2, "1", [TRUCK] * 10)
    v2.vote("2", 10.0, *CAR, 0.8)
    assert v2.vote("1", 10.1, *TRAIN, 0.8) == TRAIN, "an id unseen for > 2 s starts over"
