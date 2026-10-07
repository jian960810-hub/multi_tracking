import ast
import math
from pathlib import Path

import numpy as np
import pytest

from multi_tracking.tracking_core import AdaBoostModel, MultiTracker, pair_legs, scan_segments, segment_features

ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / 'resource/adaboost_trained_data_mess_430.txt'


@pytest.mark.parametrize('count', [1, 90, 360, 720, 1080])
def test_variable_scan_size_and_empty_returns(count):
    assert scan_segments([float('inf')] * count, 0., .01, .1, 10.) == []
    assert scan_segments([0.] * count, 0., .01, .1, 10.) == []


def test_invalid_ray_breaks_cluster_and_limits_are_checked():
    values = [1.] * 3 + [float('nan'), -1., 0., float('inf'), 20., .01] + [1.] * 3
    result = scan_segments(values, 0., .001, .1, 10.)
    assert [len(s) for s in result] == [3, 3]
    with pytest.raises(ValueError):
        scan_segments([1.], 0., 0., .1, 10.)


def test_seam_wrap_only_on_full_circle():
    values = [1., 1.] + [float('inf')] * 356 + [1., 1.]
    assert [len(s) for s in scan_segments(values, 0., math.pi / 180, .1, 10.)] == [4]
    assert scan_segments(values, 0., .001, .1, 10.) == []


@pytest.mark.parametrize('xy', [np.zeros((3, 2)), np.array([[1, 0], [1, 0], [1, 1]]),
                              np.array([[1, 0], [1, 1], [1, 2]])])
def test_degenerate_features_are_finite(xy):
    assert np.isfinite(segment_features(xy, 0., 0.)).all()


def legacy_functions():
    source = ROOT / 'test/data/legacy_real_time_6_5.py'
    names = {'isbox', 'boundary_std', 'boundary_length', 'mean_curvature', 'mean_angular',
             'mean_average_deviation_from_median', 'linearity', 'circularity'}
    tree = ast.parse(source.read_text())
    functions = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name in names]
    namespace = {'np': np, 'math': math, 'train_data': np.loadtxt(MODEL), 'classified_num': 100}
    # Only the named pure numeric functions; never import or run the ROS 1 node.
    exec(compile(ast.Module(body=functions, type_ignores=[]), str(source), 'exec'), namespace)
    return namespace


def test_original_feature_values_and_classifier_scores_are_preserved():
    old = legacy_functions()
    model = AdaBoostModel(MODEL)
    rng = np.random.default_rng(183)
    for _ in range(30):
        n = int(rng.integers(4, 30))
        theta = np.linspace(-1.0, 1.0, n)
        xy = np.column_stack((.12 * np.cos(theta), .12 * np.sin(theta)))
        xy += rng.normal(0, .001, xy.shape) + [1., .3]
        expected = [n, np.linalg.norm(np.std(xy, axis=0)), old['boundary_std'](xy, n),
                    old['circularity'](xy, n, 0), old['circularity'](xy, n, 1),
                    old['boundary_length'](xy, n), old['mean_average_deviation_from_median'](xy, n),
                    .7, .8, old['linearity'](xy, n), np.linalg.norm(xy[0] - xy[-1]),
                    old['mean_angular'](xy, n), old['mean_curvature'](xy, n)]
        actual = segment_features(xy, .7, .8)
        np.testing.assert_allclose(actual, expected, rtol=1e-8, atol=1e-10)
        assert model.score(actual) == pytest.approx(old['isbox'](*actual))


def test_single_segment_and_no_segments_do_not_index_out_of_bounds():
    model = AdaBoostModel(MODEL)
    assert model.detect([]).shape == (0, 2)
    assert model.detect([np.array([[1., 0], [1., .01], [1., .02]])]).shape[1] == 2


def test_bad_model_rejected(tmp_path):
    for i, data in enumerate([np.zeros((4, 100)), np.ones((5, 0)), np.full((5, 1), np.nan),
                              np.array([[1.5], [1.], [1.], [1.], [0.]]),
                              np.array([[1.], [1.], [1.], [1.], [2.]])]):
        path = tmp_path / f'{i}.txt'
        np.savetxt(path, data)
        with pytest.raises(ValueError):
            AdaBoostModel(path)


def test_unpaired_legs_remain_available():
    points, pairs = pair_legs([[0, 0], [.2, 0], [3, 0], [5, 0]])
    assert pairs == 1
    np.testing.assert_allclose(points, [[.1, 0], [3, 0], [5, 0]])


def test_zero_cost_match_and_stationary_track_has_no_injected_acceleration():
    tracker = MultiTracker()
    for _ in range(50):
        tracks = tracker.update([[1., -.15], [1., .15]], .2)
        assert len(tracks) == 1
        assert tracks[0].id == 0
        np.testing.assert_allclose(tracks[0].position, [1., 0.], atol=1e-12)
    assert tracks[0].pair_hits == 50


def test_multiple_births_do_not_suppress_each_other():
    tracker = MultiTracker()
    tracks = tracker.update([[1, -.15], [1, .15], [3, -.15], [3, .15]], .2)
    assert len(tracks) == 2
    assert [t.id for t in tracks] == [0, 1]


def test_missing_detections_predict_and_eventually_delete():
    tracker = MultiTracker()
    tracker.update([[1, -.15], [1, .15]], .2)
    for _ in range(8):
        assert len(tracker.update([], .2)) == 1
    assert tracker.update([], .2) == []
    assert tracker.update([[1, -.15], [1, .15]], .2)[0].id == 1


def test_single_leg_can_update_but_cannot_create_person():
    tracker = MultiTracker()
    assert tracker.update([[1, 0]], .2) == []
    tracker.update([[1, -.15], [1, .15]], .2)
    tracks = tracker.update([[1, 0]], .2)
    assert len(tracks) == 1 and tracks[0].pair_hits == 1
    assert tracks[0].history[-1]


def test_motion_and_covariance_remain_finite_and_positive():
    tracker = MultiTracker()
    for i in range(100):
        x = 1. + i * .02
        dt = .1 if i % 2 else .2
        tracks = tracker.update([[x, -.15], [x, .15]], dt)
        assert len(tracks) == 1
        assert np.linalg.eigvalsh(tracks[0].covariance).min() > -1e-10
        assert np.isfinite(tracks[0].state).all()
    assert abs(tracks[0].position[0] - x) < .1


def test_assignment_is_one_to_one_and_gates_far_detection():
    tracker = MultiTracker(association_gate=10.)
    tracker.update([[1, -.15], [1, .15], [4, -.15], [4, .15]], .1)
    tracks = tracker.update([[1, -.15], [1, .15], [20, -.15], [20, .15]], .1)
    by_id = {t.id: t for t in tracks}
    assert by_id[0].history[-1]
    assert not by_id[1].history[-1]
    assert len(tracks) == 3
    assert len(set(t.id for t in tracks)) == 3


def test_replay_supplied_benchmark_without_nonfinite_states():
    data = np.loadtxt(ROOT / 'test/data/stationary_simple_bencnmark.txt', delimiter=',')
    frames = data.reshape(-1, 360, 2)
    model = AdaBoostModel(MODEL)
    tracker = MultiTracker()
    # Every recorded frame, including inf returns, passes through the new core.
    for frame in frames:
        segments = scan_segments(frame[:, 1], float(frame[0, 0]),
                                 float(frame[1, 0] - frame[0, 0]), .01, 10.)
        tracks = tracker.update(model.detect(segments), .2)
        assert all(np.isfinite(t.state).all() for t in tracks)
