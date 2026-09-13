from copy import deepcopy

import pytest

from panda_manipulation.pick_plan_pipeline import pre_place_height_candidate


def test_candidates_vary_height_without_mutating_nominal_or_final_pose():
    nominal = {"position": {"x": 0.5, "y": -0.3, "z": 0.3},
               "orientation": {"x": 1, "y": 0, "z": 0, "w": 0}}
    place = deepcopy(nominal)
    place["position"]["z"] = 0.1
    candidates = [pre_place_height_candidate(nominal, place, i, 0.04) for i in range(1, 5)]
    assert [c["position"]["z"] for c in candidates] == pytest.approx([0.3, 0.34, 0.26, 0.38])
    assert nominal["position"]["z"] == 0.3
    assert place["position"]["z"] == 0.1
    assert all(c["orientation"] == nominal["orientation"] for c in candidates)
    assert all(c["position"]["y"] == -0.3 for c in candidates)
    assert pre_place_height_candidate(nominal, place, 2, 0) == nominal


def test_candidate_never_descends_into_final_place():
    nominal = {"position": {"z": 0.13}}
    place = {"position": {"z": 0.1}}
    assert pre_place_height_candidate(nominal, place, 3, 0.04)["position"]["z"] == pytest.approx(0.15)


@pytest.mark.parametrize("step", [-1, float("nan"), float("inf")])
def test_invalid_height_step_rejected(step):
    with pytest.raises(ValueError):
        pre_place_height_candidate({}, {}, 1, step)
