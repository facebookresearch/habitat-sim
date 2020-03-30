import math

import hypothesis
import hypothesis.strategies as st
import numpy as np
import pytest

import habitat_sim


@pytest.fixture(scope="function")
def test_data():
    pf = habitat_sim.PathFinder()
    pf.load_nav_mesh(
        "data/scene_datasets/habitat-test-scenes/skokloster-castle.navmesh"
    )

    # The test point won't be deterministic
    return pf, pf.get_random_navigable_point()


@hypothesis.given(
    nudge=st.tuples(st.floats(-10, 10), st.floats(-2.5, 2.5), st.floats(-10, 10))
)
@hypothesis.settings(max_examples=int(1e3))
def test_snap_point(nudge, test_data):
    pf, start_pt = test_data

    pt = start_pt + nudge

    proj_pt = pf.snap_point(pt)
    hypothesis.assume(not math.isnan(proj_pt[0]))

    assert pf.is_navigable(proj_pt), "{} -> {} not navigable!".format(pt, proj_pt)


@hypothesis.given(nudge=st.floats(-1.0, 1.0))
@hypothesis.settings(max_examples=int(1e3))
def test_snap_point_vertical(nudge, test_data):
    pf, start_pt = test_data

    pt = start_pt.copy()
    pt[1] += nudge

    proj_pt = pf.snap_point_vertical(pt)

    if nudge < -0.05:
        assert math.isnan(proj_pt[0]), "Big nudge down, still navigable!"
    else:
        assert np.allclose(proj_pt, start_pt), "Did not find the same point"
