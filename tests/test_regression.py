"""Whole-pipeline regression test (brief Section 8).

Takes a scan, runs the full pipeline, and checks the zone floor area against
an expected value within 2%. Skipped until the pipeline and a fixture scan exist.
"""

import pytest

EXPECTED_FLOOR_AREA_M2 = None  # set once a fixture scan has a taped floor area
TOLERANCE = 0.02


@pytest.mark.skip(reason="pipeline not implemented yet")
def test_floor_area_within_tolerance():
    ...
