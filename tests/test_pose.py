# coding=utf-8
import math
from nose.tools import assert_almost_equal
from Rotation_Matrix.calc_pose import calculate_translation


# calculations:
# mag = sqrt(100^2 + 150^2- 2*100*150×cos(30))
# theta = asin(150 × sin(30) ÷ mag)
def test_translation_behind_right():
    (theta, mag) = calculate_translation(math.radians(30), 150, 100)
    assert_almost_equal(theta, -68.26, places=1)
    assert_almost_equal(mag, 80.7, places=1)


def test_translation_behind_left():
    (theta, mag) = calculate_translation(math.radians(-30), 150, 100)
    assert_almost_equal(theta, 68.26, places=1)
    assert_almost_equal(mag, 80.7, places=1)


def test_translation_ahead():
    (theta, mag) = calculate_translation(math.radians(0), 150, 200)
    assert_almost_equal(theta, -180)
    assert_almost_equal(mag, 50, places=1)


def test_translation_ahead_right():
    (theta, mag) = calculate_translation(math.radians(5), 150, 200)
    assert_almost_equal(theta, -155.5, places=1)
    assert_almost_equal(mag, 52.2, places=1)


def test_translation_ahead_left():
    (theta, mag) = calculate_translation(math.radians(-5), 150, 200)
    assert_almost_equal(theta, 155.5, places=1)
    assert_almost_equal(mag, 52.2, places=1)
