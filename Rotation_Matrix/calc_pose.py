import math


def calculate_translation(y_theta, z_dist, approach_dist):
    """
    Returns the translation vector needed to reach the "approach point" (rad, dist)
    :param y_theta: rotation of platform (rad)
    :param z_dist: distance to platform (cm)
    :param approach_dist: desired distance in front of platform (cm)
    :return:
    """
    # find dist to approach point using law of cosines
    dist = math.sqrt(approach_dist**2 + z_dist**2 - 2 * approach_dist * z_dist * math.cos(y_theta))
    rel_angle = math.asin(approach_dist * math.sin(y_theta) / dist)

    angle = -y_theta - rel_angle

    # handle case when we're in front of the approach point
    if z_dist < approach_dist:
        angle = -angle + math.copysign(math.pi, angle)

    return math.degrees(angle), dist
