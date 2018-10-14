import math
from collections import namedtuple
from operator import itemgetter


def findOrientation(origin, p1, p2):
    '''
    Returns the orientation of the Point p1 with regards to Point p2 using origin.
    Negative if p1 is clockwise of p2 (asssuming origin is where the clock "hands originate.
    clockwise of p2 will insidcate whether it is farther away from the origin than p1
    :param p1:
    :param p2:
    :param origin: point of
    :return: integer
    '''
    return (((p2.x - origin.x) * (p1.y - origin.y)) - ((p1.x - origin.x) * (p2.y - origin.y)))


def findHull(bounded_rect_coords):
    '''
    Computes the points that make up the convex hull. It essentially sets an origin and then finds the point
    that is most "clockwise"
    :return:
    '''

    Point = namedtuple('Point', 'x y')
    points = []
    for _ in bounded_rect_coords:
        points.append(Point(_[0], _[1]))
    #print "POINTS: ", points

    hull_points = []
    start = points[0]
    min_x = start.x
    for p in points[1:]:
        if p.x < min_x:
            min_x = p.x
            start = p

    point = start
    hull_points.append(start)

    far_point = None
    while far_point is not start:
        #get the first point (initial max) to use to compare with others
        p1 = None
        for p in points:
            if p is point:
                continue
            else:
                p1 = p
                break

        far_point = p1

        for p2 in points:  #find second point to compare to
            #ensure we arent comparing to self or pivot points
            if p2 is point or p2 is p1:
                continue
            else:
                direction = findOrientation(point, far_point, p2)
                if direction > 0:
                    far_point = p2
        hull_points.append(far_point)
        point = far_point
    return hull_points


def main():
    """
    this is for debugger use only
    :return:NO!
    """
    # test cases
    # points =[[610, 260], [516, 129], [465, 64], [346, 64], [219, 67], [126, 262], [186, 128]]#no error
    # points = [[554, 238], [220, 269], [398, 166], [357, 392], [240, 163]]  # no error
    # points = [[335, 202], [498, 145], [202, 128], [250, 79], [477, 94]]  # no error
    # points = [[356, 351], [543, 313], [171, 286], [276, 199], [418, 177]]  # no error
    # points =[[54, 287], [165, 168], [255, 71], [283, 447], [458, 178], [381, 298], [158, 359]]#error
    # points = [[579,249],[29,172],[604,115],[130,73],[553,47],[231,24],[389,27]]#error box too wide
    # points = [[389, 71], [304, 64], [217, 65], [259, 122], [158, 108], [375, 122], [401, 94]] #requires abs angles?
    # points = [[379,302],[272,213],[563,176],[218,118],[490,83],[404,24],[308,56]]
    # points = [[443, 39], [305, 25], [174, 41], [150, 283], [324, 296], [490, 257], [139, 145]]
    # points = [[395, 92], [223, 36], [313, 55], [280, 186], [93, 97], [363, 142], [160, 146]]
    # points = [[491, 297], [217, 170], [315, 99], [383, 434], [223, 381], [131, 287], [515, 176]]
    # points = [[360, 139], [220,101], [155,63], [489,74], [391, 46], [221, 40], [296,30]]
    # points = [[293,150], [353,123], [183,119], [379,97],[282,75],[228,92],[335,82]]
    # points = [[445,135],[440,104],[432, 169],[336,170],[393,101],[345,107],[333,137]]
    points = [[395, 92], [223, 36], [313, 55], [280, 186], [93, 97], [363, 142], [160, 146]]

    # points = [[361,74],[240,66],[297,74],[248,51],[374,59],[369,44],[266,37]]
    # points = [[158, 336], [533, 349], [164, 200], [520, 211], [481, 114], [345, 92], [209, 107]]
    # points = [[131,415],[522,417],[510,253],[130,252],[175,133],[324,110],[472,134]]
    #points = [[471,198],[158,188],[316,205],[474,121],[426,68],[325,59],[221,64]]

    hull = findHull(points)
    hull.pop()  #hull wraps to check completion so pop to duplicate
    # print "hull",hull
    groups, size = findGroups(hull)
    # print "groups",groups
    if size != 7:
        target = [0, 0, 0, 0, 0, 0, 0]
    else:
        groupedAngles = findGroupAngles(groups)
        topCandidates = findTopCandidates(groupedAngles, 4)
        groupsNoDoubleLinks = findDuplicatesPerGroup(topCandidates)
        #groupsNoDoubleLinks = findDoubleLink(groupsWithDuplicates)
        target = reconstructTarget(groupsNoDoubleLinks)

        print "target", target


def getTarget(points):
    """
    this takes in 7 points in random order and then re-orders so that we can assign distances to them in the world coordinate
    frame for the perspective transform.
    :param points: centers of the bounded rectangles. buildTarget only works if there are 7
    :return: The same 7 points, just ordered starting from the bottom right leg going clockwise to the bottom left leg
    """
    # test cases
    # points =[[610, 260], [516, 129], [465, 64], [346, 64], [219, 67], [126, 262], [186, 128]]#no error
    # points = [[554, 238], [220, 269], [398, 166], [357, 392], [240, 163]]  # no error
    # points = [[335, 202], [498, 145], [202, 128], [250, 79], [477, 94]]  # no error
    # points = [[356, 351], [543, 313], [171, 286], [276, 199], [418, 177]]  # no error
    # points =[[54, 287], [165, 168], [255, 71], [283, 447], [458, 178], [381, 298], [158, 359]]#error
    # points = [[579,249],[29,172],[604,115],[130,73],[553,47],[231,24],[389,27]]#error box too wide

    # points = [[389, 71], [304, 64], [217, 65], [259, 122], [158, 108], [375, 122], [401, 94]]
    # points = [[379,302],[272,213],[563,176],[218,118],[490,83],[404,24],[308,56]]
    # points = [[443, 39], [305, 25], [174, 41], [150, 283], [324, 296], [490, 257], [139, 145]]
    #points = [[395, 92], [223, 36], [313, 55], [280, 186], [93, 97], [363, 142], [160, 146]]
    #points = [[158,336],[533,349],[164,200],[520,211],[481,114],[345,92],[209,107]]

    # points = [[491, 297], [217, 170], [315, 99], [383, 434], [223, 381], [131, 287], [515, 176]]
    # points = [[360, 139], [220,101], [155,63], [489,74], [391, 46], [221, 40], [296,30]]
    # points = [[293,150], [353,123], [183,119], [379,97],[282,75],[228,92],[335,82]]

    # points = [[445,135],[440,104],[432, 169],[336,170],[393,101],[345,107],[333,137]]
    # points = [[334, 231], [64, 211], [590, 216], [121, 144], [198, 112], [519, 153], [440, 119]]
    hull = findHull(points)
    hull.pop()
    #print "hull",hull
    groups, size = findGroups(hull)
    #print "groups",groups
    try:
        groupedAngles = findGroupAngles(groups)
        topCandidates = findTopCandidates(groupedAngles, 3)
        groupsNoDoubleLinks = findDuplicatesPerGroup(topCandidates)
        #groupsNoDoubleLinks = findDoubleLink(groupsWithDuplicates)
        target = reconstructTarget(groupsNoDoubleLinks)

        # groupedAngles = findGroupAngles(groups)
        # topCandidates = findTopCandidates(groupedAngles, 4)
        # groupsWithDuplicates = findDuplicatesPerGroup(topCandidates)
        # groupsNoDoubleLinks = findDoubleLink(groupsWithDuplicates)
        # target = reconstructTarget(groupsNoDoubleLinks)
        return [target, size]
    except:
        return [[0, 0, 0, 0, 0, 0, 0], 0]


def findDoubleLink(groups):
    L1 = []  #list of groups with 3 duplicates
    L2 = []  #everything less than 3 duplicates
    try:
        for line in groups:
            if line[4] == 3:
                L1.append(line)
            else:
                L2.append(line)
        if len(L2) > 2:  #no double links so order is fine. fail fast

            return groups
        else:  # there is a double link we need to address
            for dup in L1:
                d = 0
                for single in L2:
                    for point in dup:
                        if single[0] == point:
                            d += 1
                        if single[1] == point:
                            d += 1
                        if single[2] == point:
                            d += 1
                    if d == 2:  #found the bad double link
                        L1.remove(dup)
                        L2.append(L1[0])
                        L2[0].pop(4)
                        L2[1].pop(4)
                        L2[2].pop(4)
                        return findDuplicatesPerGroup(L2)

                d = 0
        groups.pop()
        groups[0].pop(4)
        groups[1].pop(4)
        groups[2].pop(4)
        return findDuplicatesPerGroup(groups)  #if false positive
    except:
        print "index error in findDoubleLink"


def reconstructTarget(groups):
    """
    We now have 1 group with 2 duplicates and 2 groups with 1 duplicate. We know the base has 2 duplicates so we have to
    figure out where the left and right are. since the hull is always built going counterclockwise this is essentially already
    know. the right side's duplicate will be in position [0] and the left sides duplicate will be in position [2]. since we are
    returning the full target we pop off the stuff we don't need like the angles and the number of duplicates per group and then
    return the built target
    :param groups: [[pointsx, pointsy, angle, int]]
    :return: [pointsx0,pointsy0][pointsx1,pointsy1]. the points are now in order
    """
    base = []  # 2,3,4
    right = []  # 0,1,2
    left = []  # 4,5,6

    #remove corner group if it exists
    try:
        for line in groups:
            if line[4] == 2:
                line.pop()
                line.pop()
                base = line

        if not base:
            print "NOBASE!!"
        else:
            for line in groups:
                if line[2] == base[0]:
                    line.pop()
                    line.pop()
                    right = line
                if line[0] == base[2]:
                    line.pop()
                    line.pop()
                    left = line

        target = [right, base, left]
        try:  # pop the duplicates
            coords = []
            for i in target:
                for j in i:
                    coords.append(j)
            coords.pop(2)
            coords.pop(5)
            return coords
        except:
            print "index error in reconstructTarget"
    except:
        print "nonetype error in reconstruct target"


def findDuplicatesPerGroup(groups):
    """
    At this point we have a few candidates for the "lines" of the target, but we still have to "hook them up" to actually
    recreate the target. Since there are only 7 points and 3 groups of 3 has 9 points we know that 2 poitns are overlapped.
    the ones that are overlapped will always be the corners so, if we can figure out how many duplicates are in each group then
    we can rebuild the target. should be 2 with 1 duplicate and 1 with 2 duplicates. we append the result to our return group as
    an int.
    :param groups: [[pointsx, pointsy, angle]]
    :return: [[pointsx, pointsy, angle, int,]]
    """
    L1 = []
    L2 = []
    for line in groups:  # build the small list of duplicate points (should be 2)
        for point in line:
            if point in L1 and point not in L2:
                L2.append(point)
            L1.append(point)

    for line in groups:
        num_duplicates = 0
        for duplicate in L2:
            for point in line:
                if not isinstance(point, float):
                    if point == duplicate:  #is duplicate point and not float
                        num_duplicates += 1

        line.append(num_duplicates)

    return groups


def findGroups(hull_points):
    """
    This finds all possible combinations of sides of length 3 around the hull. To do this we effectively create a circularly
    linked list and iterate through for a total of 7 combinations
    :param hull_points: all the points along the hull. 7 total and in order going clockwise. We will use this to our advantage
    later on when we reconstruct the target.
    :return: [[pointsx, pointsy]] returns the 7 groups of potential lines and the size. mostly for debugging
    """
    hull_size = len(hull_points)
    print "hull_size: ", hull_size
    start_node = 0
    groups = []
    for i in range(
        len(hull_points)
    ):  # with the modulo this is effectively a circularLinkedList. I getting linked combinations around the hull
        if (start_node == hull_size):
            break
        comb = []
        for j in range(3):
            j += start_node
            j = j % hull_size
            comb.append(hull_points[j])
        groups.append(comb)
        start_node += 1

    return groups, hull_size


def findGroupAngles(groups):
    """
    Here we are taking the groups of lines and we are checking for how "straight" they are. Since each group is made up of
    2 connected lines. we compare the angles of each of the lines relative to the x-axis and append the difference to the end
    of each group. 8 cases total plus when slopes are infinite and 0
    :param groups: [[pointsx, pointsy]]
    :return: [[pointsx, pointsy, angle]] groups with the appended difference in angles
    """
    angle = []
    for g in groups:
        # find m or the angle of intersect with the x axis (this is commonly referred to as m)

        m = [
            slope(g[0][0], g[1][0], g[0][1], g[1][1]),  # 0 & 1
            slope(g[1][0], g[2][0], g[1][1], g[2][1])
        ]  # 1 & 2

        dist = [
            findDist(g[0][0], g[1][0], g[0][1], g[1][1]),  # 0 & 1
            findDist(g[1][0], g[2][0], g[1][1], g[2][1])
        ]  # 1 & 2
        # for some fucked up reason a ratio of 1/3 or x%3 == 0 makes theta 0
        # so i just add an insignificant float to m[1] and that fixes it
        try:
            if ((m[0] / m[1] == 3) or (m[1] / m[0] == 3) or ((m[1] * m[0]) % 3 == 0)):
                m[1] += .00001
        except ZeroDivisionError:
            pass

        theta = [math.atan(m[0]), math.atan(m[1])]
        theta_degrees = [math.degrees(math.atan(m[0])), math.degrees(math.atan(m[1]))]

        # 4 cases for each permutation of slopes (pospos, posneg...etc)
        a = []
        i = 0
        for t in theta_degrees:
            a.append(t)

        if dist[0][0] > 0 and dist[0][1] > 0 and dist[1][0] < 0 and dist[1][1] > 0:
            angle.append(180 - math.fabs(a[1]) - math.fabs(a[0]))
        elif dist[0][0] > 0 and dist[0][1] > 0 and dist[1][0] > 0 and dist[1][1] > 0:
            angle.append(a[1] - a[0])
        elif dist[0][0] > 0 and dist[0][1] < 0 and dist[1][0] > 0 and dist[1][1] > 0:
            angle.append(math.fabs(a[1] - a[0]))
        elif dist[0][0] > 0 and dist[0][1] < 0 and dist[1][0] > 0 and dist[1][1] < 0:
            angle.append(math.fabs(a[0]) - math.fabs(a[1]))
        elif dist[0][0] < 0 and dist[0][1] > 0 and dist[1][0] < 0 and dist[1][1] > 0:
            angle.append(a[1] - a[0])
        elif dist[0][0] < 0 and dist[0][1] > 0 and dist[1][0] < 0 and dist[1][1] < 0:
            angle.append(math.fabs(a[0]) + a[1])
        elif dist[0][0] < 0 and dist[0][1] < 0 and dist[1][0] < 0 and dist[1][1] < 0:
            angle.append(a[1] - a[0])
        elif dist[0][0] < 0 and dist[0][1] < 0 and dist[1][0] > 0 and dist[1][1] < 0:
            angle.append(180 - math.fabs(a[1]) - math.fabs(a[0]))
        else:
            print 'bad_slope'
    i = 0
    for g in groups:  # attach the slopes to their respective groups
        try:
            groups[i].append(angle[i])
            i += 1
        except IndexError:
            print "index error in group angles"
    return groups


def findTopCandidates(groups, num_candidates):
    """
    This sorts the groups by their angles from least to greatest. the idea being that the smaller the angle, the higher
    the probability it is a straight line
    :param groups: [[pointsx, pointsy,angle]]
    :param num_candidates: int. how many "straightest" lines we want to evaluate further. just a fail fast thing
    :return: [[pointsx, pointsy, angle]]
    """
    candidates = []
    try:
        comb_sorted_by_slope = sorted(groups, key=itemgetter(3))
        for i in range(0, num_candidates):
            candidates.append(comb_sorted_by_slope[i])
        # print "top candidates",list(candidates)
    except IndexError:
        print "index error in top candidates"
    return candidates


def getAngle(x1, x2, y1, y2):
    #print "points", x1, x2, y1, y2
    x = math.fabs(x2 - x1)
    #print "x", x
    y = math.fabs(y2 - y1)
    #print "y", y
    #print "theta: ", math.atan(y / x)
    return math.atan(y / x)


def slope(x1, x2, y1, y2):

    # print "points",x1,x2,y1,y2
    # print "Raw Slope", float((y2 - y1)) / float((x2 - x1))
    try:
        return float((y2 - y1)) / float((x2 - x1))
    except ZeroDivisionError:
        return 0


def findDist(x1, x2, y1, y2):
    return [float(x2 - x1), float(y2 - y1)]


if __name__ == "__main__":
    main()
