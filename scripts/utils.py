from math import sqrt

def dist_point_to_segment(point, seg_start, seg_end):
    x0, y0 = point
    x1, y1 = seg_start
    x2, y2 = seg_end
    return abs((x2-x1)*(y1-y0) - (x1-x0)*(y2-y1)) / sqrt((x2-x1)**2 + (y2-y1)**2)
