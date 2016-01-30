#!/usr/bin/env python

from __future__ import division

import sys, os
from common_utils import *
import sliding_window_back_removal as sw_back
import sliding_window_bottom_top_removal as sw_bt
import sliding_window_side_removal as sw_side
import matplotlib.pyplot as plt
from bisect import bisect
import random


top_bottom_space = 0.24
left_right_space = 0.45
plane_removal_threshold = 0.03
two_wall_space_tolerance = 0.05

args = [i for i in sys.argv]

if "-silence" in args:
    args.remove("-silence")
    sys.stdout = open(os.devnull, 'w')

if "-nocc" in args:
    args.remove("-nocc")
    findcc = True
else:
    findcc = False

render = len(args)<3

if len(args)<2:
    print "Usage: \"program arg1 [arg2]\", where arg1 is input file name and arg2 (optional) is output file name (to be created)"
    quit()

print ""

print "assumed top and bottom space is:", top_bottom_space
print "assumed left and right space is:", left_right_space

combo = read_pcd_file(args[1], ['xyzrgb', 'xyz', 'xy', 'yz', 'xz', 'x', 'y', 'z'])
xyzrgb, xyz, xy, yz, xz, x, y, z = combo

if render:
    render_3d_scatter(xyzrgb, proportion=0.1).set_title("original cloud")

print ""

#=================back======================
ret = sw_back.remove_back(xyz, xy, x, y, z)
min_z, min_dist, sampled_z, all_dists = ret
if min_dist < plane_removal_threshold:
    remove_idxs = remove_plane_idx(xyz, min_z, 'xy')
    combo = select_each(combo, remove_idxs)
    xyzrgb, xyz, xy, yz, xz, x, y, z = combo
    print "Back removed at", min_z

    print "removing points behind back plane"
    _, in_front_idxs = filter_val_idx(lambda z: z<min_z, z)
    combo = select_each(combo, in_front_idxs)
    xyzrgb, xyz, xy, yz, xz, x, y, z = combo
else:
    print "No back detected"

#plt.plot(sampled_z, all_dists)
#plt.show()


if render:
    render_3d_scatter(xyzrgb, proportion=0.13).set_title("back removed")

print ""

first_side_remove_loc = None
second_side_remove_loc = None

#=================side======================
ret = sw_side.remove_side(xyz, yz, x, y, z)
min_x, min_dist, sampled_x, all_dists = ret
#plt.figure()
#plt.plot(sampled_x, all_dists)
#plt.show()
if min_dist < plane_removal_threshold:
    remove_idxs = remove_plane_idx(xyz, min_x, 'yz')
    combo = select_each(combo, remove_idxs)
    xyzrgb, xyz, xy, yz, xz, x, y, z = combo
    first_side_detected = True
    first_side_remove_loc = min_x
    print "Side removed at", min_x
else:
    first_side_detected = False
    print "No side detected"

#=================side 2====================
if (left_right_space is not None) and first_side_detected:
    x_range = [sampled_x[0], sampled_x[-1]]
    assert x_range[0]<x_range[1]
    first_is_left = (x_range[1]-min_x) > (min_x-x_range[0])
    if first_is_left:
        new_x_expect = min_x + left_right_space
    else:
        new_x_expect = min_x - left_right_space
    assert len(sampled_x) == len(all_dists)
    idx_in_sample = bisect(sampled_x, new_x_expect)
    if idx_in_sample==len(all_dists):
        idx_in_sample -= 1
    local_min_idx, _ = local_min(all_dists, idx_in_sample)
    local_min_loc = sampled_x[local_min_idx]
    if abs(local_min_loc-new_x_expect)<=two_wall_space_tolerance:
        remove_idxs = remove_plane_idx(xyz, local_min_loc, 'yz')
        combo = select_each(combo, remove_idxs)
        xyzrgb, xyz, xy, yz, xz, x, y, z = combo
        local_min_works = True
        second_side_remove_loc = local_min_loc
        print "Local min: another side removed at", local_min_loc
    else:
        local_min_works = False
        print "Local min: another side not in expected location, expect:", new_x_expect, "actual:", local_min_loc


# old other side removal code
if (left_right_space is not None) and first_side_detected and (not local_min_works):
    x_range = [sampled_x[0], sampled_x[-1]]
    assert x_range[0]<x_range[1]
    first_is_left = (x_range[1]-min_x) > (min_x-x_range[0])
    if first_is_left:
        new_x_expect = min_x + left_right_space
    else:
        new_x_expect = min_x - left_right_space
    print "New search: expect the other side to be at "+str(new_x_expect)+"..."
    ret = sw_side.remove_side(xyz, yz, x, y, z, new_x_expect)
    if ret is None:
        print "New search: another side not in expected location"
    else:
        min_x, min_dist, _, _ = ret
        if min_dist < plane_removal_threshold:
            remove_idxs = remove_plane_idx(xyz, min_x, 'yz')
            combo = select_each(combo, remove_idxs)
            xyzrgb, xyz, xy, yz, xz, x, y, z = combo
            second_bt_remove_loc = min_x
            print "New search: another side removed at", min_x
        else:
            print "New search: no side detected"



print "remove all points that are not between two sides"

if first_side_remove_loc is not None:
    if second_side_remove_loc is None:
        if first_is_left:
            second_side_remove_loc = float('inf')
        else:
            second_side_remove_loc = -float('inf')
    if first_is_left:
        left = first_side_remove_loc
        right = second_side_remove_loc
    else:
        left = second_side_remove_loc
        right = first_side_remove_loc
    print "keeping only points between", left, "and", right    
    _, in_between_idxs = filter_val_idx(lambda x: left<x<right, x)
    combo = select_each(combo, in_between_idxs)
    xyzrgb, xyz, xy, yz, xz, x, y, z = combo


if render:
    render_3d_scatter(xyzrgb, proportion=0.2).set_title("back and side removed")

print ""


first_bt_remove_loc = None
second_bt_remove_loc = None

#==============top/bottom===================
ret = sw_bt.remove_bottom_top(xyz, xz, x, y, z)
min_y, min_dist, sampled_y, all_dists = ret
if min_dist < plane_removal_threshold:
    remove_idxs = remove_plane_idx(xyz, min_y, 'xz')
    combo = select_each(combo, remove_idxs)
    xyzrgb, xyz, xy, yz, xz, x, y, z = combo
    first_bt_detected = True
    first_bt_remove_loc = min_y
    print "Top/bottom removed at", min_y
else:
    first_bt_detected = False
    print "No top/bottom detected"

#==============top/bottom 2=================
if (top_bottom_space is not None) and first_bt_detected:
    y_range = [sampled_y[0], sampled_y[-1]]
    assert y_range[0]<y_range[1]
    removed_is_top = (y_range[1]-min_y) > (min_y-y_range[0])
    if removed_is_top:
        first_is_top = True
        new_y_expect = min_y + top_bottom_space
    else:
        first_is_top = False
        new_y_expect = min_y - top_bottom_space
    assert len(sampled_y) == len(all_dists)
    idx_in_sample = bisect(sampled_y, new_y_expect)
    if idx_in_sample==len(sampled_y):
        idx_in_sample -= 1
    local_min_idx, _ = local_min(all_dists, idx_in_sample)
    local_min_loc = sampled_y[local_min_idx]
    if abs(local_min_loc-new_y_expect)<=two_wall_space_tolerance:
        second_bt_remove_loc = local_min_loc
        remove_idxs = remove_plane_idx(xyz, local_min_loc, 'xz')
        combo = select_each(combo, remove_idxs)
        xyzrgb, xyz, xy, yz, xz, x, y, z = combo
        local_min_works = True
        print "Local min: another top/bottom removed at", local_min_loc
    else:
        local_min_works = False
        print "Local min: another top/bottom not in expected location, expect:", new_y_expect, "actual:", local_min_loc
        

# old other top/bottom removal code
if (top_bottom_space is not None) and first_bt_detected and (not local_min_works):
    y_range = [sampled_y[0], sampled_y[-1]]
    assert y_range[0]<y_range[1]
    removed_is_top = (y_range[1]-min_y) > (min_y-y_range[0])
    if removed_is_top:
        new_y_expect = min_y + top_bottom_space
    else:
        new_y_expect = min_y - top_bottom_space
    print "New search: expect the other top/bottom to be at "+str(new_y_expect)+"..."
    ret = sw_bt.remove_bottom_top(xyz, xz, x, y, z, new_y_expect)
    if ret is None:
        print "New search: another top/bottom not in expected location"
    else:
        min_y, min_dist, _, _ = ret
        if min_dist < plane_removal_threshold:
            second_bt_remove_loc = min_y
            remove_idxs = remove_plane_idx(xyz, min_y, 'xz')
            combo = select_each(combo, remove_idxs)
            xyzrgb, xyz, xy, yz, xz, x, y, z = combo
            print "New search: another top/bottom removed at", min_y
        else:
            print "New search: no top/bottom detected"

print "removing all points that are not between top and bottom (if top or bottom is identified)"
if first_bt_remove_loc is not None:
    if first_is_top:
        if second_bt_remove_loc is None:
            second_bt_remove_loc = float('inf')
    else:
        if second_bt_remove_loc is None:
            second_bt_remove_loc = -float('inf')
    if first_is_top:
        top = first_bt_remove_loc
        bottom = second_bt_remove_loc
    else:
        top = second_bt_remove_loc
        bottom = first_bt_remove_loc
    print "keeping only points between", top, "and", bottom
    _, in_between_idxs = filter_val_idx(lambda y: top<y<bottom, y)
    combo = select_each(combo, in_between_idxs)
    xyzrgb, xyz, xy, yz, xz, x, y, z = combo

print ""

if findcc:
    cc = xyzrgb
else:
    print "Finding largest connected component using", len(xyzrgb), "points..."
    base_threshold = 0.005
    if len(xyzrgb)>10000:
        sampled = random.sample(xyzrgb, 10000)
        threshold = base_threshold * len(xyzrgb)/10000
    else:
        sampled = xyzrgb
        threshold = base_threshold
    if (-100 < first_bt_remove_loc < 100) and (-100 < second_bt_remove_loc < 100):
        # only counts points that are in lower half of the space between top and bottom
        cc = get_largest_cc(sampled, neighbor_max_dist=base_threshold, eligible_condition=lambda x:x[1]>(top+bottom)/2)
    else:
        cc = get_largest_cc(sampled, neighbor_max_dist=base_threshold)

print "Done"
print ""

if len(args)==3:
    write_pcd_file(cc, args[2])
else:
    render_3d_scatter(xyzrgb, proportion=0.2).set_title("all removed")
    render_3d_scatter(cc, proportion=0.3).set_title("largest connected component")
    plt.show()
    
