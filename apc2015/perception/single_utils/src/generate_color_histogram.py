
from __future__ import division
from common_utils import *
from subprocess import call
import numpy as np
import colorsys
import matplotlib.pyplot as plt

all_items = ["champion_copper_plus_spark_plug",
"cheezit_big_original",
"crayola_64_ct",
"dove_beauty_bar",
"dr_browns_bottle_brush",
"elmers_washable_no_run_school_glue",
"expo_dry_erase_board_eraser",
"feline_greenies_dental_treats",
"first_years_take_and_toss_straw_cups",
"genuine_joe_plastic_stir_sticks",
"highland_6539_self_stick_notes",
"kong_air_dog_squeakair_tennis_ball",
"kong_duck_dog_toy",
"kong_sitting_frog_dog_toy",
"kygen_squeakin_eggs_plush_puppies",
"laugh_out_loud_joke_book",
"mark_twain_huckleberry_finn",
"mead_index_cards",
"mommys_helper_outlet_plugs",
"munchkin_white_hot_duck_bath_toy",
"one_with_nature_soap_dead_sea_mud",
"oreo_mega_stuf",
"paper_mate_12_count_mirado_black_warrior",
"rollodex_mesh_collection_jumbo_pencil_cup",
"safety_works_safety_glasses",
"sharpie_accent_tank_style_highlighters",
"stanley_66_052"]

save_dir = '/home/motion/ece590-s2015/perception/hue_array/'

for item in all_items:
	print item
	ply_file = '/home/motion/ece590-s2015/klampt_models/items/' + item + '/textured_meshes/optimized_tsdf_textured_mesh.ply'
	call(['/home/motion/ece590-s2015/perception/single_utils/bin/PLYtoPCD', ply_file, 'temp.pcd'])
	xyzrgb = read_pcd_file("temp.pcd", ["xyzrgb"])[0]
	hue_array = []
	for _,_,_,rgb in xyzrgb:
		r,g,b = pcl_float_to_rgb(rgb)
		r /= 255
		g /= 255
		b /= 255
		h,_,_ = colorsys.rgb_to_hsv(r,g,b)
		hue_array.append(h*180)
	hue_array = np.array(hue_array)
	np.savez_compressed(save_dir+item, hue_array)
	#plt.hist(hue_array, bins=18)
	#plt.show()

