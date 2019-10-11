import numpy as np 
import h5py
import matplotlib.pyplot as plt
from matplotlib import interactive

def main():
	f = h5py.File("/home/leus/force_test_bag/desired_trajectory.hdf5","r")
	avg_sequence_touch = f["touching/avg_position"][0::]
	max_sequence_touch = f["touching/max_position"][0::]
	min_sequence_touch = f["touching/min_position"][0::]
	avg_sequence_no_touch = f["not_touching/avg_position"][0::]
	max_sequence_no_touch = f["not_touching/max_position"][0::]
	min_sequence_no_touch = f["not_touching/min_position"][0::]
	f.close()

	fig, axs = plt.subplots(2, 1)
	axs[0].plot(range(len(avg_sequence_touch)),avg_sequence_touch,"firebrick")
	axs[0].plot(range(len(max_sequence_touch)),max_sequence_touch,"darksalmon")
	axs[0].plot(range(len(min_sequence_touch)),min_sequence_touch,"darksalmon")

	axs[0].plot(range(len(avg_sequence_no_touch)),avg_sequence_no_touch,"c")
	axs[0].plot(range(len(max_sequence_no_touch)),max_sequence_no_touch,"powderblue")
	axs[0].plot(range(len(min_sequence_no_touch)),min_sequence_no_touch,"powderblue")

	fig.suptitle("average max and min error of position of r_arm_controller_state")
	interactive(True)
	plt.show()



	#act_sequence = max_sequence_touch
	act_sequence = min_sequence_no_touch

	diff_touch_array = []
	diff_no_touch_array = []
	reaction = []
	window = 50
	length = min(len(avg_sequence_touch),len(avg_sequence_no_touch))

	touch_threshold = 0.02 * window / 50
	no_touch_threshold  = 0.01 * window /50

	for i in range(length-window):
		diff_touch = sum(abs(avg_sequence_touch[i:i+window] - act_sequence[i:i+window])) #if I use abs I do not have direction (not informative anyways) but count both positive
		diff_no_touch = sum(abs(avg_sequence_no_touch[i:i+window] - act_sequence[i:i+window]))
		diff_touch_array.append(diff_touch)
		diff_no_touch_array.append(diff_no_touch)

		if diff_touch > touch_threshold and diff_no_touch <= no_touch_threshold:
			#robot is not touching the bowl
			move_toward_spatula(diff_touch,diff_no_touch)
			reaction.append(1)
		elif diff_touch > touch_threshold and diff_no_touch > no_touch_threshold:
			#robot is touching the bowl too much
			move_away_from_spatula(diff_touch,diff_no_touch)
			reaction.append(0)
		else:
			print "everything alright it is touching!"
			reaction.append(-1)

	#print diff_touch_array
	#print diff_no_touch_array

	fig, axs = plt.subplots(2, 1)
	axs[0].plot(range(len(diff_touch_array)),diff_touch_array,"firebrick")
	axs[0].plot(range(len(diff_no_touch_array)),diff_no_touch_array,"c")

	axs[1].plot(range(len(reaction)),reaction)
	fig.suptitle("difference to touching in red, difference to not touchign in blue, real touching")
	interactive(False)
	plt.show()


def move_toward_spatula(diff_touch,diff_no_touch):
	print "moving toward spatula, score is touch is %f, score no touch is %f" % (diff_touch,diff_no_touch)

def move_away_from_spatula(diff_touch,diff_no_touch):
	print "moving away from spatula, score is %f, score no touch is %f" % (diff_touch,diff_no_touch)

if __name__ == "__main__":
    main()