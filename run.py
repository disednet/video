import subprocess
import multiprocessing
import sys
from random import randint
#thread_num
#use_gpu
#plate_max_width
#plate_min_width
num_threads = 8#multiprocessing.cpu_count()
print("Start testing.\n")
print("System has " + str(num_threads) + " threads.\n")

def main(execName, videoName):
	data = dict()
	for i in range(1, num_threads+1):
		data["thread_num"] = i#randint(1, num_threads)
		setupDataFile = "setupData.txt"
		outputDataFile = "outputFile.txt"
		with open(setupDataFile, 'w') as file_object:
			whole_str = "thread_num|" + str(data["thread_num"]) + "\n"
			file_object.write(whole_str)
		process = subprocess.run([execName, videoName, setupDataFile, outputDataFile])
		if process.returncode == 1:
			print("something wrong...")
			return 1
		outData = dict()
		with open(outputDataFile) as file_object:
			lines = file_object.readlines()
			for line in lines:
				dt = line.rstrip().split("|")
				print(dt[0]+ " = "  + dt[1])
				outData[dt[0]] = dt[1]
		with open("result.csv", "a") as file_object:
			whole_str = str(data["thread_num"])# + ";" + str(fps) + "\n"
			for key, value in outData.items():
				whole_str  = whole_str + ";" + str(value)
			whole_str += "\n"
			file_object.write(whole_str)

			


if __name__ == '__main__':
	if (len(sys.argv) < 2):
		print("Need define video file.")
	else:
		main(sys.argv[1], sys.argv[2])
	


print("Finish testing.\n")
