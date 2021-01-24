import subprocess
import multiprocessing
import sys

#thread_num
#use_gpu
#plate_max_width
#plate_min_width
num_threads = multiprocessing.cpu_count()
print("Start testing.\n")
print("System has " + num_threads + " threads.\n");

def main(videoName):
	data = dict()
	for i in range(1, num_threads):
		data["thread_num"] = i
		setupDataFile = "setupData.txt"
		outputDataFile = "outputFile.txt"
		with open(setupDataFile, 'w') as file_object:
			whole_str = "thread_num|" + i + "\n"
			file_object.write(whole_str)
		programma = "./minimal_lpr " + videoName +" " + setupDataFile +" " + outputDataFile
		process = subprocess.run(programma)
		if process.returncode == 1:
			print("something wrong...")
			return 1;
		fps = 0.0;
		with open(outputDataFile) as file_object:
			lines = file_object.readlines();
			for line in lines:
				dt = line.rstrip().split("|")
				print(dt[0]+ " = "  + dt[1])
				fps = dt[1]
		with open("result.csv", "a") as file_object:
			whole_str = i + ";" + fps + "\n"
			file_object.write(whole_str)

			


if __name__ == '__main__':
	if (len(sys.argv) < 2):
		print("Need define video file.")
	else:
		main(sys.argv[1])


print("Finish testing.\n")