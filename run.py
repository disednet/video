import subprocess
import multiprocessing
import sys
import os
from random import randint
from random import choice
#thread_num 1..N
#disable_gpu [0,1]
#plate_max_width [min..preset_max]
#plate_min_width [preset_min ..max]
#region   TODO
#screen resolution

num_threads = multiprocessing.cpu_count()
print("Start testing.\n")
print("System has " + str(num_threads) + " threads.\n")

#----------------------------------------------------
def generateThread():
	return randint(1, num_threads)

#----------------------------------------------------
def generateDisableGpu():
	return bool(randint(0, 1))

#----------------------------------------------------
def generateMinPlate():
	return randint(20, 40)

#----------------------------------------------------
def generateMaxPlate():
	return randint(40, 200)


#----------------------------------------------------
def generateInputData():
	result = dict()
	result["thread_num"] = generateThread()
	result["disable_gpu"] = generateDisableGpu()
	result["plate_min_width"] = generateMinPlate()
	result["plate_max_width"] = generateMaxPlate()
	return result

#----------------------------------------------------
def writeSetupData(data, file_name):
	with open(file_name, 'w') as file_object:
		for key, value in data.items():
			whole_str = str(key) + "|" + str(value) + "\n"
			file_object.write(whole_str)

#----------------------------------------------------
def readOutputData(file_name):
	outData = dict()
	with open(file_name) as file_object:
		lines = file_object.readlines()
		for line in lines:
			dt = line.rstrip().split("|")
			print(dt[0]+ " = "  + dt[1])
			outData[dt[0]] = dt[1]
	return outData


#----------------------------------------------------
def writeToFinalStatistic(file_name, input_data, result_data):
	emptyFile = os.stat(file_name).st_size == 0
	with open(file_name, "a") as file_object:
		if emptyFile == True:
			header = ""
			for key in sorted(input_data.keys()):
				header += str(key) + ";" 
			for key in sorted(result_data.keys()):
				header += str(key) + ";"
			header = header.rstrip(";")
			header += "\n"
			file_object.write(header)
		whole_str = ""
		for key in sorted(input_data.keys()):
			val = input_data[key]
			if (type(val) == bool):
				 val = int(val)
			whole_str += str(val) + ";"
		for key in sorted(result_data.keys()):
			whole_str  += str(result_data[key]) + ";"
		whole_str = whole_str.rstrip(";")
		whole_str += "\n"
		file_object.write(whole_str)

#----------------------------------------------------
def main(execName, videoName, test_num, result_statistic_data):
	data = dict()
	setupDataFile = "setupData.txt"
	outputDataFile = "outputFile.txt"
	with open(result_statistic_data, "w") as file_object:
		file_object.truncate(0)
	for i in range(0, test_num):
		data = generateInputData()
		writeSetupData(data, setupDataFile)
		resolution = choice([25, 50, 75, 100])
		process = subprocess.run([execName, videoName, str(resolution), setupDataFile, outputDataFile])
		if process.returncode == 1:
			print("something wrong...")
			return 1
		result = readOutputData(outputDataFile)
		data["resolution"] = resolution
		writeToFinalStatistic(result_statistic_data, data, result)
			

#----------------------------------------------------
if __name__ == '__main__':
	if (len(sys.argv) < 3):
		print("Need define video file.")
	else:
		result_file = "result.csv"
		tests_num = 12
		if len(sys.argv) >= 4:
			tests_num = int(sys.argv[3])
		if len(sys.argv) == 5:
			result_file = sys.argv[4]
		main(sys.argv[1], sys.argv[2], tests_num,  result_file)
	


print("Finish testing.\n")
