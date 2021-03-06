import subprocess
import multiprocessing
import sys
import os
from random import randint
from random import choice
import json
#thread_num 1..N
#disable_gpu [0,1]
#plate_max_width [min..preset_max]
#plate_min_width [preset_min ..max]
#region   TODO
#screen resolution

num_threads = 8#multiprocessing.cpu_count()
print("Start testing.\n")
print("System has " + str(num_threads) + " threads.\n")

#----------------------------------------------------
def generateThread():
	return randint(1, num_threads)

#----------------------------------------------------
def generateDisableGpu():
  return (choice([0, 1]))

#----------------------------------------------------
def generateMinPlate():
  return randint(20, 80)

#----------------------------------------------------
def generateMaxPlate():
  return randint(100, 250)


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
			outData[dt[0]] = dt[1]
	return outData


#----------------------------------------------------
def writeToFinalStatistic(file_name, input_data, result_data):
	emptyFile = os.stat(file_name).st_size == 0
	with open(file_name, "a") as file_object:
		if emptyFile == True:
			header = ""
			for key in input_data:
				header += str(key) + ";" 
			for key in result_data:
				header += str(key) + ";"
			header = header.rstrip(";")
			header += "\n"
			file_object.write(header)
		whole_str = ""
		for key in input_data:
			val = input_data[key]
			if (type(val) == bool):
				 val = int(val)
			whole_str += str(val) + ";"
		for key in result_data:
			whole_str  += str(result_data[key]) + ";"
		whole_str = whole_str.rstrip(";")
		whole_str += "\n"
		file_object.write(whole_str)

#----------------------------------------------------
def showProgress(current, maxValue):
	current_progress = int(100.0*float(current)/float(maxValue))
	progress = "["
	for prInd in range(1,51):
		if prInd < current_progress/2:
			progress += "-"
		else:
			progress += " "
	progress+= "]"
	progress+= " " + str(current_progress) + "%\r"
	print(progress, end="", flush=True)

#----------------------------------------------------
def writeToLogs(errors, messages):
	with open("errors.log", "a") as log_file:
		log_file.write(errors)
	with open("messages.log", "a") as log_file:
		log_file.write(messages)

#----------------------------------------------------
def main(execName, videoName, test_num, result_statistic_data, correct_plates):
	data = dict()
	setupDataFile = "setupData.txt"
	outputDataFile = "outputFile.txt"
	with open(result_statistic_data, "w") as file_object:
		file_object.truncate(0)
	for i in range(1, test_num+1):
		data = generateInputData()
		writeSetupData(data, setupDataFile)
		resolution = choice([25, 50, 75, 100])
		process = subprocess.run([execName, videoName, str(resolution), setupDataFile, outputDataFile, correct_plates], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)#capture_output=True, text=True)
		#writeToLogs(process.stderr, process.stdout)
		if process.returncode == 1:
			print("something wrong...\n")
			return 1
		result = readOutputData(outputDataFile)
		data["resolution"] = resolution
		writeToFinalStatistic(result_statistic_data, data, result)
		showProgress(i, test_num)
	print("\n")

			

#----------------------------------------------------
if __name__ == '__main__':
	conf_file = "conf.json"
	if (len(sys.argv)==3):
		conf_file = sys.argv[2]
	with open(conf_file) as file_object:
		data = json.load(file_object)
		main(data["app"], data["video"], data["test_num"], data["output_statistic_file"], data["correct_plates"])
	
print("Finish testing.\n")
