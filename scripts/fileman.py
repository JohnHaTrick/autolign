__author__      = "icoen"

import numpy as np
import csv
import datetime
import time

rows=[]	#List of: Dictionaries containing North, East...
#fieldnames = ['E', 'N', 'psi', 'Ux', 'Uy', 'r']	#List of Dictionaries keys
fieldnames = ["North", "East", "West", "South"]	#List of Dictionaries keys


def initializer():      #creates filename based on exact date of execution
	filename = time.time()
	filename = datetime.datetime.fromtimestamp(filename).strftime('%Y-%m-%d-%H-%M-%S')+'.csv'
	rows=[]
	return filename

# def initializer2():      #creates filename based on exact date of execution // Ignore (Testing)
# 		filename = time.time()
# 		filename = datetime.datetime.fromtimestamp(filename).strftime('%Y-%m-%d-%H-%M-%S')
# 		rows=[]
# 		with open(filename, 'a') as data:
# 			return filename, data

def write_row_csv(file, paket):  #writes a CSV file. Params: filename, array/list/tuple... of values
	with open(file, 'a') as data:
		filewriter = csv.DictWriter(data, fieldnames=paket.keys(), delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
		filewriter.writerow(paket)
		#print(paket.keys())

# def write_row_csv2(a, data):  #writes a CSV file. Params: filename, array/list/tuple of values //Ignore (testing)
# 	filewriter = csv.writer(data, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
# 	filewriter.writerow(a)

# def append_rows_csv(paket):    #builds a list for future CSV writing. Params: array/list/tuple... of values
# 	rows.append(paket)

# def write_full_csv(file):  #writes a CSV file. Params: filename, array/list/tuple of values
# 	with open(file, 'a') as data:
# 		filewriter = csv.DictWriter(data, fieldnames=fieldnames, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
# 		for paket in rows:
# 			filewriter.writerow(paket)


def read_csv(file):      #reads a CSV file. Params: filename, array/list/tuple of values
	with open(file, 'r') as data:
		reader = csv.reader(data)
		for row in reader:
			if any(row):
				rowArr=np.asarray(row)
				print(rowArr)



