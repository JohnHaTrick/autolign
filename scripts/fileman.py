__author__      = "icoen"

import numpy as np
import csv
import datetime
import time

#rows=[]	#List of: Dictionaries containing North, East...
headers = ['E', 'N', 'psi', 'Ux', 'Uy', 'r']	#List of Dictionaries keys
hehe=[]


#Creates File
#Args: (),"File","File+" --> Timestamp.CSV, File.CSV, File-Timestamp.CSV 
def initializer(filename = datetime.datetime.fromtimestamp(time.time()).strftime('%Y-%m-%d-%H-%M-%S'), **sets): 
	if filename[-1:]=='+':
	   filename = filename[:-1]+'-'+datetime.datetime.fromtimestamp(time.time()).strftime('%Y-%m-%d-%H-%M-%S')
	filename +='.csv'
	with open(filename, 'a') as csvfile:
         writer = csv.writer(csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
         writer.writerow(sets.get("order"))
	return filename

# def initializer2():      #creates filename based on exact date of execution // Ignore (Testing)
# 		filename = time.time()
# 		filename = datetime.datetime.fromtimestamp(filename).strftime('%Y-%m-%d-%H-%M-%S')
# 		rows=[]
# 		with open(filename, 'a') as data:
# 			return filename, data

def write_row_csv(file, state, **sets):  #writes a CSV file. Params: filename, array/list/tuple... of values
	with open(file, 'a') as csvfile:
		filewriter = csv.DictWriter(csvfile, fieldnames=sets.get("order"), delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
		filewriter.writerow(state)
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



