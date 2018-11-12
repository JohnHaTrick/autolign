import fileman as fm
import numpy as np
import time

keys = ["North", "East", "West", "South"]
values = np.random.rand(4)
paket=dict(zip(keys, values))

print('Writing one at a time...')
filename=fm.initializer() #Add this in RUN, to initialize module
start_time = time.time()  #Computing time start //Just for testing
for i in range(500):
	values = np.random.rand(4)
	paket=dict(zip(keys, values))
	print(paket)
	fm.write_row_csv(filename,paket) #Add this to RUN, to write in RT
print("--- %s seconds ---" % (time.time() - start_time))   #Computing time end //Just for testing

print('Writing all at the end...')
time.sleep(5) #Pause to see time difference
start_time = time.time()  #Computing time start //Just for testing
for i in range(500):
	print(paket)
	fm.append_rows_csv(paket)
fm.write_full_csv(fm.initializer()) #Add this in RUN, to initialize module AND write in the end
print("--- %s seconds ---" % (time.time() - start_time))   #Computing time end //Just for testing

# print('Writing one at a time...')
# filename,data=fm.initializer2()
# filename+='.csv'
# start_time = time.time()
# for i in range(100):
# 	a=np.random.rand(10)
# 	fm.write_row_csv2(a, data)
# 	#fm.append_rows_csv(a)
# 	#print(a)
# #fm.write_full_csv(fm.initializer()+'.csv')
# print("--- %s seconds ---" % (time.time() - start_time))
# #print('Reading...')
# #fm.read_csv(filename)

###############################################

