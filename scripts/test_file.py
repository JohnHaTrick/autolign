import fileman as fm
import numpy as np
import time

#keys = ["E", "N", "psi", "Ux", "Uy", "r"]
#values = np.random.rand(len(keys))
#paket=dict(zip(keys, values))
#paket={"E": 1,"N": 2, "psi": 3, "Ux": 4, "Uy": 5, "r": 6}


state = {'E'  : 1,
         'N'  : 2,
         'psi': 3,
         'Ux' : 4, 
         'Uy' : 5,
         'r'  : 6
        }


ordered = ['N', 'E', 'psi', 'Ux', 'Uy', 'r']
#ordered = state.keys()

its=50

print('Writing one at a time...')
filename=fm.initializer(order=ordered) #RUN 2 init module | Args: (),"File","File+" --> Timestamp.CSV, File.CSV, File-Timestamp.CSV 
start_time = time.time()  #Computing time start //Just for testing
for i in range(its):
    #values = np.random.rand(len(keys))  
    #paket=dict(zip(keys, values))
    print(state)
    fm.write_row_csv(filename,state,order=ordered) #Add this to RUN, to write in RT
print("--- %s seconds ---" % (time.time() - start_time))   #Computing time end //Just for testing

# print('Writing all at the end...')
# time.sleep(5) #Pause to see time difference
# start_time = time.time()  #Computing time start //Just for testing
# for i in range(its):
#   print(paket)
#   fm.append_rows_csv(paket)
# fm.write_full_csv(fm.initializer()) #Add this in RUN, to initialize module AND write in the end
# print("--- %s seconds ---" % (time.time() - start_time))   #Computing time end //Just for testing

# print('Writing one at a time...')
# filename,data=fm.initializer2()
# filename+='.csv'
# start_time = time.time()
# for i in range(100):
#   a=np.random.rand(10)
#   fm.write_row_csv2(a, data)
#   #fm.append_rows_csv(a)
#   #print(a)
# #fm.write_full_csv(fm.initializer()+'.csv')
# print("--- %s seconds ---" % (time.time() - start_time))
print('Reading...')
fm.read_csv(filename)

###############################################

