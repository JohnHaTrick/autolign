import sys
import time
import numpy as np
import pandas as pd

alpha = 0.5         # learning rate
gamma = 0.95        # discount factor


# read in CSV file
def readCSV(inputfilename):
    rawData = pd.read_csv(inputfilename)
    data = rawData.values
    df = pd.DataFrame(data)     # genreate a_t dataframe
    return df


# Q Learning Algorithm
def qLearning(df):
    # Return all unique states in the dataframe
    states = np.unique(df[0])
    # Construct a_t dictionary of Q values from these states
    Q = dict.fromkeys(states)
    # Note: this method of initialization is crucial for the correctness of Q
    for i in states:
        Q[i] = [0, 0, 0]
    # Initialze Q value with reward in a_t way that only higher reward updates
    # existing values
    for i in range(len(df[0])):
        s_t = df[0][i].item()
        a_t = df[1][i].item()
        R = df[2][i].item()
        if Q[s_t][a_t-1] < R:
            Q[s_t][a_t-1] = R
    # Run the Q learning updates
    for k in range(2):
        for i in range(len(df[0])):
            s_t = df[0][i].item()
            a_t = df[1][i].item()
            R = df[2][i].item()
            sPrime = df[3][i].item()
            # Algorithm 5.3 in DMU
            Q[s_t][a_t-1] = Q[s_t][a_t-1] + alpha * (R + gamma * max(Q[sPrime]) - Q[s_t][a_t-1])
    return Q


# Policy generation algorithm
def generatePolicy(Q, df):
    states = np.unique(df[0])
    pi = [0]*len(states)
    # for all the states:
    for i in range(len(states)):
        if (i+1) in states:     # if this state is calculated
            pi[i] = Q[i + 1].index(max(Q[i + 1])) + 1   # add actual action
        else:
            pi[i] = 4     # else: add action 4 to maximize reward
    return pi


# Write out policy to file
def writePolicy(pi, outputfilename):
    with open(outputfilename, 'w') as f:
        for i in pi:
            f.write("{}\n".format(i))


# Main compute function that callls the other functions
def compute(inputfilename, outputfilename):
    df = readCSV(inputfilename)
    Q = qLearning(df)
    pi = generatePolicy(Q, df)
    writePolicy(pi, outputfilename)


# Main function that checks input, callls the compute function and calculate
# run time
def main():
    if len(sys.argv) != 3:
        raise Exception("usage: python large.py <infile>.csv <outfile>.policy")
    inputfilename = sys.argv[1]
    outputfilename = sys.argv[2]
    startTime = time.perf_counter()
    compute(inputfilename, outputfilename)
    endTime = time.perf_counter()
    print("This program took", endTime-startTime, "seconds to run")


if __name__ == '__main__':
    main()
