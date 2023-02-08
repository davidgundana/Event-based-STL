import numpy as np

if __name__ == "__main__":
    with open('disjunction.txt') as f:
        data = f.readlines()

    for i in range(np.size(data)):
        data[i] = eval(data[i])

    time2compute = 0
    for i in range(np.size(data,0)-1):
        time2compute += data[i+1][0] - data[i][0]

    averageTime = time2compute/np.size(data,0)
    print(averageTime)

    for i in range(np.size(data, 0) - 1):
        if data[i+1][43] != data[i][44]:
            print('here')
        if data[i][42] != 0:
            print('here')
    print('here')
