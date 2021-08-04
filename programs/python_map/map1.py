import numpy as np


f = open('map.txt', 'r') 

datalist = f.readlines()
raw = len(datalist)
col = 0
for line in datalist:
    col = max(col, len(line))

original = np.zeros((raw, col), dtype=np.int32)

for i in range(raw):
    for j in range(col):
        if j < len(datalist[i]):
            if(datalist[i][j] == "#"):
                original[i,j] = 1
            else:
                original[i,j] = -1
        else:
                original[i,j] = -1
print(original)

for i in range(raw):
    for j in range(col):
        if original[i,j] != -1:
            continue
        values = []
        for ii in [-1, 0 ,1]:
            for jj in [-1, 0, 1]:
                if 0<=i+ii<raw and 0<=j+jj<col and original[i+ii,j+jj]!=-1:
                    values.append(original[i+ii,j+jj])
            if 3<=len(values):
                original[i,j]=min(values)+1


print(original)