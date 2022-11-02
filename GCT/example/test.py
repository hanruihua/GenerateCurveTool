import numpy as np

a = np.array([[1], [2]])
b = 3
c = np.vstack((a, [b]))

print(c)