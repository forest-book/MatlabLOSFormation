import numpy as np

Quad_num = 5
k01 = np.zeros((Quad_num-1, 2))
k01[0, :] = [5, 200]
k01[1, :] = [5, 200]
k01[2, :] = [5, 200]
k01[3, :] = [5, 200]

print(k01)
