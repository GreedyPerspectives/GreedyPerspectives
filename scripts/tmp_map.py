import numpy as np
from matplotlib import pyplot as plt
map = np.zeros((30,30), dtype=np.float64)


# map[0:15, 0:15] = 6
# map[14,8:15] = 3
# map[10:15,14] = 2.5
# map[17, 13:17] = 1.5
# map[18, 6:10] = 4
# map[8:10, 17] = 6

map[29, 29] = 7
# map[0:14, 10:20] = 6
# map[16:30, 10:20] = 6

# map[10:15,5] = 4.5
# map[2:6,8] = 4
# map[4:10,9] = 2
# map[9:12,15] = 2
# map[18:22,20] = 3
# map[21:30,24] = 3
# map[15:19,28] = 2.5

# map[10:13,10] = 4.5
# map[2:9,18] = 4
# map[14:20,14] = 2
# map[9:12,2] = 2
# map[18:22,3] = 3
# map[21:30,5] = 3
# map[15:19,7] = 2.5

# map[10:15,5] = 3.5
# map[6:10,20] = 4
# map[10:15,26] = 2
# map[2:10,28] = 1

# map[23:28,12] = 4


def add_square(matrix, size, value):
    x, y = np.random.randint(0, matrix.shape[0]-size), np.random.randint(0, matrix.shape[1]-size)
    matrix[x:x+size, y:y+size] = value

min_value = 5
max_value = 10



for _ in range(60):
    square_size = np.random.randint(1, 3)  # Random size between 2 and 8
    square_value = np.random.uniform(min_value, max_value)  # Random value between 5 and 10
    add_square(map, square_size, square_value)

np.save("../res/experiments/forest/map.npy", map)
plt.imshow(map, interpolation='nearest')
plt.show()