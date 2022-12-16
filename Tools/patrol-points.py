import numpy as np
from matplotlib import pyplot as plt

x = np.array([300, 750, 1090, 1090, 750, 300, 1090, 750, 300, 300, 750, 1090, 1390, 1866, 1866, 1390])
y = np.array([500, 290, 655, 845, 1210, 1000, 845, 1210, 1000, 500, 290, 655, 655, 505, 995, 845])

fig, ax = plt.subplots()
ax.scatter(x, y)

for i in range(0, 16): #16 coordinates
    print(i, ": (",x[i] ,",",y[i],")")
    ax.annotate(i, (x[i], y[i]))

plt.show()
