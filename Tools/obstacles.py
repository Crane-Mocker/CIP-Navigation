import numpy as np
from matplotlib import pyplot as plt

x = np.array([950, 1010, 920, 1100, 1200, 1250, 1050, 750])
y = np.array([750, 800, 905, 1105, 1000, 1050, 1250, 950])

fig, ax = plt.subplots()
ax.scatter(x, y)

for i in range(0, 8): #8 coordinates
    print(i, ": (",x[i] ,",",y[i],")")
    ax.annotate(i, (x[i], y[i]))

plt.show()
