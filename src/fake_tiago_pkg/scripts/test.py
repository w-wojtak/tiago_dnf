import matplotlib
matplotlib.use('TkAgg')  # or try 'Qt5Agg' if TkAgg fails

import matplotlib.pyplot as plt
import numpy as np

print("Matplotlib backend:", matplotlib.get_backend())

x = np.linspace(-10, 10, 100)
y = np.sin(x)

plt.plot(x, y)
plt.title("Test Plot")
plt.grid(True)
plt.show()