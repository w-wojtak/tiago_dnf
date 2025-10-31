#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
import sys

trial = int(sys.argv[1]) if len(sys.argv) > 1 else 1
x_lim, dx = 80, 0.2
x = np.arange(-x_lim, x_lim + dx, dx)

filepath = f'/workspaces/fake_tiago_ws/src/fake_tiago_pkg/data_basic/trial_{trial}/h_u_amem.npy'
h_u_amem = np.load(filepath)

plt.figure(figsize=(12, 4))
plt.plot(x, h_u_amem, linewidth=2, label=f'Trial {trial}')
plt.axhline(y=0, color='k', linestyle='--', alpha=0.3)
plt.xlabel('Position (x)')
plt.ylabel('h_u_amem')
plt.title(f'Adaptive Memory - Trial {trial}')
plt.legend()
plt.grid(True, alpha=0.3)
plt.tight_layout()
plt.show()

print(f"Trial {trial} - Max: {np.max(h_u_amem):.4f}, Min: {np.min(h_u_amem):.4f}")