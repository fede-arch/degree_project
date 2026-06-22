# tesi/plot_multi_scalabilita.py
import matplotlib.pyplot as plt
import numpy as np

n_droni   = [10, 20, 30, 40]
arrived   = [100, 100, 85, 70]
collision = [0,   0,  10, 20]
timeout   = [0,   0,   5, 10]

x   = np.arange(len(n_droni))
w   = 0.25

fig, ax = plt.subplots(figsize=(8, 4))

ax.bar(x - w, arrived,   w, label="Arrived",   color="#1a9641")
ax.bar(x,     collision, w, label="Collision",  color="#d7191c")
ax.bar(x + w, timeout,   w, label="Timeout",    color="#fdae61")

ax.set_xlabel("Numero di droni", fontsize=12)
ax.set_ylabel("Tasso (%)", fontsize=12)
ax.set_title("Scalabilità del sistema multi-AUV", fontsize=13)
ax.set_xticks(x)
ax.set_xticklabels([str(n) for n in n_droni])
ax.set_ylim(0, 115)
ax.legend(fontsize=10)
ax.grid(True, axis="y", alpha=0.3)
fig.tight_layout()
fig.savefig("immagini/multi_scalabilita.png", dpi=300)
print("Salvato: immagini/multi_scalabilita.png")