import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

df = pd.read_csv("../es/results/es_results.csv")

fig, ax = plt.subplots(figsize=(8, 4))

ax.plot(df["gen"], df["mean_reward"],
        color="#2c7bb6", linewidth=1.5,
        marker="o", markersize=4, label="Mean reward")

# Media mobile su finestra 3
rolling = df["mean_reward"].rolling(window=3, center=True).mean()
ax.plot(df["gen"], rolling,
        color="#d7191c", linewidth=2.0,
        linestyle="--", label="Media mobile (k=3)")

ax.axhline(y=0, color="gray", linewidth=0.8, linestyle=":")
ax.set_xlabel("Generazione", fontsize=12)
ax.set_ylabel("Mean Reward", fontsize=12)
ax.set_title("Convergenza Evolution Strategies", fontsize=13)
ax.xaxis.set_major_locator(ticker.MultipleLocator(2))
ax.legend(fontsize=10)
ax.grid(True, alpha=0.3)
fig.tight_layout()
fig.savefig("immagini/es_convergenza.png", dpi=300)
print("Salvato: immagini/es_convergenza.png")