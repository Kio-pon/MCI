import matplotlib.pyplot as plt
import numpy as np

# Set up the figure (16:9 aspect ratio)
plt.figure(figsize=(10, 5.625), facecolor='white')

# 1. Generate "Earthing Studies" Data (The Noise)
np.random.seed(42)
x_noise = 2 + 1.2 * np.random.randn(60)
y_noise = 2 + 1.2 * np.random.randn(60)
plt.scatter(x_noise, y_noise, s=100, c='#D34A4A', edgecolors='#333333', linewidths=1, zorder=3)

# 2. Generate "Scientific Standards" Data (The Signal)
x_signal = np.linspace(7, 11, 12)
y_signal = x_signal * 1.1 - 1 + 0.3 * np.random.randn(12)
plt.scatter(x_signal, y_signal, s=120, c='#2E8B57', edgecolors='#333333', linewidths=1.5, zorder=3)

# 3. Axes and Labels
plt.xlim(0, 12)
plt.ylim(0, 12)
plt.xlabel('Methodological Rigor (Controls, Sample Size)', fontsize=12, fontweight='bold')
plt.ylabel('Reproducibility', fontsize=12, fontweight='bold')
plt.title('Clinical Evidence: Signal vs. Noise', fontsize=16, fontweight='bold')

# 4. Callout Annotations
plt.text(3.8, 1.5, '← Small sample sizes (n < 50)', color='#B22222', fontsize=10, fontweight='bold')
plt.text(3.5, 3.5, '← Lack tight controls', color='#B22222', fontsize=10, fontweight='bold')
plt.text(10.8, 10.5, 'Controlled, falsifiable experiments →', color='#006400', fontsize=10, fontweight='bold', ha='right')

# 5. Clean up grid for a technical aesthetic
plt.grid(True, linestyle='--', alpha=0.5, zorder=0)
plt.gca().spines['top'].set_visible(False)
plt.gca().spines['right'].set_visible(False)

# Save and show
plt.tight_layout()
plt.show()
