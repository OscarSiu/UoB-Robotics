import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from sklearn.linear_model import LinearRegression

# Sample Data (Replace these with your actual x, y values)

# Load the Excel file
file_path = "Plot\snr_dist.xlsx"
xls = pd.ExcelFile(file_path)

# Load the data from both sheets
df_rain = xls.parse('Rain')
df_fog = xls.parse('Fog')

# Clean and reformat the data for Rain
df_rain_clean = df_rain.iloc[1:].reset_index(drop=True)
df_rain_clean.columns = ['x'] + list(df_rain.columns[1:])
df_rain_clean = df_rain_clean.drop(columns=['x']).astype(float)

# Clean and reformat the data for Fog
df_fog_clean = df_fog.iloc[1:].reset_index(drop=True)
df_fog_clean.columns = ['x'] + list(df_fog.columns[1:])
df_fog_clean = df_fog_clean.drop(columns=['x']).astype(float)

# Prepare distance values
distances = np.array(df_rain_clean.columns.astype(float)).reshape(-1, 1)

# Prepare data for all noise levels in Rain condition
rain_all_distances = np.tile(distances, (df_rain_clean.shape[0], 1)).flatten()
rain_all_noise = df_rain_clean.values.flatten()

# Perform linear regression on all Rain data points
model_rain_all = LinearRegression()
model_rain_all.fit(rain_all_distances.reshape(-1, 1), rain_all_noise)

# Prepare data for all noise levels in Fog condition
fog_all_distances = np.tile(distances, (df_fog_clean.shape[0], 1)).flatten()
fog_all_noise = df_fog_clean.values.flatten()

# Perform linear regression on all Fog data points
model_fog_all = LinearRegression()
model_fog_all.fit(fog_all_distances.reshape(-1, 1), fog_all_noise)

# Plotting
plt.figure(figsize=(10, 5))

# Plot all Rain Data with connected lines
plt.subplot(1, 2, 1)
for i in range(df_rain_clean.shape[0]):
    plt.plot(distances, df_rain_clean.iloc[i].values, marker='o', linestyle='-', alpha=0.5)
plt.plot(distances, model_rain_all.predict(distances), label=f"Linear Fit: y={model_rain_all.coef_[0]:.3f}x + {model_rain_all.intercept_:.3f}", color='red', linewidth=2)
plt.xlabel("Distance (m)")
plt.ylabel("SNR (dB)")
plt.title("Rain Condition")
plt.legend()

# Plot all Fog Data with connected lines
plt.subplot(1, 2, 2)
for i in range(df_fog_clean.shape[0]):
    plt.plot(distances, df_fog_clean.iloc[i].values, marker='o', linestyle='-', alpha=0.5)
plt.plot(distances, model_fog_all.predict(distances), label=f"Linear Fit: y={model_fog_all.coef_[0]:.3f}x + {model_fog_all.intercept_:.3f}", color='red', linewidth=2)
plt.xlabel("Distance (m)")
plt.ylabel("SNR (dB)")
plt.title("Fog Condition")
plt.legend()

plt.tight_layout()
plt.show()

# Print regression coefficients
print("Rain Condition:")
print(f"Linear Fit Equation: SNR (dB) = {model_rain_all.coef_[0]:.3f} * Distance (m) + {model_rain_all.intercept_:.3f}")
print("\nFog Condition:")
print(f"Linear Fit Equation: SNR (dB) = {model_fog_all.coef_[0]:.3f} * Distance (m) + {model_fog_all.intercept_:.3f}")






