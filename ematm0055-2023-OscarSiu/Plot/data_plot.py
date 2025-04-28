import pandas as pd
import matplotlib.pyplot as plt

# Load the Excel file
file_path = "Plot\Sim_data.xlsx"  # Update with the correct path if needed
xls = pd.ExcelFile(file_path)

# Load the Rain and Fog sheets
rain_df = pd.read_excel(xls, sheet_name='Rain')
fog_df = pd.read_excel(xls, sheet_name='Fog')

# Extract relevant data for plotting
rain_straightline = rain_df.iloc[2:][['Unnamed: 1']].astype(float).values.flatten()
rain_junction = rain_df.iloc[2:][['Unnamed: 8']].astype(float).values.flatten()
fog_straightline = fog_df.iloc[2:][['Unnamed: 1']].astype(float).values.flatten()
fog_junction = fog_df.iloc[2:][['Unnamed: 8']].astype(float).values.flatten()

# Extract intensity values
intensity_values = rain_df.iloc[2:, 0].astype(str).values

# Rain scenario plot
plt.figure(figsize=(12, 6))
plt.plot(intensity_values, rain_straightline, marker='o', label="Straight Line")
plt.plot(intensity_values, rain_junction, marker='o', label="Junction")

plt.xlabel("Rain Intensity")
plt.ylabel("Number of Points")
plt.title("Number of Points vs Rain Intensity")
plt.legend()
plt.grid(True)
plt.xticks(rotation=45)
plt.show()

# Fog scenario plot
plt.figure(figsize=(10, 5))
plt.plot(intensity_values, fog_straightline, marker='s', label="Straight Line")
plt.plot(intensity_values, fog_junction, marker='s', label="Junction")

plt.xlabel("Fog Intensity")
plt.ylabel("Number of Points")
plt.title("Number of Points vs Fog Intensity")
plt.legend()
plt.grid(True)
plt.xticks(rotation=45)
plt.show()
