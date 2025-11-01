import pandas as pd
import matplotlib.pyplot as plt

path_of_file= "path_log.csv"
df = pd.read_csv(path_of_file)
print(df.head())

plt.figure(figsize=(300, 100))
plt.plot(df["x_axis_cm"], df["y_axis_cm"], "-o", linewidth=1, markersize=1, label="ev3's trajectory")

plt.scatter(df["x_axis_cm"].iloc[0], df["y_axis_cm"].iloc[0], color="green", s=60, label="ev3's woken up")

if "cross" in df.columns:
    cross1 = df[df["cross"] == 1]
    if not cross1.empty:
        first_cross1_reading = cross1.iloc[0]
        plt.scatter(first_cross1_reading["x_axis_cm"], first_cross1_reading["y_axis_cm"], color="black", s=100, label="360")

plt.scatter(df["x_axis_cm"].iloc[-1], df["y_axis_cm"].iloc[-1], color="red", s=80, label="ev3's went to sleep")

plt.title("Visualisation of path of ev3")
plt.xlabel("X in cm")
plt.ylabel("Y in cm")
plt.legend()
plt.axis("equal")
plt.grid(True)
plt.show()
