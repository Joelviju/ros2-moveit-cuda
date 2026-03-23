import pandas as pd
import matplotlib.pyplot as plt

# Load data
df = pd.read_csv("gpu_benchmark.csv")

# Remove failed runs for some plots
df_success = df[df["success"] == 1]

# ----------------------------
# 1. Planning Time vs Run
# ----------------------------
plt.figure()
plt.plot(df["run"], df["planning_time"], marker='o')
plt.xlabel("Run")
plt.ylabel("Planning Time (s)")
plt.title("Planning Time vs Run")
plt.grid()
plt.show()

# ----------------------------
# 2. Histogram
# ----------------------------
plt.figure()
plt.hist(df_success["planning_time"], bins=10)
plt.xlabel("Planning Time (s)")
plt.ylabel("Frequency")
plt.title("Planning Time Distribution")
plt.show()

# ----------------------------
# 3. Execution Time vs Run
# ----------------------------
plt.figure()
plt.plot(df_success["run"], df_success["execution_time"], marker='o')
plt.xlabel("Run")
plt.ylabel("Execution Time (s)")
plt.title("Execution Time vs Run")
plt.grid()
plt.show()

# ----------------------------
# 4. Trajectory Points vs Planning Time
# ----------------------------
plt.figure()
plt.scatter(df_success["trajectory_points"], df_success["planning_time"])
plt.xlabel("Trajectory Points")
plt.ylabel("Planning Time (s)")
plt.title("Trajectory Complexity vs Planning Time")
plt.grid()
plt.show()

# ----------------------------
# 5. Success vs Failure
# ----------------------------
success_count = df["success"].value_counts()

plt.figure()
plt.bar(["Failure", "Success"], [success_count[0], success_count[1]])
plt.title("Success vs Failure")
plt.show()