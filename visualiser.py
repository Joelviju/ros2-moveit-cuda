
import pandas as pd
import matplotlib.pyplot as plt
import os

# ----------------------------
# Setup
# ----------------------------
# Create output folder
output_dir = "plots_CPU"
os.makedirs(output_dir, exist_ok=True)

# Load data
df = pd.read_csv("cpu_benchmark.csv")

# Filter successful runs
df_success = df[df["success"] == 1]

# ----------------------------
# 1. Planning Time vs Run
# ----------------------------
plt.figure()
plt.plot(
    df["run"].to_numpy(),
    df["planning_time"].to_numpy(),
    marker='o'
)
plt.xlabel("Run")
plt.ylabel("Planning Time (s)")
plt.title("Planning Time vs Run")
plt.grid()
plt.tight_layout()
plt.savefig(f"{output_dir}/planning_time_vs_run.png", dpi=300)
plt.close()

# ----------------------------
# 2. Histogram of Planning Time
# ----------------------------
plt.figure()
plt.hist(df_success["planning_time"].to_numpy(), bins=10)
plt.xlabel("Planning Time (s)")
plt.ylabel("Frequency")
plt.title("Planning Time Distribution")
plt.tight_layout()
plt.savefig(f"{output_dir}/planning_time_histogram.png", dpi=300)
plt.close()

# ----------------------------
# 3. Execution Time vs Run
# ----------------------------
plt.figure()
plt.plot(
    df_success["run"].to_numpy(),
    df_success["execution_time"].to_numpy(),
    marker='o'
)
plt.xlabel("Run")
plt.ylabel("Execution Time (s)")
plt.title("Execution Time vs Run")
plt.grid()
plt.tight_layout()
plt.savefig(f"{output_dir}/execution_time_vs_run.png", dpi=300)
plt.close()

# ----------------------------
# 4. Trajectory Points vs Planning Time
# ----------------------------
plt.figure()
plt.scatter(
    df_success["trajectory_points"].to_numpy(),
    df_success["planning_time"].to_numpy()
)
plt.xlabel("Trajectory Points")
plt.ylabel("Planning Time (s)")
plt.title("Trajectory Complexity vs Planning Time")
plt.grid()
plt.tight_layout()
plt.savefig(f"{output_dir}/trajectory_vs_planning.png", dpi=300)
plt.close()

# ----------------------------
# 5. Success vs Failure
# ----------------------------
success_count = df["success"].value_counts()

failures = success_count.get(0, 0)
successes = success_count.get(1, 0)

plt.figure()
plt.bar(["Failure", "Success"], [failures, successes])
plt.title("Success vs Failure")
plt.tight_layout()
plt.savefig(f"{output_dir}/success_vs_failure.png", dpi=300)
plt.close()

# ----------------------------
# DONE
# ----------------------------
print(f"✅ All plots saved in '{output_dir}' folder")

