import matplotlib.pyplot as plt
import numpy as np

# Load data
filename = "Test 3/test3.txt"

with open(filename, "r") as f:
    readings = [float(line.strip()) for line in f if line.strip()]

readings = np.array(readings)
time_ms = np.arange(len(readings)) * 50  
time_seconds = time_ms / 1000 

# Basic Stats
mean_val = np.mean(readings)
std_val = np.std(readings)
min_val = np.min(readings)
max_val = np.max(readings)

# Setpoint
setpoint = 20

# PID Performance Metrics
error = readings - setpoint
absolute_error = np.abs(error)
rmse = np.sqrt(np.mean(error**2))  # Root Mean Squared Error
max_overshoot = max(0, np.max(readings) - setpoint) if np.max(readings) > setpoint else 0

# Integral of Absolute Error
iae = np.trapezoid(absolute_error, time_seconds)

# Error statistics 
mean_abs_error = np.mean(absolute_error)
max_abs_error = np.max(absolute_error)
final_error = error[-1]

# Create plot
plt.figure(figsize=(12, 6))
plt.plot(time_ms, readings, label="Ball Position", linewidth=2, color='blue')
plt.axhline(y=setpoint, color='red', linestyle='--', linewidth=2, 
           label=f"Setpoint = {setpoint} cm")

# plt.title("Ball and Beam System PID Tuning: Kp - 8, Ki - 0.8, Kd - 1.2", fontsize=14, fontweight='bold') # Test 1
# plt.title("Ball and Beam System PID Tuning: Kp - 6, Ki - 0.5, Kd - 3", fontsize=14, fontweight='bold') # Test 2
plt.title("Ball and Beam System PID Tuning: Kp - 7, Ki - 0.6, Kd - 2", fontsize=14, fontweight='bold') # Test 3

plt.xlabel("Time (ms)", fontsize=12)
plt.ylabel("Position (cm)", fontsize=12)
plt.xlim(0, time_ms[-1])
plt.ylim(0, max(max(readings), setpoint) + 5)
plt.grid(True, alpha=0.3)
plt.legend(loc='lower right')  # Moved to bottom right

# PID Performance Metrics box (top right)
pid_performance_text = (
    f"PID Performance Metrics:\n"
    f"• RMSE: {rmse:.2f} cm\n"
    f"• Max Overshoot: {max_overshoot:.2f} cm\n"
    f"• IAE: {iae:.3f}\n"
)

pid_performance_text += (
    f"• Steady-State Error: {final_error:.2f} cm\n"
    f"• Final Position: {readings[-1]:.2f} cm"
)

# Statistics & Error Metrics box (top left - green)
stats_text = (
    f"Statistics & Error Metrics:\n"
    f"• Mean Position: {mean_val:.2f} cm\n"
    f"• Std Deviation: {std_val:.2f} cm\n"
    f"• Min Position: {min_val:.2f} cm\n"
    f"• Max Position: {max_val:.2f} cm\n"
    f"• Mean Abs Error: {mean_abs_error:.2f} cm\n"
    f"• Max Abs Error: {max_abs_error:.2f} cm\n"
    f"• Samples: {len(readings)}\n"
    f"• Duration: {time_ms[-1]} ms"
)

# Place Statistics box at top left (green)
plt.text(0.59, 0.98, stats_text, transform=plt.gca().transAxes,
        verticalalignment='top', horizontalalignment='left',
        bbox=dict(facecolor='lightgreen', alpha=0.9, boxstyle='round'),
        fontsize=9, family='monospace')

# Place PID Performance box at top right (blue)
plt.text(0.79, 0.98, pid_performance_text, transform=plt.gca().transAxes,
        verticalalignment='top', horizontalalignment='left',
        bbox=dict(facecolor='lightblue', alpha=0.9, boxstyle='round'),
        fontsize=9, family='monospace')

plt.tight_layout()
plt.show()