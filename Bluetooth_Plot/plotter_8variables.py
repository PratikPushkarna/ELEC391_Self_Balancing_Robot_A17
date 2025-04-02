import serial
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Button
from collections import deque

# Initialize data structures for 8 variables
max_data_points = 100
dataLists = [deque(maxlen=max_data_points) for _ in range(8)]
time_data = deque(maxlen=max_data_points)

# Track visibility of each plot (now 8 variables)
plot_visibility = [True] * 8

# Set up the plot with more vertical space
fig, ax = plt.subplots(figsize=(14, 10))
plt.subplots_adjust(bottom=0.35)  # Increased space for more buttons

# Create all plot lines with distinct colors
colors = ['blue', 'green', 'red', 'purple', 'orange', 'cyan', 'magenta', 'brown']
lines = [
    ax.plot([], [], label="degt: 0.00", color=colors[0], visible=True)[0],
    ax.plot([], [], label="pwm: 0.00", color=colors[1], visible=True)[0],
    ax.plot([], [], label="error: 0.00", color=colors[2], visible=True)[0],
    ax.plot([], [], label="degGyro: 0.00", color=colors[3], visible=True)[0],
    ax.plot([], [], label="degAcc: 0.00", color=colors[4], visible=True)[0],
    ax.plot([], [], label="integral: 0.00", color=colors[5], visible=True)[0],
    ax.plot([], [], label="derivative: 0.00", color=colors[6], visible=True)[0],
    ax.plot([], [], label="proportional: 0.00", color=colors[7], visible=True)[0]
]

# Configure plot
ax.set_ylim(-400, 400)
ax.set_xlim(0, max_data_points)
ax.set_title("Real-time Data Monitoring (8 Variables)")
ax.set_xlabel("Time (samples)")
ax.set_ylabel("Values")
ax.grid(True)
ax.legend(loc="upper right")

# Serial setup
ser = serial.Serial("COM11", 9600, timeout=1)
time.sleep(2)

# Button callback functions (now 8)
def toggle_plot(n):
    def callback(event):
        plot_visibility[n] = not plot_visibility[n]
        lines[n].set_visible(plot_visibility[n])
        buttons[n].label.set_text(f"{button_labels[n]}: {'ON' if plot_visibility[n] else 'OFF'}")
        plt.draw()
    return callback

# Button labels and creation
button_labels = [
    "degt", "pwm", "error", "gyro", "acc", 
    "integral", "derivative", "proportional"
]

# Create buttons in two rows
buttons = []
button_axes = []

# First row of buttons
for i in range(4):
    ax_btn = plt.axes([0.1 + i*0.2, 0.25, 0.15, 0.06])
    btn = Button(ax_btn, f"{button_labels[i]}: ON")
    btn.on_clicked(toggle_plot(i))
    buttons.append(btn)
    button_axes.append(ax_btn)

# Second row of buttons
for i in range(4, 8):
    ax_btn = plt.axes([0.1 + (i-4)*0.2, 0.15, 0.15, 0.06])
    btn = Button(ax_btn, f"{button_labels[i]}: ON")
    btn.on_clicked(toggle_plot(i))
    buttons.append(btn)
    button_axes.append(ax_btn)

def animate(i):
    global ax
    
    while ser.in_waiting > 0:
        try:
            data_string = ser.readline().decode('ascii').strip()
            parts = data_string.split(',')
            
            if len(parts) >= 8:  # Now expecting 8 values
                degt = float(parts[0])
                pwm = float(parts[1])
                new_error = float(parts[2])
                degGyro = float(parts[3])
                degAcc = float(parts[4])
                integral = float(parts[5])
                derivative = float(parts[6])
                proportional = float(parts[7])
                
                # Update all 8 data streams
                dataLists[0].append(degt)
                dataLists[1].append(pwm)
                dataLists[2].append(new_error)
                dataLists[3].append(degGyro)
                dataLists[4].append(degAcc)
                dataLists[5].append(integral)
                dataLists[6].append(derivative)
                dataLists[7].append(proportional)
                time_data.append(i)
                
                # Update all lines (only if visible)
                for idx, line in enumerate(lines):
                    if plot_visibility[idx]:
                        line.set_data(range(len(dataLists[idx])), dataLists[idx])
                
                # Update all labels with current values
                lines[0].set_label(f"degt: {degt:.2f}")
                lines[1].set_label(f"pwm: {pwm:.2f}")
                lines[2].set_label(f"error: {new_error:.2f}")
                lines[3].set_label(f"gyro: {degGyro:.2f}")
                lines[4].set_label(f"acc: {degAcc:.2f}")
                lines[5].set_label(f"integral: {integral:.2f}")
                lines[6].set_label(f"derivative: {derivative:.2f}")
                lines[7].set_label(f"proportional: {proportional:.2f}")
                
                # Adjust x-axis
                if len(time_data) >= max_data_points:
                    ax.set_xlim(len(time_data)-max_data_points, len(time_data))
                
                # Update legend
                if ax.legend_:
                    ax.legend_.remove()
                ax.legend(loc="upper right")
                
        except (ValueError, UnicodeDecodeError, IndexError) as e:
            print(f"Data error: {e}")
            continue
    
    return [line for line, visible in zip(lines, plot_visibility) if visible] + [ax.legend_]

# Animation setup
ani = animation.FuncAnimation(
    fig, animate,
    interval=10,
    blit=True,
    cache_frame_data=False
)

plt.tight_layout()
plt.show()
ser.close()
