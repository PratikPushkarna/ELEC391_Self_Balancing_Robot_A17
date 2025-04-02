import serial
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Button
from collections import deque

# Initialize data structures for 5 variables
max_data_points = 100
dataLists = [deque(maxlen=max_data_points) for _ in range(5)]
time_data = deque(maxlen=max_data_points)

# Track visibility of each plot
plot_visibility = [True, True, True, True, True]

# Set up the plot
fig, ax = plt.subplots(figsize=(12, 8))
plt.subplots_adjust(bottom=0.3)  # Make space for buttons

# Create all plot lines
lines = [
    ax.plot([], [], label="degt (Weighted Angle): 0.00", color='blue', visible=True)[0],
    ax.plot([], [], label="pwm: 0.00", color='green', visible=True)[0],
    ax.plot([], [], label="new_error: 0.00", color='red', visible=True)[0],
    ax.plot([], [], label="degGyro: 0.00", color='purple', visible=True)[0],
    ax.plot([], [], label="degAcc: 0.00", color='orange', visible=True)[0]
]

# Configure plot
ax.set_ylim(-400, 400)
ax.set_xlim(0, max_data_points)
ax.set_title("Real-time Data Monitoring (5 Variables)")
ax.set_xlabel("Time (samples)")
ax.set_ylabel("Values")
ax.grid(True)
ax.legend(loc="upper right")

# Serial setup
ser = serial.Serial("COM11", 9600, timeout=1)
time.sleep(2)

# Button callback functions
def toggle_degt(event):
    plot_visibility[0] = not plot_visibility[0]
    lines[0].set_visible(plot_visibility[0])
    btn_degt.label.set_text(f"degt: {'ON' if plot_visibility[0] else 'OFF'}")
    plt.draw()

def toggle_pwm(event):
    plot_visibility[1] = not plot_visibility[1]
    lines[1].set_visible(plot_visibility[1])
    btn_pwm.label.set_text(f"pwm: {'ON' if plot_visibility[1] else 'OFF'}")
    plt.draw()

def toggle_error(event):
    plot_visibility[2] = not plot_visibility[2]
    lines[2].set_visible(plot_visibility[2])
    btn_error.label.set_text(f"error: {'ON' if plot_visibility[2] else 'OFF'}")
    plt.draw()

def toggle_gyro(event):
    plot_visibility[3] = not plot_visibility[3]
    lines[3].set_visible(plot_visibility[3])
    btn_gyro.label.set_text(f"gyro: {'ON' if plot_visibility[3] else 'OFF'}")
    plt.draw()

def toggle_acc(event):
    plot_visibility[4] = not plot_visibility[4]
    lines[4].set_visible(plot_visibility[4])
    btn_acc.label.set_text(f"acc: {'ON' if plot_visibility[4] else 'OFF'}")
    plt.draw()

# Create buttons
ax_degt = plt.axes([0.1, 0.15, 0.15, 0.06])
ax_pwm = plt.axes([0.3, 0.15, 0.15, 0.06])
ax_error = plt.axes([0.5, 0.15, 0.15, 0.06])
ax_gyro = plt.axes([0.7, 0.15, 0.15, 0.06])
ax_acc = plt.axes([0.9, 0.15, 0.15, 0.06])

btn_degt = Button(ax_degt, 'degt: ON')
btn_pwm = Button(ax_pwm, 'pwm: ON')
btn_error = Button(ax_error, 'error: ON')
btn_gyro = Button(ax_gyro, 'gyro: ON')
btn_acc = Button(ax_acc, 'acc: ON')

btn_degt.on_clicked(toggle_degt)
btn_pwm.on_clicked(toggle_pwm)
btn_error.on_clicked(toggle_error)
btn_gyro.on_clicked(toggle_gyro)
btn_acc.on_clicked(toggle_acc)

def animate(i):
    global ax
    
    while ser.in_waiting > 0:
        try:
            data_string = ser.readline().decode('ascii').strip()
            parts = data_string.split(',')
            
            if len(parts) >= 5:
                degt = float(parts[0])
                pwm = float(parts[1])
                new_error = float(parts[2])
                degGyro = float(parts[3])
                degAcc = float(parts[4])
                
                # Update all 5 data streams
                dataLists[0].append(degt)
                dataLists[1].append(pwm)
                dataLists[2].append(new_error)
                dataLists[3].append(degGyro)
                dataLists[4].append(degAcc)
                time_data.append(i)
                
                # Update all 5 lines (only if visible)
                for idx, line in enumerate(lines):
                    if plot_visibility[idx]:
                        line.set_data(range(len(dataLists[idx])), dataLists[idx])
                
                # Update all labels with current values
                lines[0].set_label(f"degt: {degt:.2f}")
                lines[1].set_label(f"pwm: {pwm:.2f}")
                lines[2].set_label(f"error: {new_error:.2f}")
                lines[3].set_label(f"degGyro: {degGyro:.2f}")
                lines[4].set_label(f"degAcc: {degAcc:.2f}")
                
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
