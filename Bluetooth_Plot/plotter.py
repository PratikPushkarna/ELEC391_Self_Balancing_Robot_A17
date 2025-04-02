import serial
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

# Initialize data structures
max_data_points = 100
dataLists = [deque(maxlen=max_data_points) for _ in range(3)]  # degt, pwm, new_error
time_data = deque(maxlen=max_data_points)

# Set up the plot
fig, ax = plt.subplots(figsize=(10, 6))
lines = [
    ax.plot([], [], label="degt (Weighted Angle): 0.00", color='blue')[0],
    ax.plot([], [], label="pwm: 0.00", color='green')[0],
    ax.plot([], [], label="new_error: 0.00", color='red')[0]
]

# Configure plot
ax.set_ylim(-400, 400)
ax.set_xlim(0, max_data_points)
ax.set_title("Real-time Data Monitoring")
ax.set_xlabel("Time (samples)")
ax.set_ylabel("Values")
ax.grid(True)
ax.legend(loc="upper right")  # Initial legend creation

# Serial setup
ser = serial.Serial("COM11", 9600, timeout=1)
time.sleep(2)

def animate(i):
    global ax  # Ensure we can access the axis object
    
    while ser.in_waiting > 0:
        try:
            data_string = ser.readline().decode('ascii').strip()
            parts = data_string.split(',')
            
            if len(parts) >= 3:
                degt = float(parts[0])
                pwm = float(parts[1])
                new_error = float(parts[2])
                
                # Update data
                dataLists[0].append(degt)
                dataLists[1].append(pwm)
                dataLists[2].append(new_error)
                time_data.append(i)
                
                # Update lines
                lines[0].set_data(range(len(dataLists[0])), dataLists[0])
                lines[1].set_data(range(len(dataLists[1])), dataLists[1])
                lines[2].set_data(range(len(dataLists[2])), dataLists[2])
                
                # Update labels with current values
                lines[0].set_label(f"degt (Weighted Angle): {degt:.2f}")
                lines[1].set_label(f"pwm: {pwm:.2f}")
                lines[2].set_label(f"new_error: {new_error:.2f}")
                
                # Adjust x-axis
                if len(time_data) >= max_data_points:
                    ax.set_xlim(len(time_data)-max_data_points, len(time_data))
                
                # Remove existing legend and create new one
                if ax.legend_:
                    ax.legend_.remove()
                ax.legend(loc="upper right")
                
        except (ValueError, UnicodeDecodeError) as e:
            print(f"Data error: {e}")
            continue
    
    return lines + [ax.legend_]

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
