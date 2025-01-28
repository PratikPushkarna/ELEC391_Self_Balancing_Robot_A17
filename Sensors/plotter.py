

import serial
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Function to update the plot
def animate(i, dataLists, ser, legend_texts):
    ser.write(b'g')  # Send 'g' to Arduino to trigger data transmission
    arduinoData_string = ser.readline().decode('ascii').strip()  # Read and decode the data

    try:
        # Expecting data in the format "degt,degx,degacc"
        degt, degx, degacc = map(float, arduinoData_string.split(','))
        
        # Append new data to corresponding lists
        dataLists[0].append(degt)
        dataLists[1].append(degx)
        dataLists[2].append(degacc)
        
        # Update the legend text with the current values
        legend_texts[0] = f"degt (Weighted Angle): {degt:.2f}"
        legend_texts[1] = f"degx (Gyroscope Angle): {degx:.2f}"
        legend_texts[2] = f"degacc (Accelerometer Angle): {degacc:.2f}"
        
    except ValueError:
        # Ignore if the data is not properly formatted
        return

    # Keep the lists fixed to the last 50 data points
    for dataList in dataLists:
        dataList[:] = dataList[-50:]

    ax.clear()  # Clear previous plot

    # Plot each data list with labels
    ax.plot(dataLists[0], label=legend_texts[0], color="blue")
    ax.plot(dataLists[1], label=legend_texts[1], color="green")
    ax.plot(dataLists[2], label=legend_texts[2], color="red")

    ax.set_ylim([-180, 180])  # Set Y-axis limits
    ax.set_title("Real-time Angle Measurements")
    ax.set_xlabel("Time (frames)")
    ax.set_ylabel("Angle (Degrees)")
    ax.legend(loc="upper right")  # Add legend to the plot

# Initialize data lists for each value
dataLists = [[], [], []]  # [degt_values, degx_values, degacc_values]
legend_texts = [
    "degt (Complimentary Angle): 0.00",
    "degx (Gyroscope Angle): 0.00",
    "degacc (Accelerometer Angle): 0.00"
]

# Set up the plot
fig = plt.figure()
ax = fig.add_subplot(111)

# Configure the serial port
ser = serial.Serial("COM5", 9600)  # Update 'COM5' to match your Arduino port
time.sleep(2)  # Wait for the Arduino to initialize

# Set up the animation function
ani = animation.FuncAnimation(fig, animate, frames=100, fargs=(dataLists, ser, legend_texts), interval=10)

# Show the plot
plt.show()

# Close the serial port after exiting
ser.close()


