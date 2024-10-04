import serial
import numpy as np
import plotly.graph_objects as go

# Open the COM port
print("Opening COM port...")
ser = serial.Serial('COM6', 115200)

# Initialize empty lists for x and y values
x_sig = []
y_sig = []
proc_times = []

# Create a scatter plot
fig = go.Figure(data=go.Scatter())

print("Waiting to receive serial data...")

while ser.in_waiting == 0:
    pass

print("Receiving serial data...")
# Continuously read data from the COM port
while True:
    if ser.in_waiting > 0:
        # Read a line from the COM port
        line = ser.readline()
        line = line.decode().strip('\x00').strip('\r\n')
        print(f"recv:{line}")

        # Check if tracing should stop
        if "END" in line:
            print(f"All data received, stopping trace...")
            break

        # Split the line into x and y values
        values = line.split(",")

        # Append the values to the lists
        x_sig.append(float(values[0]))
        y_sig.append(float(values[1]))
        # x_sig.append(float(values[0]) * (3.3/ pow(2, 12)))
        # y_sig.append(float(values[1]) * (100 / pow(2, 16)))
        proc_times.append(float(values[2]))

print(f"Maximum processing time: {max(proc_times)}")

# Update the scatter plot
print("Plotting data...")
n = np.arange(len(x_sig))
fig.add_trace(go.Scatter(x=n, y=x_sig, mode='lines', name='Dry signal'))
fig.add_trace(go.Scatter(x=n, y=y_sig, mode='lines', name='Wet signal'))
fig.add_trace(go.Scatter(x=n, y=proc_times, mode='lines', name='Processing times (counts)'))#, visible='legendonly'))
fig.add_hline(y=max(proc_times), line_dash='dash', line_color='red', line_width=1, name='Max processing time')
fig.add_annotation(x=0, y=max(proc_times), text=f'Max processing time {max(proc_times)}', showarrow=True, arrowhead=1, ax=0, ay=-40)

# Show the plot
fig.show()
print("DONE")