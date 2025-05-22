import sys
import time
import serial
import serial.tools.list_ports
import csv
import os
from datetime import datetime
import matplotlib.pyplot as plt
import numpy as np
from pick import pick
import pandas as pd

def read_series_to_csv(port, keep_device_data):
    #open serial port with error handling
    try:
        ser = serial.Serial(port, 250000, timeout=1)
    except serial.SerialException as e:
        sys.exit(f"Can't open {port}: {e}")
    if not ser.is_open:
        sys.exit(f"Can't open {port}")
    
    # keep sending until device replies
    while ser.readline().decode(errors='ignore').strip() != "CONTACT":
        data = bytearray(datetime.now().strftime("%Y-%m-%d_%H-%M-%S").encode("ascii","strict"))
        if keep_device_data:
            data.extend(b' 1 \n')
        else:
            data.extend(b' 0 \n')
        ser.write(data) # write if device memory should be cleared and current datetime to synchronize device RTC
        ser.flush()
    output_folder = None
    started = False
    times = []
    distances = []

    while True:
        line = ser.readline().decode(errors='ignore').strip()
        if line == "": 
            continue # keep reading until you get a non-empty line
        if line == "TRANSFER COMPLETE":
            break
        if line.startswith("START"):
            # convert the string to datetime
            timestamp = datetime.strptime(line, "START %Y-%m-%d_%H-%M-%S")
            started = True
            times = []
            distances = []

            # create the output folder once on the first START
            if output_folder is None:
                output_folder = mydir = os.path.join(
                    os.getcwd(), 
                    datetime.now().strftime('gait_%Y-%m-%d_%H-%M-%S'))
                try:
                    os.makedirs(mydir)
                except OSError as e:
                    sys.exit(f"Couldn't create output folder: {e}")
            continue

        if line == "END":
            # save as .csv
            timestamp_string = timestamp.strftime("%Y-%m-%d_%H-%M-%S")
            filename = os.path.join(output_folder, f"{timestamp_string}.csv")
            df = pd.DataFrame({"Time (s)" : times, "Distance (m)" : distances})
            df.to_csv(filename, index=False)
            started = False
            continue
        if started:
            try:
                print(line)
                # read the comma-separated values
                values = [float(x.strip()) for x in line.split(",")]
                distances.append(values[0])
                times.append(values[1])
            except ValueError:
                print("Skip invalid data line")
    
    return output_folder            

def plot_series_from_csv(folder_path, selected_list):
    for filename in selected_list:
        file_path = os.path.join(folder_path, filename)
        times = []
        distances = []

        with open(file_path, 'r') as file:
            reader = csv.reader(file)
            for row in reader:
                try:
                    distance, time = float(row[1]), float(row[0])
                    distances.append(distance)
                    times.append(time)
                except ValueError:
                    continue

        times_np = np.array(times)
        distances_np = np.array(distances)

        # linear regression
        slope, intercept = np.polyfit(times_np, distances_np, 1)
        regression_line = slope * times_np + intercept

        plt.figure()
        plt.plot(times_np, distances_np, 'o', color='blue', markersize=4, label='Data')
        plt.plot(times_np, regression_line, '--', color='red', linewidth=1.8, label='Linear Fit')

        equation = f"y = {slope:.4f}x + {intercept:.4f}"
        plt.annotate(equation, xy=(0.05, 0.95), xycoords='axes fraction',
                        fontsize=10, color='red', ha='left', va='top',
                        bbox=dict(boxstyle='round,pad=0.3', edgecolor='red', facecolor='white', alpha=0.8))

        plt.xlabel('Time (s)')
        plt.ylabel('Distance (m)')
        plt.title(f'Distance vs Time: {filename}')
        plt.legend()
        plt.grid(True)
        plt.tight_layout()

        # save svg
        svg_path = os.path.join(folder_path, f"{filename}.svg")
        plt.savefig(svg_path, format='svg')
        print(f"Saved vector plot: {svg_path}")

    plt.show()


def main():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("No COM ports found. Make sure the device is plugged in via USB and try again.")
        return
    # list COM ports and let user choose one
    title = 'Available COM ports: '
    options = ports
    option, index = pick(options, title)
    selected_port = ports[index].device

    title = 'Do you want to copy or move the data to this PC? '
    options = ("Copy","Move (erases data from lidar device after transfer)")
    option, index = pick(options, title)
    if index == 0:
        keep_device_data = True
    else:
        keep_device_data = False

    # read data from COM port and output to .csv file
    output_folder = read_series_to_csv(selected_port, keep_device_data)

    if output_folder and os.path.exists(output_folder):
        folder_list = []
        folder_list.append("All")
        folder_list.append("None")
        for filename in sorted(os.listdir(output_folder)):
            if filename.endswith('.csv'):
                folder_list.append(filename)
        title = 'Visualize data series with linear regression and save figures? (press SPACE to mark, ENTER to continue)'
        options = folder_list
        selected = pick(options, title, multiselect=True, min_selection_count=1)
        selected_list = [item[0] for item in selected]
        if ("All" in selected_list):
            folder_list.remove("All")
            folder_list.remove("None")
            plot_series_from_csv(output_folder, folder_list)
        elif not "None" in selected_list:
            plot_series_from_csv(output_folder, selected_list)
    else:
        print("No data was found on the device.")

if __name__ == "__main__":
    main()
