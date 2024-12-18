import serial
import serial.tools.list_ports
import time
import numpy as np  # For statistical calculations
import csv  # For writing to CSV files

def find_esp32_port():
    """
    Finds the ESP32's serial port by looking for known identifiers.
    """
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if "USB" in port.description or "CP210" in port.description:
            return port.device
    return None

def compute_statistics(data):
    """
    Computes mean, median, min, max, and RMS from the given list of data.
    """
    mean = np.mean(data)
    median = np.median(data)
    min_val = np.min(data)
    max_val = np.max(data)
    rms = np.sqrt(np.mean(np.square(data)))

    return mean, median, min_val, max_val, rms


# def save_to_csv(filename, data):
#     """
#     Saves the collected data to a CSV file.
#     """
#     try:
#         with open(filename, mode='w', newline='') as file:
#             writer = csv.writer(file)
#             writer.writerow(["Sample Number", "Value"])  # Write header
#             for i, value in enumerate(data):
#                 writer.writerow([i + 1, value])
#         print(f"Data saved to {filename}")
#     except Exception as e:
#         print(f"Error saving to CSV: {e}")

def save_to_csv(filename, data):
    """
    Saves the collected data to a CSV file.
    """
    import csv
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["DutyA", "DutyB", "DutyC"])
        for row in data:
            writer.writerow(row)




def monitor_serial_and_collect(port, baudrate=115200, timeout=1, sample_size=100):
    """
    Collects `sample_size` values from the serial port and computes statistics.
    """
    try:
        with serial.Serial(port, baudrate, timeout=timeout) as ser:
            print(f"Connected to {port} at {baudrate} baud.")
            print(f"Collecting {sample_size} samples...")


            dutyA_values = []
            dutyB_values = []
            dutyC_values = []

            while len(dutyA_values) < sample_size:

                if ser.in_waiting > 0:
                    # Read a line from the serial port
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    print(line)
                    if not line:
                        continue 
                    # print(f"Raw data: {line}") 
                    
                    # Try to extract a floating-point value
                    try:

                        parts = line.split(',')
                        if len(parts) == 3:  # Ensure there are exactly three values
                            dutyA = float(parts[0].strip())  # First value
                            dutyB = float(parts[1].strip())  # Second value
                            dutyC = float(parts[2].strip())  # Third value
                            
                            # Add values to respective lists
                            dutyA_values.append(dutyA)
                            dutyB_values.append(dutyB)
                            dutyC_values.append(dutyC)
                            
                        # angle = float(line.split(":")[-1].strip())  # Assuming "Angle: <value>"
                        # values.append(angle)
                        # print(f"Collected: {angle}")
                    except ValueError:
                        print(f"Invalid data: {line}")  # Handle invalid lines
                
                time.sleep(0.01)  # Avoid tight polling

            # Compute statistics once the data is collected
            # mean, median, min_val, max_val, rms = compute_statistics(values)
            # print("\n--- Statistics ---")
            # print(f"Mean: {mean:.4f}")
            # print(f"Median: {median:.4f}")
            # print(f"Min: {min_val:.4f}")
            # print(f"Max: {max_val:.4f}")
            # print(f"RMS: {rms:.4f}")
            save_to_csv("result.csv", zip(dutyA_values, dutyB_values, dutyC_values))

            # save_to_csv("result.csv", values)

            

    except serial.SerialException as e:
        print(f"Error: {e}")
    except KeyboardInterrupt:
        print("\nMonitoring stopped.")

if __name__ == "__main__":
    # Find the ESP32's port automatically
    esp32_port = find_esp32_port()
    if esp32_port:
        print(f"ESP32 detected on port: {esp32_port}")
        monitor_serial_and_collect(esp32_port)


    else:
        print("No ESP32 device found. Please check the connection.")




# import serial
# import serial.tools.list_ports
# import time

# def find_esp32_port():
#     """
#     Finds the ESP32's serial port by looking for known identifiers.
#     """
#     ports = serial.tools.list_ports.comports()
#     for port in ports:
#         if "USB" in port.description or "CP210" in port.description:
#             return port.device
#     return None

# def monitor_serial(port, baudrate=115200, timeout=1):
#     """
#     Opens the serial port and captures data.
#     """
#     try:
#         # Open the serial connection
#         with serial.Serial(port, baudrate, timeout=timeout) as ser:
#             print(f"Connected to {port} at {baudrate} baud.")
#             print("Press Ctrl+C to stop monitoring.\n")
            
#             while True:
#                 if ser.in_waiting > 0:
#                     # Read data from the serial port
#                     data = ser.readline().decode('utf-8', errors='ignore').strip()
#                     print(f"Received: {data}")
                
#                 time.sleep(0.1)  # Adjust polling frequency if needed
                
#     except serial.SerialException as e:
#         print(f"Error: {e}")
#     except KeyboardInterrupt:
#         print("\nMonitoring stopped.")

# if __name__ == "__main__":
#     # Find the ESP32's port automatically
#     esp32_port = find_esp32_port()
#     if esp32_port:
#         print(f"ESP32 detected on port: {esp32_port}")
#         monitor_serial(esp32_port)
#     else:
#         print("No ESP32 device found. Please check the connection.")
