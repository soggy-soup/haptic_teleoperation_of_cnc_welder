import csv
import matplotlib.pyplot as plt

# Function to read and filter data from CSV file
def read_and_filter_csv_data(filename):
    x_data = []
    y_data = []
    last_x = None
    last_y = None
    
    with open(filename, mode='r') as file:
        reader = csv.DictReader(file)
        for row in reader:
            current_x = float(row['xpos'])
            current_y = float(row['ypos'])
            
            # Only add data points that are different from the last recorded point
            if current_x != last_x or current_y != last_y:
                x_data.append(current_x)
                y_data.append(current_y)
                last_x = current_x
                last_y = current_y
    
    return x_data, y_data

# Specify the filenames of the CSV data files
csv_filename1 = 'hapkit_data_full_space.csv'
csv_filename2 = 'hapkit_data_overlay.csv'

# Read and filter data from both CSV files
x_data1, y_data1 = read_and_filter_csv_data(csv_filename1)
x_data2, y_data2 = read_and_filter_csv_data(csv_filename2)

# Plotting the filtered data with different colors
plt.figure(figsize=(8, 6))
plt.plot(x_data1, y_data1, marker='o', linestyle='-', color='b', label='Space Limits')
plt.plot(x_data2, y_data2, marker='s', linestyle='--', color='r', label='Virtual Fixture')
plt.title('Overlay of Hapkit XY Data')
plt.xlabel('X')
plt.ylabel('Y')
plt.grid(True)
plt.legend()
plt.show()
