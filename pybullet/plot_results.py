import numpy as np
import math
import csv

import matplotlib.pyplot as plt

def plot_results(csv_files):
    plt.figure(figsize=(10, 6))
    
    colors = ['purple', 'blue', 'green']
    # List to store the RMSE of each file
    rmses = []

    for i, csv_file in enumerate(csv_files):
        with open(csv_file, 'r') as file:
            reader = csv.reader(file)
            next(reader)  # Skip header
            time, robot_position, robot_velocity, wheel_velocity, wheel_force = [], [], [], [], []
            for row in reader:
                time.append(float(row[0]))
                robot_position.append(float(row[1]))
                robot_velocity.append(float(row[2]))
                wheel_velocity.append(float(row[3]))
                wheel_force.append(float(row[4]))

        # Convert lists to numpy arrays
        time = np.array(time)
        robot_position = np.array(robot_position)
        robot_velocity = np.array(robot_velocity)
        wheel_velocity = np.array(wheel_velocity)
        wheel_force = np.array(wheel_force)

        # Plot the results
        plt.plot(robot_position, robot_velocity, label=f'File: {csv_file}', color=colors[i])

        # Calculate the RMSE and add it to the list
        MSE = np.square(np.subtract(robot_velocity, 2)).mean()
        RMSE = math.sqrt(MSE)
        rmses.append(RMSE)

    # Update the legend labels to include the RMSE
    # First, create the legend with the original labels
    labels = [f'{csv_files[i]} (RMSE: {rmses[i]:.5f})' for i in range(len(csv_files))]
    plt.legend(labels, loc='best')

    plt.xlabel('Robot Position (Y)')
    plt.ylabel('Robot Velocity')
    plt.title('Variation of Robot Velocity with Position')
    plt.grid(True)
    plt.show()

# Paths to the CSV files
# video 1
#csv_files = ['csvs/robot_data31.csv', 'csvs/robot_data32.csv', 'csvs/Fase3.csv']
#fase 4 
#csv_files = ['csvs/Fase4.csv']
# video 2
csv_files = ['csvs/Fase4.csv', 'csvs/Fase3.csv']

# Call the function to plot the results
plot_results(csv_files)
