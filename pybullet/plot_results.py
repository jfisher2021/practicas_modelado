import pandas as pd
import matplotlib.pyplot as plt

# Read the CSV file
data = pd.read_csv('robot_data.csv')

# Extract position and velocity data
position = data['Posición_Y']
velocity = data['Velocidad_Ruedas']

# Plot the data
plt.plot(position, velocity)

# Add labels to the axes
plt.xlabel('Position (units)')
plt.ylabel('Velocity (units)')

# Display the graph
plt.show()
