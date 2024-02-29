import numpy as np
import matplotlib.pyplot as plt
import math
import csv

def plot_results(csv_files):
    # Crear el gráfico
    plt.figure(figsize=(10, 6))
    
    # Define a list of colors
    colors = ['purple', 'blue', 'green']
    
    for i, csv_file in enumerate(csv_files):
        # Leer los datos del archivo CSV
        with open(csv_file, 'r') as file:
            reader = csv.reader(file)
            # Saltar la primera fila si contiene encabezados
            next(reader)
            # Leer datos y almacenar en listas
            tiempo = []
            posicion_robot = []
            velocidad_robot = []
            velocidad_ruedas = []
            fuerza_ruedas = []
            for row in reader:
                tiempo.append(float(row[0]))
                posicion_robot.append(float(row[1]))
                velocidad_robot.append(float(row[2]))
                velocidad_ruedas.append(float(row[3]))
                fuerza_ruedas.append(float(row[4]))

        # Convertir listas a arrays de numpy para facilitar el manejo
        tiempo = np.array(tiempo)
        posicion_robot = np.array(posicion_robot)
        velocidad_robot = np.array(velocidad_robot)
        velocidad_ruedas = np.array(velocidad_ruedas)
        fuerza_ruedas = np.array(fuerza_ruedas)

        # Plotear los resultados con un color diferente para cada archivo
        for i, csv_file in enumerate(csv_files):
            # Leer los datos del archivo CSV
            with open(csv_file, 'r') as file:
                reader = csv.reader(file)
                # Saltar la primera fila si contiene encabezados
                next(reader)
                # Leer datos y almacenar en listas
                tiempo = []
                posicion_robot = []
                velocidad_robot = []
                velocidad_ruedas = []
                fuerza_ruedas = []
                for row in reader:
                    tiempo.append(float(row[0]))
                    posicion_robot.append(float(row[1]))
                    velocidad_robot.append(float(row[2]))
                    velocidad_ruedas.append(float(row[3]))
                    fuerza_ruedas.append(float(row[4]))

            # Convertir listas a arrays de numpy para facilitar el manejo
            tiempo = np.array(tiempo)
            posicion_robot = np.array(posicion_robot)
            velocidad_robot = np.array(velocidad_robot)
            velocidad_ruedas = np.array(velocidad_ruedas)
            fuerza_ruedas = np.array(fuerza_ruedas)

            # Plotear los resultados con un color diferente para cada archivo
            plt.plot(posicion_robot, velocidad_robot, label=f'Archivo: {csv_file}', color=colors[i])

            # Calcular el RMSE
            MSE = np.square(np.subtract(velocidad_robot, 2)).mean()
            RMSE = math.sqrt(MSE)
            print("Root Mean Square Error:", RMSE)

            # Mostrar el RMSE en el gráfico

        plt.xlabel('Posición del robot (Y)')
        plt.ylabel('Velocidad del robot')
        plt.title('Variación de la velocidad del robot en función de su posición')
        plt.legend()
        plt.grid(True)
        plt.show()

# Rutas a los archivos CSV
csv_files = ['robot_data31.csv', 'robot_data33.csv', 'probar.csv']
# Llamar a la función para plotear los resultados
plot_results(csv_files)
