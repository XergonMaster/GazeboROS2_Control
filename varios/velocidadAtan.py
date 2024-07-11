import math
import numpy as np
import timeit
import random

# Funciones para un solo valor
def atan_math(value):
    return math.atan(value)

def atan_numpy(value):
    return np.arctan(value)

# Número de iteraciones para las pruebas
num_tests = 10  # Número de veces que se repite cada prueba

# Realizar múltiples pruebas y almacenar los resultados
def run_tests():
    results = {
        "single_value": {
            "math": [],
            "numpy": []
        },
        "array": {
            "math": [],
            "numpy": []
        }
    }
    
    value = random.uniform(-1, 1)
    
    for _ in range(num_tests):
        # Generar un número aleatorio para cada prueba individual
        value = random.uniform(-1, 1)
        
        # Pruebas para un solo valor
        time_math = timeit.timeit(lambda: atan_math(value), number=1000000)
        time_numpy = timeit.timeit(lambda: atan_numpy(value), number=1000000)
        
        results["single_value"]["math"].append(time_math)
        results["single_value"]["numpy"].append(time_numpy)
        
        # Crear un arreglo grande de valores para pruebas vectorizadas
        array = np.random.rand(1000000)
        
        # Pruebas para un arreglo de valores
        time_numpy_array = timeit.timeit(lambda: np.arctan(array), number=100)
        time_math_array = timeit.timeit(lambda: [math.atan(x) for x in array], number=100)
        
        results["array"]["math"].append(time_math_array)
        results["array"]["numpy"].append(time_numpy_array)
    
    return results

# Calcular estadísticas
def calculate_statistics(results):
    stats = {
        "single_value": {},
        "array": {}
    }
    
    for key in results["single_value"]:
        stats["single_value"][key] = {
            "mean": np.mean(results["single_value"][key]),
            "std": np.std(results["single_value"][key])
        }
        
    for key in results["array"]:
        stats["array"][key] = {
            "mean": np.mean(results["array"][key]),
            "std": np.std(results["array"][key])
        }
        
    return stats

# Ejecutar pruebas
results = run_tests()

# Calcular estadísticas
stats = calculate_statistics(results)

# Mostrar resultados
print("Resultados de pruebas para un solo valor:")
print(f"math.atan: {stats['single_value']['math']['mean']:.6f} ± {stats['single_value']['math']['std']:.6f} segundos")
print(f"numpy.arctan: {stats['single_value']['numpy']['mean']:.6f} ± {stats['single_value']['numpy']['std']:.6f} segundos")

print("\nResultados de pruebas para un arreglo de valores:")
print(f"numpy.arctan (array): {stats['array']['numpy']['mean']:.6f} ± {stats['array']['numpy']['std']:.6f} segundos")
print(f"math.atan (array in loop): {stats['array']['math']['mean']:.6f} ± {stats['array']['math']['std']:.6f} segundos")
