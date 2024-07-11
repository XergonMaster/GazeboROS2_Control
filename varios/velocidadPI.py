import math
import numpy as np
import timeit

# Funciones para pruebas individuales
def calc_with_math_pi():
    pi = math.pi
    return np.sin(pi)

def calc_with_truncated_pi():
    pi = 3.14
    return np.sin(pi)

# Funciones para pruebas vectorizadas
def numpy_sin_with_math_pi(arr):
    pi = math.pi
    return np.sin(arr * pi)

def numpy_sin_with_truncated_pi(arr):
    pi = 3.14
    return np.sin(arr * pi)

# Número de iteraciones para las pruebas
iterations = 1000000
array_iterations = 100
array_size = 1000000
num_tests = 10  # Número de veces que se repite cada prueba

# Arreglo para pruebas vectorizadas
array = np.random.rand(array_size)

# Realizar múltiples pruebas y almacenar los resultados
def run_tests():
    results = {
        "individual": {
            "math_pi": [],
            "truncated_pi": []
        },
        "vectorized": {
            "math_pi": [],
            "truncated_pi": []
        }
    }
    
    for _ in range(num_tests):
        # Pruebas individuales
        time_math_pi = timeit.timeit(calc_with_math_pi, number=iterations)
        time_truncated_pi = timeit.timeit(calc_with_truncated_pi, number=iterations)
        
        results["individual"]["math_pi"].append(time_math_pi)
        results["individual"]["truncated_pi"].append(time_truncated_pi)
        
        # Pruebas vectorizadas
        time_numpy_math_pi = timeit.timeit(lambda: numpy_sin_with_math_pi(array), number=array_iterations)
        time_numpy_truncated_pi = timeit.timeit(lambda: numpy_sin_with_truncated_pi(array), number=array_iterations)
        
        results["vectorized"]["math_pi"].append(time_numpy_math_pi)
        results["vectorized"]["truncated_pi"].append(time_numpy_truncated_pi)
    
    return results

# Calcular estadísticas
def calculate_statistics(results):
    stats = {
        "individual": {},
        "vectorized": {}
    }
    
    for key in results["individual"]:
        stats["individual"][key] = {
            "mean": np.mean(results["individual"][key]),
            "std": np.std(results["individual"][key])
        }
        
    for key in results["vectorized"]:
        stats["vectorized"][key] = {
            "mean": np.mean(results["vectorized"][key]),
            "std": np.std(results["vectorized"][key])
        }
        
    return stats

# Ejecutar pruebas
results = run_tests()

# Calcular estadísticas
stats = calculate_statistics(results)

# Mostrar resultados
print("Resultados de pruebas individuales (sin vectorizar):")
print(f"Usando math.pi: {stats['individual']['math_pi']['mean']:.6f} ± {stats['individual']['math_pi']['std']:.6f} segundos")
print(f"Usando pi truncado: {stats['individual']['truncated_pi']['mean']:.6f} ± {stats['individual']['truncated_pi']['std']:.6f} segundos")

print("\nResultados de pruebas vectorizadas (con numpy):")
print(f"Numpy con math.pi: {stats['vectorized']['math_pi']['mean']:.6f} ± {stats['vectorized']['math_pi']['std']:.6f} segundos")
print(f"Numpy con pi truncado: {stats['vectorized']['truncated_pi']['mean']:.6f} ± {stats['vectorized']['truncated_pi']['std']:.6f} segundos")
