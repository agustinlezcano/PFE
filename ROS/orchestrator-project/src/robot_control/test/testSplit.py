
time_iter_input = input("Ingresa Tiempo Total T y cantidad de Iteraciones N: ").strip()

try:
    parts = time_iter_input.strip().split()
    print(len(parts))
    if time_iter_input.strip():
        t_total, n_iter = map(float, time_iter_input.split())
    for i in range(len(parts)):
        print(f"Parte {i}: '{parts[i]}'")
except Exception as e:
    print(f"Error: {e}")