import matplotlib.pyplot as plt
import re

def parse_algorithm_data(file_path='run_tests.txt'):
# Dicionário para armazenar os dados
    # Dicionário para armazenar os dados
    data = {}

    # Abre o arquivo de texto
    with open(file_path, 'r') as file:
        lines = file.readlines()

    # Variáveis temporárias para armazenar os dados do algoritmo
    algorithm = None
    cost = None
    time = None
    expanded_nodes = None

    # Itera sobre as linhas do arquivo
    for line in lines:
        line = line.strip()  # Remove espaços em branco extras

        # Quando encontramos o nome do algoritmo
        if line in ['Greedy', 'A*', 'BFS', 'IDS', 'UCS']:
            # Se já houver dados de um algoritmo anterior, os adicionamos ao dicionário
            if algorithm:
                data[algorithm] = {
                    'expanded_nodes': expanded_nodes,
                    'time': time,
                    'cost': cost
                }

            # Atualiza o nome do algoritmo
            algorithm = line
            continue

        # Lê o custo (linha que contém "Custo: ...")
        if line.startswith("Custo:"):
            cost = float(line.split()[1])  # Extrai o valor após "Custo:"
            continue  # Ignora o restante da linha

        # Lê o tempo (linha que contém "Time: ...")
        if line.startswith("Time:"):
            time = float(line.split()[1])

        # Lê os nós expandidos (linha que contém "Expanded nodes: ...")
        if line.startswith("Expanded nodes:"):
            expanded_nodes = int(line.split()[2])

    # Não se esquecer de adicionar o último algoritmo processado
    if algorithm:
        data[algorithm] = {
            'expanded_nodes': expanded_nodes,
            'time': time,
            'cost': cost
        }

    return data

data = parse_algorithm_data()
print(data)

# Extracting data for the bar chart and line chart

algorithms = list(data.keys())
costs = [data[algo]['cost'] for algo in algorithms]
expanded_nodes = [data[algo]['expanded_nodes'] for algo in algorithms]
times = [data[algo]['time'] for algo in algorithms]

plt.figure(figsize=(8, 6))
plt.bar(algorithms, costs, color='skyblue')
plt.title('Custos encontrados por algoritmo Grid Floresta_256x256')
plt.xlabel('Algoritmo')
plt.ylabel('Custo')
plt.savefig('cost_per_algorithm_Floresta_256x256.png')  # Save the bar chart as a PNG file

sorted_data = sorted(zip(times, expanded_nodes, algorithms), key=lambda x: x[0])
times_sorted, expanded_nodes_sorted, algorithms_sorted = zip(*sorted_data)

# Plot: Line chart for expanded nodes vs time (corrected axes)
plt.figure(figsize=(8, 6))
plt.plot(times_sorted, expanded_nodes_sorted, marker='o', linestyle='-', color='green')

# Annotate each point with its corresponding algorithm
for i, algo in enumerate(algorithms_sorted):
    plt.annotate(algo, (times_sorted[i], expanded_nodes_sorted[i]), textcoords="offset points", xytext=(0, 10), ha='center')

plt.title('Número de nós expandidos vs Tempo Grid Floresta_256x256')
plt.ylabel('Número de nós expandidos')
plt.xlabel('Tempo (s)')

# Save the figure
plt.savefig('expanded_nodes_vs_time_labeled_Floresta_256x256.png')  # Save the chart with labels