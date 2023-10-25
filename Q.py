import tkinter as tk
import heapq
import math
import random

GRID_WIDTH = 60
GRID_HEIGHT = 60
CELL_SIZE = 15

grid = [[0 for _ in range(GRID_WIDTH)] for _ in range(GRID_HEIGHT)]

start = (2, 3)
destination = (58, 57)

obstacles = [
    (5, 0),(5, 1),(5, 2),(5, 3),(5, 4),(5, 5), (5, 6), (5, 7), (5, 8), (5, 9),(5, 10),(5, 11),(5, 12),(5, 13),(5, 14),(5, 15),
    (15, 15), (15, 16), (15, 17),
    (25, 25), (25, 26), (25, 27), (25, 28),
    (35, 35), (35, 36), (35, 37),
    (45, 45), (45, 46), (45, 47), (45, 48),
]

# obstacles = [
#     # Vertical wall close to the start
#     *[(3, i) for i in range(GRID_HEIGHT)],

#     # Vertical wall close to the destination
#     *[(57, i) for i in range(GRID_HEIGHT)],

#     # Horizontal wall on the top with a gap near the start
#     *[(i, 1) for i in range(1, GRID_WIDTH-1) if i != 3],

#     # Horizontal wall at the bottom with a gap near the destination
#     *[(i, 58) for i in range(1, GRID_WIDTH-1) if i != 57],

#     # Another horizontal wall in the middle without gaps
#     *[(i, 30) for i in range(GRID_WIDTH)]
# ]


for obstacle in obstacles:
    x, y = obstacle
    grid[y][x] = 1

def astar_search(start, goal):
    open_list = [(0, start)]
    came_from = {}
    g_score = {(x, y): float('inf') for x in range(GRID_WIDTH) for y in range(GRID_HEIGHT)}
    g_score[start] = 0

    while open_list:
        _, current = heapq.heappop(open_list)

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path

        for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
            new_x, new_y = current[0] + dx, current[1] + dy
            neighbor = (new_x, new_y)

            if 0 <= new_x < GRID_WIDTH and 0 <= new_y < GRID_HEIGHT and grid[new_y][new_x] != 1:
                tentative_g_score = g_score[current] + 1

                if tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score = tentative_g_score + heuristic(neighbor, goal)
                    heapq.heappush(open_list, (f_score, neighbor))

    return None

def heuristic(cell, goal):
    x1, y1 = cell
    x2, y2 = goal
    return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5

population_size = 50
num_generations = 100
mutation_rate = 0.1

def create_random_individual():
    return [(random.randint(0, GRID_WIDTH - 1), random.randint(0, GRID_HEIGHT - 1)) for _ in range(GRID_WIDTH)]

population = [create_random_individual() for _ in range(population_size)]

def fitness(individual):
    path_length = sum(heuristic(individual[i], individual[i + 1]) for i in range(len(individual) - 1))
    return path_length

for generation in range(num_generations):
    fitness_scores = [fitness(individual) for individual in population]

    total_fitness = sum(fitness_scores)
    parent1 = random.choices(population, weights=[total_fitness - score for score in fitness_scores])[0]
    parent2 = random.choices(population, weights=[total_fitness - score for score in fitness_scores])[0]

    crossover_point = random.randint(1, GRID_WIDTH - 1)
    child = parent1[:crossover_point] + parent2[crossover_point:]

    if random.random() < mutation_rate:
        mutation_point = random.randint(0, GRID_WIDTH - 1)
        child[mutation_point] = (random.randint(0, GRID_WIDTH - 1), random.randint(0, GRID_HEIGHT - 1))

    worst_index = fitness_scores.index(max(fitness_scores))
    population[worst_index] = child

best_individual = min(population, key=fitness)

def draw_grid(canvas):
    for x in range(0, GRID_WIDTH * CELL_SIZE, CELL_SIZE):
        canvas.create_line(x, 0, x, GRID_HEIGHT * CELL_SIZE, fill="gray")
    for y in range(0, GRID_HEIGHT * CELL_SIZE, CELL_SIZE):
        canvas.create_line(0, y, GRID_WIDTH * CELL_SIZE, y, fill="gray")

    for y in range(GRID_HEIGHT):
        for x in range(GRID_WIDTH):
            if grid[y][x] == 1:
                canvas.create_rectangle(x * CELL_SIZE, y * CELL_SIZE, (x + 1) * CELL_SIZE, (y + 1) * CELL_SIZE, fill="black")

def draw_path(canvas, path, color="blue"):
    for x, y in path:
        canvas.create_rectangle(x * CELL_SIZE, y * CELL_SIZE, (x + 1) * CELL_SIZE, (y + 1) * CELL_SIZE, fill=color)

def clear_path():
    canvas.delete("all")
    draw_grid(canvas)

def find_path_astar():
    path = astar_search(start, destination)
    if path:
        draw_path(canvas, path)
        status_label.config(text="A* Path Found")
    else:
        status_label.config(text="A* No Path Found")

def find_path_ga():
    path = best_individual
    if path:
        draw_path(canvas, path, color="green")
        status_label.config(text="GA Path Found")
    else:
        status_label.config(text="GA No Path Found")

root = tk.Tk()
root.title("Route Planner for RESCUE 1122")

canvas = tk.Canvas(root, width=GRID_WIDTH * CELL_SIZE, height=GRID_HEIGHT * CELL_SIZE)
canvas.pack()

draw_grid(canvas)

button_frame = tk.Frame(root)
button_frame.pack()

find_path_astar_button = tk.Button(button_frame, text="Find Path (A*)", command=find_path_astar)
find_path_astar_button.pack(side=tk.LEFT)

find_path_ga_button = tk.Button(button_frame, text="Find Path (GA)", command=find_path_ga)
find_path_ga_button.pack(side=tk.LEFT)

clear_path_button = tk.Button(button_frame, text="Clear Path", command=clear_path)
clear_path_button.pack(side=tk.LEFT)

status_label = tk.Label(root, text="", fg="green")
status_label.pack()

root.mainloop()
