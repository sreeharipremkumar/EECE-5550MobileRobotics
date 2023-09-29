import os

import numpy as np
from PIL import Image
import matplotlib.pyplot as plt

Occupancy_Grid = []

# Maps cost(value) from start node s to v(key)
CostToReach_map = {}

# Maps parent(value) to child(key)
predecessor_map = {}

# Maps cost(value) of reaching the goal from the vertex(key)
EstTotalCost_map = {}

# Queue that stores node and their costs to reach goal
Vertex_Priority_Queue = []


def recover_path(start_vertex, goal_vertex):
    result_path = [goal_vertex]
    temp_vertex = goal_vertex

    while temp_vertex != start_vertex:
        temp_vertex = predecessor_map[temp_vertex]
        result_path.append(temp_vertex)
    return result_path


def get_valid_children(vertex):
    neighbors = np.array([(-1,1), (0,1), (1,1), (-1,0), (1,0), (-1,-1), (0,-1), (1,-1)])
    children_indices = vertex + neighbors
    all_children = Occupancy_Grid[children_indices[:,0], children_indices[:,1]]
    return children_indices[all_children == 1]


def euclidean_weight(vertex_1, vertex_2):
    x1, y1 = vertex_1
    x2, y2 = vertex_2
    return np.sqrt((x2-x1)**2 + (y2-y1)**2)


def est_cost_calculation(start_v, finish_v):
    x1, y1 = start_v
    x2, y2 = finish_v
    return np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def a_star_search_implementation(vertex_list, start_vertex, goal_vertex):
    for vertex_v in vertex_list:
        CostToReach_map[vertex_v] = float("inf")
        EstTotalCost_map[vertex_v] = float("inf")

    CostToReach_map[start_vertex] = 0
    EstTotalCost_map[start_vertex] = est_cost_calculation(start_vertex, goal_vertex)

    Vertex_Priority_Queue.append((EstTotalCost_map[start_vertex], start_vertex))
    print(Vertex_Priority_Queue)
    while Vertex_Priority_Queue:
        _, current_vertex = Vertex_Priority_Queue.pop()
        print(current_vertex, "and", goal_vertex)
        if current_vertex == goal_vertex:
            return recover_path(start_vertex, goal_vertex)

        current_v_children = get_valid_children(current_vertex)
        for each_child in current_v_children:
            child_ctr = CostToReach_map[current_vertex] + euclidean_weight(current_vertex, each_child)
            each_child = tuple(each_child.tolist())
            if child_ctr < CostToReach_map[each_child]:
                predecessor_map[each_child] = current_vertex
                CostToReach_map[each_child] = child_ctr
                prev_total_cost_child = EstTotalCost_map[each_child]
                EstTotalCost_map[each_child] = child_ctr + est_cost_calculation(each_child, goal_vertex)

                if (prev_total_cost_child, each_child) in Vertex_Priority_Queue:
                    pop_index = Vertex_Priority_Queue.index((prev_total_cost_child, each_child))
                    Vertex_Priority_Queue.pop(pop_index)
                    Vertex_Priority_Queue.append((EstTotalCost_map[each_child], each_child))
                    Vertex_Priority_Queue.sort(reverse=True)
                else:
                    Vertex_Priority_Queue.append((EstTotalCost_map[each_child], each_child))
                    Vertex_Priority_Queue.sort(reverse=True)
    return "Path not Found"


def main():
    occupancy_map_img = Image.open(r'C:\Users\akhil\All_my_codes\Northeastern_Courses\Mobile_robotics\occupancy_map.png')
    global Occupancy_Grid
    Occupancy_Grid = (np.asarray(occupancy_map_img) > 0).astype(int)

    rows, cols = Occupancy_Grid.shape
    vertex_list = []

    for r in range(rows):
        vertex_list += (list(zip([r]*cols, np.arange(cols))))

    path_vertices = np.array(a_star_search_implementation(vertex_list, (635,140), (350,400)))

    implot = plt.imshow(occupancy_map_img, cmap='gray')

    # put a red dot, size 40, at 2 locations:
    plt.scatter(x=path_vertices[:,1], y=path_vertices[:,0], c='r', s=4)

    plt.show()


if __name__ == "__main__":
    main()

