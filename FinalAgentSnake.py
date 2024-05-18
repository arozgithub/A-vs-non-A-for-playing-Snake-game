from collections import deque
from abc import ABC, abstractmethod


class Agent(object):
    def SearchSolution(self, state):
        return []


# sir wala code


# class AgentSnake(Agent):
#     def SearchSolution(self, state):
#         FoodX = state.FoodPosition.X
#         FoodY = state.FoodPosition.Y
#
#         HeadX = state.snake.HeadPosition.X  # L
#         HeadY = state.snake.HeadPosition.Y  # T
#
#         DR = FoodY - HeadY
#         DC = FoodX - HeadX
#
#         plan = []
#
#         F = -1
#         if (DR == 0 and state.snake.HeadDirection.X * DC < 0):
#             plan.append(0)
#             F = 6
#
#         if (state.snake.HeadDirection.Y * DR < 0):
#             plan.append(3)
#             if (DC == 0):
#                 F = 9
#             else:
#                 DC = DC - 1
#         Di = 6
#         if (DR < 0):
#             Di = 0
#             DR = -DR
#         for i in range(0, int(DR)):
#             plan.append(Di)
#         Di = 3
#         if (DC < 0):
#             Di = 9
#             DC = -DC
#         for i in range(0, int(DC)):
#             plan.append(Di)
#         if (F > 0):
#             plan.append(F)
#             F = -1
#
#         return plan
#
#     def showAgent():
#         print("A Snake Solver By MB")

# A star
#
# class AgentSnake(Agent):
#
#     def SearchSolution(self, state):
#         from queue import PriorityQueue
#
#         class Node:
#             def __init__(self, position, parent=None, g=0, h=0):
#                 self.position = position
#                 self.parent = parent
#                 self.g = g
#                 self.h = h
#                 self.f = g + h
#
#             def __eq__(self, other):
#                 return self.position == other.position
#
#             def __lt__(self, other):
#                 return self.f < other.f
#
#         def manhattan_distance(nodeA, nodeB):
#             x1, y1 = nodeA
#             x2, y2 = nodeB
#             return abs(x1 - x2) + abs(y1 - y2)
#
#         def a_star_search(grid, start, goal):
#             frontier = PriorityQueue()
#             start_node = Node(start)
#             goal_node = Node(goal)
#             frontier.put(start_node)
#             came_from = {}
#             cost_so_far = {}
#             came_from[start_node.position] = None
#             cost_so_far[start_node.position] = 0
#
#             while not frontier.empty():
#                 current = frontier.get()
#
#                 if current == goal_node:
#                     break
#
#                 for next_node in get_neighbors(grid, current.position):
#                     new_cost = cost_so_far[current.position] + 1  # Assuming each move costs 1
#                     if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
#                         cost_so_far[next_node] = new_cost
#                         priority = new_cost + manhattan_distance(next_node, goal)
#                         frontier.put(Node(next_node, current, new_cost, manhattan_distance(next_node, goal)))
#                         came_from[next_node] = current
#
#             # Reconstruct the path from the came_from dictionary
#             path = []
#             node = goal_node
#             while node != start_node:
#                 path.append(node.position)
#                 node = came_from[node.position]
#             path.append(start)
#             path.reverse()
#
#             return path
#
#         def get_neighbors(grid, position):
#             neighbors = []
#             x, y = position
#             # Define the possible movements: up, down, left, right
#             movements = [(0, 1), (0, -1), (1, 0), (-1, 0)]
#             for dx, dy in movements:
#                 new_x, new_y = x + dx, y + dy
#                 # Check if the new position is within the grid and not occupied
#                 if (0 <= new_x < len(grid) and 0 <= new_y < len(grid[0]) and grid[new_x][new_y] != 1):
#                     neighbors.append((new_x, new_y))
#             return neighbors
#
#         FoodX = state.FoodPosition.X
#         FoodY = state.FoodPosition.Y
#
#         HeadX = state.snake.HeadPosition.X  # L
#         HeadY = state.snake.HeadPosition.Y  # T
#
#         DR = FoodY - HeadY
#         DC = FoodX - HeadX
#
#         # Example usage:
#         import random
#
#         grid = [[0] * 60 for _ in range(60)]
#
#         # Define the start position
#         start = (10, 10)
#
#         # Define the goal position as a random position within the grid
#         goal = (FoodX, FoodY)
#
#         path = a_star_search(grid, start, goal)
#         print("Path:", path)
#
#         # Update goal position to the new food position
#         goal = (FoodX, FoodY)
#
#         plan = []
#
#         for i in range(len(path) - 1):
#             current_x, current_y = path[i]
#             next_x, next_y = path[i + 1]
#
#             # Determine the movement direction based on the change in x and y values
#             if next_x - current_x == 0 and next_y - current_y == -1:
#                 # Move left
#                 plan.append(0)
#             elif next_x - current_x == 1 and next_y - current_y == 0:
#                 # Move up
#                 plan.append(3)
#             elif next_x - current_x == 0 and next_y - current_y == 1:
#                 # Move right
#                 plan.append(6)
#             elif next_x - current_x == -1 and next_y - current_y == 0:
#                 # Move down
#                 plan.append(9)
#
#         print("Plan:", plan)
#
#
#         return plan

# Bfs
# from collections import deque
#
# from collections import deque
#
# class AgentSnake(Agent):
#     def SearchSolution(self, state):
#         def bfs(grid, start, goal):
#             queue = deque([start])
#             came_from = {}
#             came_from[start] = None
#
#             while queue:
#                 current = queue.popleft()
#
#                 if current == goal:
#                     break
#
#                 for next_node in get_neighbors(grid, current):
#                     if next_node not in came_from:
#                         queue.append(next_node)
#                         came_from[next_node] = current
#
#             # Reconstruct the path from the came_from dictionary
#             path = []
#             node = goal
#             while node != start:
#                 path.append(node)
#                 node = came_from[node]
#             path.append(start)
#             path.reverse()
#
#             return path
#
#         def get_neighbors(grid, position):
#             neighbors = []
#             x, y = position
#             # Define the possible movements: up, down, left, right
#             movements = [(0, 1), (0, -1), (1, 0), (-1, 0)]
#             for dx, dy in movements:
#                 new_x, new_y = x + dx, y + dy
#                 # Check if the new position is within the grid and not occupied
#                 if (0 <= new_x < len(grid) and 0 <= new_y < len(grid[0]) and grid[new_x][new_y] != 1):
#                     neighbors.append((new_x, new_y))
#             return neighbors
#
#         # Extracting positions from the state object
#         FoodX = state.FoodPosition.X
#         FoodY = state.FoodPosition.Y
#         HeadX = state.snake.HeadPosition.X
#         HeadY = state.snake.HeadPosition.Y
#
#         # Define the grid
#         grid = [[0] * 60 for _ in range(60)]
#
#         # Define the start position
#         start = (HeadX, HeadY)
#
#         # Define the goal position as the position of the food
#         goal = (FoodX, FoodY)
#
#         # Perform BFS to find the path
#         path = bfs(grid, start, goal)
#
#         # Convert the path into a sequence of directions
#         plan = []
#         for i in range(len(path) - 1):
#             current_x, current_y = path[i]
#             next_x, next_y = path[i + 1]
#
#             # Determine the movement direction based on the change in x and y values
#             if next_x - current_x == 0 and next_y - current_y == -1:
#                 # Move left
#                 plan.append(0)
#             elif next_x - current_x == 1 and next_y - current_y == 0:
#                 # Move up
#                 plan.append(3)
#             elif next_x - current_x == 0 and next_y - current_y == 1:
#                 # Move right
#                 plan.append(6)
#             elif next_x - current_x == -1 and next_y - current_y == 0:
#                 # Move down
#                 plan.append(9)
#
#         return plan

# Greedy Search
class AgentSnake(Agent):

    def SearchSolution(self, state):
        FoodX, FoodY = state.FoodPosition.X, state.FoodPosition.Y
        HeadX, HeadY = state.snake.HeadPosition.X, state.snake.HeadPosition.Y

        plan = []
        candidates = [(HeadX, HeadY - 1), (HeadX, HeadY + 1), (HeadX - 1, HeadY), (HeadX + 1, HeadY)]

        while (HeadX, HeadY) != (FoodX, FoodY):
            HX, HY = zip(*candidates)
            distance = [abs(FoodX - hx) + abs(FoodY - hy) for hx, hy in zip(HX, HY)]

            loc = distance.index(min(distance))
            move = [0, 6, 9, 3][loc]

            plan.append(move)

            HeadX, HeadY = candidates[loc]
            candidates = [(HeadX, HeadY - 1), (HeadX, HeadY + 1), (HeadX - 1, HeadY), (HeadX + 1, HeadY)]

        return plan
