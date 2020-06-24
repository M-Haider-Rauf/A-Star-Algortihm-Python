import pygame
import math
from collections import deque
from constants import *

import random


class Node:
    def __init__(self, x=0, y=0):
        self.x = x  # since we're using a grid...
        self.y = y
        self.parent = None
        self.g = float("inf")   # by default g and f == infinity
        self.f = float("inf")
        self.obstacle = False
        self.neighbours = []

    def distance_sqrd(self, node):
        x = node.x - self.x
        y = node.y - self.y
        return (x * x) + (y * y)

    def distance(self, node):
        return math.sqrt(self.distance_sqrd(node))

    def manhattan_distance(self, node):  # it's the sum of absolute differences of the x and y-axes
        return abs(node.x - self.x) + abs(node.y - self.y)


class Engine:
    mouse_pos = (0, 0)  # store prev mouse position globally

    def __init__(self):
        # set up pygame
        pygame.init()
        self.window: pygame.Surface = pygame.display.set_mode((WIN_WIDTH, WIN_HEIGHT))
        pygame.display.set_caption("A* path finding")
        self.clock = pygame.time.Clock()
        self.running = True

        # set up grid and connections
        self.grid = [[Node(x, y) for x in range(GRID_WIDTH)] for y in range(GRID_HEIGHT)]
        self.start_node = None
        self.end_node = None
        self.set_connections(diagonal=False)
        self.set_end(GRID_WIDTH - 1, GRID_HEIGHT - 1)
        self.set_start(0, 0)
        self.path = []

        # run the a* algorithm
        self.randomize_obstacles()
        self.solve_astar()

    def handle_input(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:  # ESCAPE key quits
                    self.running = not self.running
                elif event.key == pygame.K_SPACE:  # clear all obstacles if SPACE is pressed
                    for y in range(GRID_HEIGHT):
                        for x in range(GRID_WIDTH):
                            self.grid[y][x].obstacle = False
                elif event.key == pygame.K_r:  # generate random obstacles
                    self.randomize_obstacles()
                self.solve_astar()

            elif event.type == pygame.MOUSEBUTTONUP:
                if event.button == 1:
                    released = True

        if pygame.mouse.get_pressed()[0]:  # On press of left mouse button
            x, y = pygame.mouse.get_pos()
            # get grid mouse is currently over
            x //= CELL_SIZE
            y //= CELL_SIZE
            x %= GRID_WIDTH
            y %= GRID_HEIGHT

            if Engine.mouse_pos != (x, y):
                key_states = pygame.key.get_pressed()
                if key_states[pygame.K_LCTRL]:
                    self.set_start(x, y)
                elif key_states[pygame.K_LSHIFT]:
                    self.set_end(x, y)
                else:
                    self.grid[y][x].obstacle = not self.grid[y][x].obstacle

            Engine.mouse_pos = (x, y)
            self.solve_astar()

    def update(self):
        self.clock.tick(30)

    def render(self):
        self.window.fill(CLR_BLACK)
        # !begin drawing
        self.draw_cells()
        self.draw_path()
        # self.draw_lines()

        # !end drawing
        pygame.display.flip()

    def main_loop(self):
        while self.running:
            self.handle_input()
            self.update()
            self.render()

    def draw_cells(self): # draw cell in different color based on status
        for y in range(GRID_HEIGHT):
            for x in range(GRID_WIDTH):
                node = self.grid[y][x]
                color = None
                if node is self.start_node:
                    color = CLR_GREEN
                elif node is self.end_node:
                    color = CLR_RED
                elif node.obstacle:
                    color = CLR_GRAY
                else:
                    color = CLR_BLACK

                self.window.fill(color, (x * CELL_SIZE, y * CELL_SIZE, CELL_SIZE, CELL_SIZE))

    # I use the Manhattan distance between two points as the Heuristic
    def heuristic(self, node):
        return node.manhattan_distance(self.end_node)
        # return node.distance(self.end_node)

    def set_end(self, x, y):
        self.end_node = self.grid[y][x]
        self.end_node.obstacle = False  # end node can never be obstacle

    def set_start(self, x, y):
        self.start_node = self.grid[y][x]
        self.start_node.g = 0.0
        self.start_node.f = self.heuristic(self.end_node)
        self.start_node.obstacle = False  # start node can never be obstacle

    # set neighbours of each node
    # by default it's von neumann neighbourhood (4-way)
    # if diagonal == TRUE, then it's moor neighbourhood (8-way)
    def set_connections(self, diagonal=False):
        for y in range(GRID_HEIGHT):
            for x in range(GRID_WIDTH):
                node = self.grid[y][x]
                if x > 0:
                    node.neighbours.append(self.grid[y][x - 1])
                    if diagonal and y > 0:
                        node.neighbours.append(self.grid[y - 1][x - 1])
                if x < GRID_WIDTH - 1:
                    node.neighbours.append(self.grid[y][x + 1])
                    if diagonal and y < GRID_HEIGHT - 1:
                        node.neighbours.append(self.grid[y + 1][x + 1])
                if y > 0:
                    node.neighbours.append(self.grid[y - 1][x])
                    if diagonal and x < GRID_WIDTH - 1:
                        node.neighbours.append(self.grid[y - 1][x + 1])
                if y < GRID_HEIGHT - 1:
                    node.neighbours.append(self.grid[y + 1][x])
                    if diagonal and x > 0:
                        node.neighbours.append(self.grid[y + 1][x - 1])

    def solve_astar(self):
        self.reset()  # reset every node status
        nodes_to_test = deque([self.start_node])  # deque with only start node (open list)
        visited = set()  # nodes marked visited (closed list)

        while nodes_to_test:
            # sort list by f value
            nodes_to_test = sorted(nodes_to_test, key=lambda node: node.f)

            # get rid of visited nodes
            while nodes_to_test and nodes_to_test[0] in visited:
                nodes_to_test.pop(0)

            if not nodes_to_test:
                break

            # get node at front (it has least f value) and mark it visited
            current_node = nodes_to_test.pop(0)
            visited.add(current_node)
            
            # if target node found, break
            if current_node is self.end_node:
                break
            # the a* behaviour...
            for neighbour in current_node.neighbours:
                if not neighbour.obstacle and neighbour not in visited:
                    nodes_to_test.append(neighbour)

                if not neighbour.obstacle:
                    distance = current_node.distance(neighbour)
                    if current_node.g + distance < neighbour.g:
                        neighbour.g = distance + current_node.g
                        neighbour.f = neighbour.g + self.heuristic(neighbour)
                        neighbour.parent = current_node

        # trace path, following backwards from end node
        node = self.end_node
        while node:
            self.path.append((node.x * CELL_SIZE + CELL_SIZE // 2, node.y * CELL_SIZE + CELL_SIZE // 2))
            node = node.parent

    def draw_path(self):
        if len(self.path) > 1:
            pygame.draw.lines(self.window, CLR_YELLOW, False, self.path)

    def reset(self):
        self.path.clear()
        for y in range(GRID_HEIGHT):
            for x in range(GRID_WIDTH):
                self.grid[y][x].parent = None
                self.grid[y][x].g = float("inf")
                self.grid[y][x].f = float("inf")
                self.grid[y][x].visited = False

        self.start_node.f = self.heuristic(self.start_node)
        self.start_node.obstacle = False
        self.end_node.obstacle = False
        self.start_node.g = 0.0

    def randomize_obstacles(self):
        for y in range(GRID_HEIGHT):
            for x in range(GRID_WIDTH):
                # 1 in 7 chance of being an obstacle, hence 14% obstacles...
                n = random.randint(0, 6)  
                if n:
                    self.grid[y][x].obstacle = False
                else:
                    self.grid[y][x].obstacle = True


def main():
    engine = Engine()
    engine.main_loop()
    return 0


if __name__ == "__main__":
    main()
