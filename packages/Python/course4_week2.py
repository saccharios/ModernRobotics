import csv
import math
import os
import pathlib
import random
from math import sqrt
from typing import List, Dict, Any, Tuple

from anytree import Node, RenderTree, PreOrderIter, Walker

# pip3 install anytree

class Point:
    def __init__(self, x: float, y: float) -> None:
        self.x = x
        self.y = y

    def __str__(self) -> str:
        return f"Point(x: {self.x}, y: {self.y})"

    def __repr__(self) -> str:
        return self.__str__()

    def distance_to(self, point: Any) -> float:
        return sqrt((self.x - point.x) * (self.x - point.x) + (self.y - point.y) * (self.y - point.y))


class Line:
    def __init__(self, start: Point, end: Point) -> None:
        self.start = start
        self.end = end

    def length(self) -> float:
        return self.start.distance_to(self.end)

    def __str__(self) -> str:
        return f"Line(start: {self.start}, end: {self.end})"

    def __repr__(self) -> str:
        return self.__str__()


class Disk:
    def __init__(self, center: Point, radius: float) -> None:
        self.center = center
        self.radius = radius

    def __str__(self) -> str:
        return f"Disk(center: {self.center}, radius: {self.radius})"

    def __repr__(self) -> str:
        return self.__str__()

    def contains(self, point: Point) -> bool:
        return self.center.distance_to(point) <= self.radius

    def intersects(self, line: Line) -> bool:
        if self.contains(line.end) or self.contains(line.start):
            return True
        # Based on the fact that a + lambda * (b-a) + mu * (b-a)_rot_90 = r
        det_A = -(line.end.x - line.start.x) * (line.end.x - line.start.x) - (line.end.y - line.start.y) * (
                    line.end.y - line.start.y)
        lambd = (-(line.end.x - line.start.x) * (self.center.x - line.start.x) - (line.end.y - line.start.y) * (
                    self.center.y - line.start.y)) / det_A
        mu = (-(line.end.y - line.start.y) * (self.center.x - line.start.x) + (line.end.x - line.start.x) * (
                    self.center.y - line.start.y)) / det_A
        if 0.0 <= lambd <= 1.0:
            # evaluate distance of orthogonal distance
            orth_dist = math.fabs(mu) * line.length()
            return orth_dist <= self.radius
        else:
            return False


def read_in_obstacles(obstacles_file: pathlib.Path) -> List[Disk]:
    disks: List[Disk] = []
    with open(obstacles_file) as f:
        csvreader = csv.reader(f)
        for row in csvreader:
            if '#' in row[0]:
                continue
            x = float(row[0])
            y = float(row[1])
            r = float(row[2]) / 2.0
            disks.append(Disk(center=Point(x=x, y=y), radius=r))
    return disks


def print_tree(root: Node) -> None:
    for pre, fill, node in RenderTree(root):
        print("%s%s" % (pre, node.name))


def get_sample(start: Point, end: Point) -> Point:
    get_sample.counter += 1
    if get_sample.counter % 10 == 0:
        get_sample.counter = 0
        return end
    x_rand = random.uniform(start.x, end.x)
    y_rand = random.uniform(start.y, end.y)
    return Point(x_rand, y_rand)


get_sample.counter = 0


def find_nearest(root: Node, point_sample: Point) -> Node:
    nearest_node = Point(-1, -1)
    smallest_distance = 99999
    for node in PreOrderIter(root):
        distance = point_sample.distance_to(node.name)
        if distance < smallest_distance:
            smallest_distance = distance
            nearest_node = node
    return nearest_node


def is_collision_free(obstacles: List[Disk], line: Line) -> bool:
    for disk in obstacles:
        if disk.intersects(line):
            return False
    return True


def run_RRT(obstacles: List[Disk], start: Point, end: Point) -> Tuple[bool, Node]:
    root = Node(start)
    MAX_TREE_SIZE = 11
    while root.height <= MAX_TREE_SIZE:
        point_sample = get_sample(start, end)
        node_nearest = find_nearest(root, point_sample)
        if is_collision_free(obstacles, Line(node_nearest.name, point_sample)):
            Node(point_sample, parent=node_nearest)
            if point_sample == end:
                return True, root
    return False, root


def write_nodes_to_csv(nodes_dict: Dict[Point, int], end: Point, file_path: pathlib.Path) -> None:
    os.remove(file_path.as_posix())
    with open(file_path, 'a', newline='') as f:
        writer = csv.writer(f)
        for point, id_ in nodes_dict.items():
            distance = end.distance_to(point)
            writer.writerow([id_, point.x, point.y, distance])


def write_edges_to_csv(root: Node, nodes_dict: Dict[Point, int], file_path: pathlib.Path) -> None:
    edges: Dict[(int, int), float] = {}
    # Find edges with cost
    for node in PreOrderIter(root):
        for child in node.children:
            node_i = nodes_dict[node.name]
            node_j = nodes_dict[child.name]
            distance = node.name.distance_to(child.name)
            edges[node_i, node_j] = distance
    # Remove and write to csv
    os.remove(file_path.as_posix())
    with open(file_path, 'a', newline='') as f:
        writer = csv.writer(f)
        for edge, cost in edges.items():
            writer.writerow([edge[0], edge[1], cost])


def write_path_to_csv(nodes_dict: Dict[Point, int], root: Node, end: Point, file_path: pathlib.Path) -> None:
    if end not in nodes_dict:
        nodes_dict[end] = 1
    # go backwards from end
    path: List[int] = [nodes_dict[end]]
    for node in PreOrderIter(root):
        if node.name == end:
            while node.parent is not None:
                path.append(nodes_dict[node.parent.name])
                node = node.parent
    path.reverse()
    # Remove and write to csv
    os.remove(file_path.as_posix())
    with open(file_path, 'a', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(path)


def write_to_csv(root: Node, base_path: pathlib.Path, end: Point) -> None:
    # Write the content of the tree into csv files, edges.csv nodes.csv, path.csv
    # Give integer-ids to nodes
    nodes_dict: Dict[Point, int] = {}
    counter = 1
    for node in PreOrderIter(root):
        nodes_dict[node.name] = counter
        counter += 1
    write_nodes_to_csv(nodes_dict, end, base_path / 'nodes.csv')
    write_edges_to_csv(root, nodes_dict, base_path / 'edges.csv')
    write_path_to_csv(nodes_dict, root, end, base_path / 'path.csv')


if __name__ == '__main__':
    obstacles = read_in_obstacles(
        pathlib.Path('/home/sv/bags/modern-robotics/packages/Python/results_course4_week2/obstacles.csv'))
    start = Point(x=-0.5, y=-0.5)
    end = Point(x=0.5, y=0.5)
    success, root = run_RRT(obstacles, start, end)
    write_to_csv(root, pathlib.Path('/home/sv/bags/modern-robotics/packages/Python/results_course4_week2'), end)
    if success:
        print("Success")
        print_tree(root)
    else:
        print("Failure")
