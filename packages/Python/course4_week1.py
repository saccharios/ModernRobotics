import csv
import pathlib
from typing import List, Optional, Dict

import pip
from dataclasses import dataclass

import numpy as np


class Graph:
    def __init__(self):
        self.edges: Dict[int, List[int]] = {}

    def neighbors(self, id_: int) -> List[int]:
        return self.edges[id_]


class Data:
    def __init__(self) -> None:
        self.heuristics: Dict[int,float] = {}
        self.costs: Dict[(int,int),float] = {}
        self.graph: Graph = Graph()


def read_in_graph(nodes: pathlib.Path, edges: pathlib.Path) -> Data:
    data = Data()
    # read in nodes
    with open(nodes) as node_file:
        csvreader = csv.reader(node_file)
        for row in csvreader:
            node_id=int(row[0])
            heuristic=float(row[3])
            data.graph.edges[node_id] = []
            data.heuristics[node_id] = heuristic
    # read in edges and assigned to correct node
    with open(edges) as edges_file:
        csvreader = csv.reader(edges_file)
        for row in csvreader:
            node_1 = int(row[0])
            node_2 = int(row[1])
            cost = float(row[2])
            data.graph.edges[node_1].append(node_2)
            data.graph.edges[node_2].append(node_1)
            data.costs[(node_1,node_2)] = cost
            data.costs[(node_2,node_1)] = cost

    return data


def write_to_csv(path: List[int], file_name: pathlib.Path) -> None:
    with open(file_name, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerows([path])


def find_path(end_node_id:int, start_node_id:int, parents:Dict[int,int]) -> List[int]:
    path: List[int] = [end_node_id]
    current = end_node_id
    while current != start_node_id:
        path.append(parents[current])
        current = parents[current]
    
    path.reverse()
    return path


def run_a_star(data: Data, start_node_id: int, end_node_id: int) -> List[int]:
    # Init algorithm
    open: List[int] = [start_node_id]
    past_cost: Dict[int, float] = {}
    for k,v in data.graph.edges.items():
        past_cost[k] = 99999999

    past_cost[start_node_id] = 0.0
    estimated_total_cost: Dict[int,float] = {}
    closed: List[int] = []
    parents: Dict[int,int] = {}

    # Start algorithm
    while len(open) > 0:
        current = open.pop(0)
        closed.append(current)
        if current == end_node_id:
            print("SUCCESS")
            return find_path(end_node_id, start_node_id, parents)
        for neighbor in data.graph.edges[current]:
            if neighbor in closed:
                continue
            tentative_past_cost = past_cost[current] + data.costs[current, neighbor]
            if tentative_past_cost < past_cost[neighbor]:
                past_cost[neighbor] = tentative_past_cost
                parents[neighbor] = current
                # Add neighbor at the correct location in open
                estimated_total_cost[neighbor] = past_cost[neighbor] + data.heuristics[neighbor]
                idx = [index for index,value in enumerate(open) if estimated_total_cost[value] > estimated_total_cost[neighbor]]
                if len(idx) == 0:
                    open.append(neighbor)
                else:
                    open.insert(idx[0],neighbor)
    print("FAILURE")
    return []


if __name__ == '__main__':
    data = read_in_graph('/home/sv/bags/modern-robotics/packages/Python/Scene5_example/nodes.csv',
                          '/home/sv/bags/modern-robotics/packages/Python/Scene5_example/edges.csv')

    optimal_path = run_a_star(data, 1, 12)
    print(f"optimal_path = {optimal_path}")
    write_to_csv(optimal_path,'/home/sv/bags/modern-robotics/packages/Python/Scene5_example/path.csv')



