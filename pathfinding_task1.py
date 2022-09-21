
from asyncio import events
from asyncio.windows_events import INFINITE
import click
from typing import Optional
from events import log
from maps import Location, Map
from parsing import validate_location, validate_map
import numpy as np
import events


def find_shortest_path(start: Location, goal: Location,
                       terrain_map: Map, terrain_threshold: int) \
        -> tuple[Optional[int], Optional[list[Location]]]:

    path = []
    frontier = [start]
    explored = set()
    path_cost = 0
    parent = {}

    # check if start or goal is below terrain threshold
    if terrain_map[goal] > terrain_threshold or terrain_map[start] > terrain_threshold:
        return None, None

    while frontier:

        # pop() lowest value state and add to explored states
        current_state = frontier.pop(
            sort_frontier(frontier, terrain_map, goal))
        events.log_visit_state(current_state, terrain_map[current_state] + h(current_state, goal, terrain_map))
        explored.add(current_state)

        # if state is goal break loop
        if current_state == goal:
            break

        # for all adjacent states
        for i in adjacent_list(current_state, terrain_map):
            # parent saves key and value pairs of states to create path once goal is found
            if i not in parent.keys() and terrain_map[i] <= terrain_threshold:
                parent[i] = current_state
            # add state to frontier if state is not already explored and is below or equal to terrain threshold
            if i not in explored and terrain_map[i] <= terrain_threshold:
                frontier.append(i)
                events.log_enqueue_state(i, terrain_map[i] + h(i, goal, terrain_map))

    # if no goal is found
    if goal not in explored:
        return None, None
    if not parent:
        return None, None

    # output path_cost and path
    return path_finder(path, goal, start, terrain_map, parent, path_cost)


def path_finder(path: list, goal, start, map, parent: dict, path_cost):
    '''path_finder builds path from the key value pairs of states in parent dict. then calculates path cost'''
    # insert goal into path list
    path.append(goal)

    while start not in path:
        # pass the last item added to path into parent as a key to return its parent state as temp
        temp = parent.get(path[-1])
        # add temp to end of path
        path.append(temp)
    # reverse path list as starting state is currently the last item
    path.reverse()
    # once path is found calculate the total path_cost 
    i = 0
    while i < len(path)-1:
        path_cost += map[path[i]] + map[path[i+1]]
        i += 1

    return path_cost, path


def adjacent_list(state: Location, terrain_map: Map):
    '''adjacent_list() finds adjacent states to a given state. returns a list of adjacent states'''
    a, b = state

    adjacent_list = list()

    north = (a-1, b)
    south = (a+1, b)
    east = (a, b+1)
    west = (a, b-1)

    # the most northern point on the map (0, x)
    if north[0] >= 0:
        adjacent_list.append(north)
    # the most southern point on 10x10 map (9, x)
    if south[0] < len(terrain_map):
        adjacent_list.append(south)
    # the most eastern point on a 10x10 map (x, 9)
    if east[1] < len(terrain_map[0]):
        adjacent_list.append(east)
    # the most western point on a 10x10 map (x, 0)
    if west[1] >= 0:
        adjacent_list.append(west)

    return adjacent_list


def sort_frontier(frontier:list, map, goal):
    '''gets the index of the lowest cost edge in the frontier. calls h function to calculate best state to explore next'''
    num = INFINITE
    index = 0
    for i in frontier:
        # f = g + h
        f = map[i] + h(i, goal, map)
        # if f(i) is less than num get index 
        if f <= num:
            num = f
            index = frontier.index(i)
        else:
            events.log_ignore_state(i, f)
    return index


def h(x, y: Location, map):
    '''
    Heuristic Estimation Function
    
    Manhattan Distance (|x1-x2| + |y1-y2|) * average terrain difficulty.

    The function is consistent because the estimate is always less than or equal to the estimated distance from any adjacent state to the goal + the cost of reaching the adjacent state.

    This heuristic was chosen because the Manhattan distance suits the movements that are permitted within the grid, vertical and horizontal.
    If the grid allowed any direction of movement, Euclidean distance would have suited better as a straight line in any direction could be a possible path.
    '''
    size = len(map) * len(map[0])
    average_terrain_difficulty = np.sum(map) / size

    x1, y1 = x
    x2, y2 = y
    h = abs(x1-x2) + abs(y1-y2)

    return h * average_terrain_difficulty


@click.command(no_args_is_help=True)
@click.argument('start', required=True, callback=validate_location)
@click.argument('goal', required=True, callback=validate_location)
@click.argument("terrain_map", required=True, type=click.Path(exists=True), callback=validate_map)
@click.argument("terrain_threshold", required=True, type=click.IntRange(min=0, max=1000))
def main(start: Location, goal: Location, terrain_map: Map, terrain_threshold: int) -> None:
    """Example usage:


    \b
    python pathfinding_task1.py 3,2 0,3 resources/terrain01.txt 50
    python pathfinding_task1.py 9,3 0,8 resources/terrain05.txt 40
    python pathfinding_task1.py 4,1 0,3 resources/terrain03.txt 50
    python pathfinding_task1.py 20,80 80,40 resources/terrain04.txt 500
    """

    path = find_shortest_path(start, goal, terrain_map, terrain_threshold)
    if path:
        log(f"The path is {path[1]} with cost {path[0]}.")
    else:
        log('No path found')


if __name__ == '__main__':
    main()
