import click
from typing import Optional
from events import log
from maps import Location, Map
from parsing import validate_location, validate_map
import events
from asyncio.windows_events import INFINITE
import numpy as np
def find_shortest_safe_path(start: Location, goal: Location, 
                            terrain_map: Map, terrain_threshold: int,
                            success_map: Map, success_threshold: float) \
                            -> tuple[Optional[int],Optional[float],Optional[list[Location]]]:
    """Finds the path with lowest total cost that also satisfies 
       the minimum success probability threshold (Task 2).
       Returns (cost,prob_success,list(locations)) when a path is found.
       Returns (None,None,None) if no path is found."""

    # This is the entry point for your code for Task 2.
    # Please create additional functions and classes etc as needed 
    # to structure your implementation. 
    # Avoid implementing the entire algorithm in one long chunk.
    path = []
    frontier = [start]
    explored = set()
    path_cost = 0
    prob_success = 1
    parent = {}
    current_path = 0
    

    # check if start or goal is below terrain threshold
    if terrain_map[goal] > terrain_threshold or terrain_map[start] > terrain_threshold:
        return None, None, None

    while frontier:

        try:
            i, current_path = sort_frontier(frontier, terrain_map, success_map, success_threshold, parent, start, goal, current_path)
            # pop() lowest value state and add to explored states
            current_state = frontier.pop(i)
            events.log_visit_state(current_state, current_path)
            explored.add(current_state)
        except:
            if frontier:
                continue
            if path:
                print("47 50 = ",h((47,50),goal, terrain_map))
                print("48 51 = ",h((48,51),goal, terrain_map))
                return path_cost, prob_success, path
            else:
                return None, None, None

        
        # if state is goal break loop
        if current_state == goal:

            if path and current_path < path_cost:
                path_cost, prob_success, path = path_finder(path, terrain_map, start, goal, parent, path_cost, success_map, prob_success)
                break
            elif not path:
                path_cost, prob_success, path = path_finder(path, terrain_map, start, goal, parent, path_cost, success_map, prob_success)
            else:
                break
            if not path:
                continue
            if prob_success < success_threshold:
                continue
            

 

        # for all adjacent states
        for i in adjacent_list(current_state, terrain_map):
            
        # add state to frontier if state is not already explored and is below or equal to terrain threshold
            if i not in explored and i not in frontier:
                if terrain_map[i] <= terrain_threshold:
                    frontier.append(i)
                    parent[i] = current_state
                    events.log_enqueue_state(i, terrain_map[i] + h(i, goal, terrain_map))

            elif not frontier and parent[current_state] != i or path and parent[current_state] != i:
                parent[i] = current_state
                if i in explored:
                    explored.remove(i)
                    frontier.append(i)
                    events.log_enqueue_state(i, terrain_map[i] + h(i, goal, terrain_map))
            else:
                events.log_ignore_state(i, path_cost, prob_success)


    if prob_success < success_threshold:
        return None, None, None

    # output path_cost and path
    return path_cost, prob_success, path


def path_finder(path: list,map, start, state, parent: dict, path_cost, enemies, prob_success):
    '''path_finder calculates path cost and risk factor'''
    path.clear()
    prob_success = 1
    path_cost = 0
    path.append(state)

    while start not in path:
        
        temp = parent.get(path[-1])
        if temp not in path:
            path.append(temp)
        else:
            break
    
    path.reverse()

    i = 0
    if path:
        while i < len(path)-1:
            path_cost += map[path[i]] + map[path[i+1]]
            prob_success *= 1-enemies[path[i]] / 100
            i += 1


    return path_cost, prob_success, path


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


def sort_frontier(frontier:list, map, enemies, success, parent:dict, start, goal, current_path_cost):
    '''gets the index of the lowest cost edge in the frontier. calls h function to calculate best state to explore next'''
    num = INFINITE
    success_num = 0
    possible_path = []
    current_path_cost = 0
    prob_success = 1
    index = 0
    lowest_path_cost = 0
    next_state = frontier[index]

    counter = 0
    l = len(frontier)
    for i in frontier:
        f = map[i] + h(i, goal, map)
        possible_path.clear()
        current_path_cost = 0
        path_cost = 0
        prob_success = 1
        possible_path.append(i)
        counter+=1

        while start not in possible_path:
            temp = parent.get(possible_path[-1])
            if temp not in possible_path:
                possible_path.append(temp)
            else:
                frontier.remove(i)
                if counter == len(frontier):
                    return index, current_path_cost
                

        
        possible_path.reverse()
        j = 0
    
        while j < len(possible_path)-1:
            current_path_cost += map[possible_path[j]] + map[possible_path[j+1]]
            path_cost += map[possible_path[j]] + map[possible_path[j+1]] + f
            prob_success *= 1-enemies[possible_path[j]] / 100
            j += 1


        if prob_success < success:
            frontier.remove(i)
            
        if path_cost < num and prob_success > success:
            success_num = prob_success
            num = path_cost
            lowest_path_cost = current_path_cost
            index = frontier.index(i)

        elif path_cost == num and prob_success > success_num:
            success_num = prob_success
            num = path_cost
            lowest_path_cost = current_path_cost
            index = frontier.index(i)
        else:
            continue

    

    return index, current_path_cost


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
@click.argument("terrain_threshold", required=True, type=click.IntRange(min=0,max=1000))
@click.argument("success_map", required=True, type=click.Path(exists=True), callback=validate_map)
@click.argument("success_threshold", required=True, type=click.FloatRange(min=0.0,max=1.0))
def main(start: Location, goal: Location, 
         terrain_map: Map, success_map: Map, 
         terrain_threshold: int, success_threshold: float) -> None:
    """Example usage:

        \b
        python safe_pathfinding_task2.py 3,2 0,3 resources/terrain01.txt 50 resources/enemy01.txt 1.0
        python safe_pathfinding_task2.py 3,2 0,3 resources/terrain01.txt 50 resources/enemy01.txt 1.0
        python safe_pathfinding_task2.py 3,3 0,3 resources/terrain02.txt 50 resources/enemy02.txt 0.6
        python safe_pathfinding_task2.py 3,2 0,3 resources/terrain01.txt 50 resources/enemy01.txt 0.2
        python safe_pathfinding_task2.py 20,80 80,40 resources/terrain04.txt 200 resources/enemy04.txt 0.5
        python safe_pathfinding_task2.py 4,1 0,3 resources/terrain03.txt 50 resources/enemy03.txt 0.5
    """
    path = find_shortest_safe_path(start, goal, terrain_map, terrain_threshold, success_map, success_threshold)
    if path:
        log(f"The path is {path[2]} with cost {path[0]} and success probability {path[1]}")
    else:
        log('No path found')

if __name__ == '__main__':
    main()
