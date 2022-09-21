from maps import read_map
from pathfinding_task1 import find_shortest_path

def test_world_01():
    terrain_map = read_map("resources/terrain01.txt")
    cost, path = find_shortest_path((3, 2), (0, 3), terrain_map, 50)
    assert cost == 80
    assert path == [(3, 2), (3, 3), (2, 3), (1, 3), (0, 3)]


def test_world_01_impossible_start():
    terrain_map = read_map("resources/terrain01.txt")
    terrain_map[(3, 2)] = 15  # make terrain impossible to traverse at the start
    _, path = find_shortest_path((3, 2), (0, 3), terrain_map, 10)
    assert path is None


def test_world_01_impossible_goal():
    terrain_map = read_map("resources/terrain01.txt")
    terrain_map[(0, 3)] = 15  # make terrain impossible to traverse at the goal
    _, path = find_shortest_path((3, 2), (0, 3), terrain_map, 10)
    assert path is None


def test_world_03():
    terrain_map = read_map("resources/terrain03.txt")
    cost, path = find_shortest_path((4, 1), (0, 3), terrain_map, 50)
    assert cost == 128
    assert path == [(4, 1), (4, 2), (3, 2), (2, 2), (1, 2), (0, 2), (0, 3)]


def test_world_03_impossible_terrain():
    terrain_map = read_map("resources/terrain03.txt")
    _, path = find_shortest_path((4, 1), (0, 3), terrain_map, 28)
    assert path is None


def test_world_04_large():
    terrain_map = read_map("resources/terrain04.txt")
    cost, path = find_shortest_path((20, 80), (80, 40), terrain_map, 500)
    assert cost == 24024
    assert path == [ (20, 80), (20, 79), (20, 78), (20, 77), (20, 76), (20, 75), (20, 74), (20, 73), (20, 72), (20, 71), (20, 70),
                     (20, 69), (20, 68), (21, 68), (21, 67), (22, 67), (22, 66), (22, 65), (23, 65), (24, 65), (24, 64), (24, 63), (25, 63), (25, 62), (26, 62), (27, 62), (27, 61), (28, 61), (29, 61), (29, 60), (30, 60), (30, 59), (31, 59), (32, 59), (32, 58), (33, 58), (34, 58), (35, 58), (35, 57), (36, 57), (36, 56), (37, 56), (38, 56), (39, 56), (40, 56), (40, 55), (41, 55), (41, 54), (42, 54), (43, 54), (44, 54), (45, 54), (45, 53), (45, 52), (46, 52), (47, 52), (47, 51), (48, 51), (48, 50), (49, 50), (49, 49), (49, 48), (49, 47), (50, 47), (51, 47), (51, 46), (52, 46), (52, 45), (52, 44), (53, 44), (53, 43), (54, 43), (54, 42), (55, 42), (55, 41), (55, 40), (56, 40), (57, 40), (58, 40), (59, 40), (60, 40), (61, 40), (62, 40), (63, 40), (64, 40), (65, 40), (66, 40), (67, 40), (68, 40), (69, 40), (70, 40), (71, 40), (72, 40), (73, 40), (74, 40), (75, 40), (76, 40), (77, 40), (78, 40), (79, 40), (80, 40) ]


def test_world_05():
    terrain_map = read_map("resources/terrain05.txt")
    cost, path = find_shortest_path((9, 3), (0, 8), terrain_map, 40)
    assert cost == 615
    assert path == [ (9, 3), (8, 3), (8, 4), (7, 4), (7, 5), (6, 5), (6, 6), (5, 6), (5, 7), (5, 8), 
                     (4, 8), (3, 8), (2, 8), (1, 8), (0, 8) ]
