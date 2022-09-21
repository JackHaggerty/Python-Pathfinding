import math
from maps import read_map
from safe_pathfinding_task2 import find_shortest_safe_path

def test_world_01_enemy_a():
    terrain_map = read_map("resources/terrain01.txt")
    enemy_map = read_map("resources/enemy01.txt")
    cost, prob_success, path = find_shortest_safe_path((3, 2), (0, 3), terrain_map, 50, enemy_map, 1.0)
    assert cost == 300
    assert prob_success == 1.0
    assert path == [(3,2),(3,1),(3,0),(2,0),(1,0),(1,1),(0,1),(0,2),(0,3)]

def test_world_01_enemy_b():
    terrain_map = read_map("resources/terrain01.txt")
    enemy_map = read_map("resources/enemy01.txt")
    cost, prob_success, path = find_shortest_safe_path((3, 2), (0, 3), terrain_map, 50, enemy_map, 1.0)
    assert cost == 300
    assert prob_success == 1.0
    assert path == [(3,2),(3,1),(3,0),(2,0),(1,0),(1,1),(0,1),(0,2),(0,3)]

def test_world_01_enemy_c():
    terrain_map = read_map("resources/terrain01.txt")
    enemy_map = read_map("resources/enemy01.txt")
    cost, prob_success, path = find_shortest_safe_path((3, 2), (0, 3), terrain_map, 50, enemy_map, 0.2)
    assert cost == 80
    assert prob_success == 0.25
    assert path == [(3,2),(3,3),(2,3),(1,3),(0,3)]

def test_world_01_enemy_d():
    terrain_map = read_map("resources/terrain01.txt")
    enemy_map = read_map("resources/enemy01.txt")
    cost, prob_success, path = find_shortest_safe_path((3, 2), (0, 3), terrain_map, 10, enemy_map, 0.3)
    assert path is None

def test_world_02_enemy():
    terrain_map = read_map("resources/terrain02.txt")
    enemy_map = read_map("resources/enemy02.txt")
    cost, prob_success, path = find_shortest_safe_path((3, 3), (0, 3), terrain_map, 50, enemy_map, 0.6)
    assert cost == 240
    assert math.isclose(prob_success, 0.72, rel_tol=1e-5)
    assert path == [(3,3),(3,2),(3,1),(3,0),(2,0),(1,0),(1,1),(1,2),(0,2),(0,3)]

def test_world_03_enemy():
    terrain_map = read_map("resources/terrain03.txt")
    enemy_map = read_map("resources/enemy03.txt")
    cost, prob_success, path = find_shortest_safe_path((4, 1), (0, 3), terrain_map, 50, enemy_map, 0.5)
    assert cost == 128
    assert math.isclose(prob_success,0.648,rel_tol=1e-5)
    assert path == [(4,1),(4,2),(3,2),(2,2),(1,2),(0,2),(0,3)]

def test_world_03_enemy_highthreshold():
    terrain_map = read_map("resources/terrain03.txt")
    enemy_map = read_map("resources/enemy03.txt")
    cost, prob_success, path = find_shortest_safe_path((4, 1), (0, 3), terrain_map, 50, enemy_map, 0.9)
    assert cost == 168
    assert prob_success == 1.0
    assert path == [(4,1),(3,1),(2,1),(1,1),(0,1),(0,2),(0,3)]

def test_world_04_enemy():
    terrain_map = read_map("resources/terrain04.txt")
    enemy_map = read_map("resources/enemy04.txt")
    cost, prob_success, path = find_shortest_safe_path((20, 80), (80, 40), terrain_map, 200, enemy_map, 0.5)
    assert cost == 24836
    assert math.isclose(prob_success,0.51334208,rel_tol=1e-5)
    assert path == [(20,80),(20,79),(20,78),(20,77),(20,76),(20,75),(20,74),(20,73),(20,72),(20,71),(20,70),(20,69),
                    (20,68),(21,68),(21,67),(22,67),(22,66),(22,65),(23,65),(24,65),(24,64),(24,63),(25,63),(25,62),
                    (26,62),(27,62),(27,61),(28,61),(29,61),(29,60),(30,60),(30,59),(31,59),(32,59),(32,58),(33,58),
                    (34,58),(35,58),(35,57),(36,57),(36,56),(37,56),(38,56),(39,56),(40,56),(40,55),(41,55),(41,54),
                    (42,54),(43,54),(44,54),(45,54),(45,53),(45,52),(46,52),(47,52),(47,51),(47,50),(47,49),(47,48),
                    (47,47),(48,47),(48,46),(48,45),(48,44),(48,43),(49,43),(50,43),(50,42),(50,41),(51,41),(51,40),
                    (51,39),(52,39),(53,39),(53,38),(54,38),(55,38),(56,38),(57,38),(58,38),(59,38),(60,38),(61,38),
                    (62,38),(63,38),(64,38),(65,38),(66,38),(67,38),(68,38),(69,38),(70,38),(70,39),(70,40),(71,40),
                    (72,40),(73,40),(74,40),(75,40),(76,40),(77,40),(78,40),(79,40),(80,40)]
