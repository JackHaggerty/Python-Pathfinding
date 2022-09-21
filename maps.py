import numpy as np

Location = tuple[int,int]
Map = np.ndarray

def read_map(file_name: str) -> Map:
    return np.loadtxt(file_name, dtype=int)
