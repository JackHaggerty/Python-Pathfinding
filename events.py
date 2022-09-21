from maps import Location

def log(message: str) -> None:
    print(message)

def log_visit_state(location: Location, cost: int, p_success: float=1.0) -> None:
    """Records that a state has been visited by the search algorithm.
    Call this function whenever you remove the best state from the frontier and process it.

    :param location: the location on the map that is visited. 
                     A tuple (row,col) holding the coordinates of the location.
                     The top left corner is (0,0).
    :param cost: the total estimated path cost (f-cost) from the start state to the goal state via location
    :param p_success: the success probability (float in the range [0.0...1.0]) associated with 
                      the current path. Optional for Question 1.
    """
    log(f'VISITED {location} cost={cost} success={p_success:.4f}')


def log_enqueue_state(location: Location, cost: int, p_success: float=1.0) -> None:
    """Records that a state has been added to the frontier.
    Call this function whenever you add a state to the frontier.
    
    :param location: the location on the map that is added. 
                     A tuple (row,col) holding the coordinates of the location. 
                     The top left corner is (0,0).
    :param cost: the total estimated path cost (f-cost) from the start state to the goal state via location
    :param p_success: the success probability (float in the range [0.0...1.0]) associated with 
                      the current path. Optional for Question 1.
    """
    log(f'+ ENQUEUED {location} cost={cost} success={p_success:.4f}')


def log_ignore_state(location: Location, cost: int, p_success: float=1.0) -> None:
    """Records that a state has been found that is strictly worse than one encountered previously.
    Hence, this state is ignored.    
    Call this function whenever you generate a path to a location that is not added to the frontier.

    :param location: the location on the map that is visited. 
                     A tuple (row,col) holding the coordinates of the location.
                     The top left corner is (0,0).
    :param cost: the total estimated path cost (f-cost) from the start state to the goal state via location
    :param p_success: the success probability (float in the range [0.0...1.0]) associated with 
                      the current path. Optional for Question 1.
    """
    log(f'+ IGNORED {location} cost={cost} success={p_success:.4f}')
