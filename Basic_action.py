import numpy as np

def move_to_static_search_position(arm):
    q_static_search=np.array([ 0.28856799,0.20482551,0.08279217,-1.06908351,-0.01759427,1.27327017,1.14987698])
    arm.safe_move_to_position(q_static_search)
    print("Search Pos arrived!")