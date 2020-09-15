def find_intermediate_positions(init_pos, final_pos):
    curr_pos = list(init_pos)
    increment = [0] * len(init_pos)
    intermediate_positions = []    
    for idx in range(0, len(init_pos)):
        increment[idx] = (final_pos[idx] - init_pos[idx]) / float(4)
    for n in range(0, 4):
        for idx in range(0, len(init_pos)):
            curr_pos[idx] += increment[idx]
        intermediate_positions.append(list(curr_pos))    
    return intermediate_positions

print(find_intermediate_positions([1,2,3],[4,5,100]))

import IK_lib
print(IK_lib.cartesian_to_angles([100,100,100],175,175))