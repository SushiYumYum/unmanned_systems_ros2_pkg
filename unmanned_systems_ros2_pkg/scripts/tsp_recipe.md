# TSP Example
'''
cost_list = []
for loop to go through each permutations:
    - Set cost = 0
    - for loop to go to each inidual waypoint
        - current cost = compute distance(wp_curr,wp_next)
        - cost = cost + current_cost
    - cost_list.append(cost)

min_index = fin_min_index(cost_list)
best_solution = list(permutations[min_index])

'''