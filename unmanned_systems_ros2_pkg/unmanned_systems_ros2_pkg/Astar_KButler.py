# -*- coding: utf-8 -*-
"""
Created on Tue Sep 12 22:24:29 2023

@author: Kaitie Butler

Hwk 3, P1 Working - Now Astar Template

Goal: To modify Djikstra's code so that it is A*. 
    - Add Heuristic cost to total cost equation
    - Show x and y plot
"""
import numpy as np
import math as m
import matplotlib.pyplot as plt

# Initialize Parameters

start_x = 1
start_y = 1

#current_x = 0     # Originally: curr_x and curr_y
#current_y = 10

min_x = 0 
max_x = 15

min_y = 0 
max_y = 15

gs = 1

goal_x = 7
goal_y = 13

# obstacle_positions =  [(2,2), (2,3), (2,4), (5,5), (5,6), (6,6), (7,3),
                       #(7,4), (7,5), (7,6), (8,6)]
def run_astar(min_x:float, max_x:float, min_y:float, max_y:float, gs:float, goal_x:float, goal_y:float):

    def merge(x,y):
        merged_list = [(x[i],y[i]) for i in range(0, len(x))]
        return merged_list

    Obstacle_x = [ 2, 2, 2, 2, 0, 1, 2, 3, 4, 5, 5, 5, 5, 5, 8, 9, 10, 11, 12, 13, 8, 8, 8,
    8, 8, 8, 8, 2, 3, 4, 5, 6, 7, 9, 10, 11, 12, 13, 14, 15, 2, 2, 2, 2, 2, 2, 5, 5, 5, 5, 5,
    5, 5, 6, 7, 8, 9, 10, 11, 12, 12, 12, 12, 12]

    Obstacle_y = [ 2, 3, 4, 5, 5, 5, 5, 5, 5, 5, 2, 3, 4, 5, 2, 2, 2, 2, 2, 2, 3, 4, 5, 6, 7,
    8, 9, 7, 7, 7, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 8, 9, 10, 11, 12, 13, 9, 10, 11, 12, 13,
    14, 15, 12, 12, 12, 12, 12, 12, 8, 9, 10, 11, 12] 

    obstacle_positions = merge(Obstacle_x, Obstacle_y)

    #coord = []

    #for a, b in zip( Obstacle_x, Obstacle_y ):
        #coord.append( [ a, b ] )

    #print( coord )

    obstacle_list = [] # store obstacle classes
    obstacle_radius = 0.5

    # Creating Classes for (1) Node and (2) Obstacle

    # Class used for defining nodes and node params
    class Node():
        def __init__(self, x:float, y:float, cost:float, parent_idx:int) -> None:
            self.x = x
            self.y = y
            self.cost = cost
            self.parent_idx = int(parent_idx)
            
    # Class used for defining obstacles and obstacle params
    class Obstacle():
            def __init__(self, x_pos:float, y_pos:float, radius: float) -> None:
                self.x_pos = x_pos
                self.y_pos = y_pos
                self.radius = radius
            
    # Creating function that determines whether current location is in an obstacle

            def is_inside(self, curr_x:float, curr_y:float,
                robot_radius:float = 0.5) -> bool:  #orig:curr_x and curr_y
                    # If you don't want to include radius, can default it to 0

                # Euclidean distance from current location
                dist_from = np.sqrt((curr_x - self.x_pos)**2 + (curr_y - self.y_pos)**2)
                # orig: curr_x and curr_y
                
                if dist_from > self.radius + robot_radius:
                        return False
                return True # AKA, inside obstacle
        
    # Creating function that processes index eqn and returns index number (from hwk #1)

    def compute_index(min_x:int, max_x:int, min_y:int, max_y:int,
                    gs:float, curr_x:int, curr_y:int) -> float: #changed curr_x to start_x, same for y, changed back
        
        
        index = ((curr_x - min_x)/gs) + ((curr_y - min_y)/gs) * ((max_x+gs - min_x)/gs) # orig: curr_x and y

        return index        

    # Creating function that returns list of moves. This move list will be filtered
    # later to only include the most efficient route. 

    def get_all_moves(current_x:float, current_y:float, gs:float) -> list:
        # for x (-gs,gs) and then for y (-gs,gs, x = -0.5, y = 0)
        # gs = 0.5, curr.x = 0, curr.y = 0
        # move_list = []
        # For x -> (-gs,gs)
            # For y -> (-gs,gs)
                # dx = -0.5, dy = -0.5
        '''
        Inputs: 
            - current x and current y
            - grid space/ step size
            
        How should we do this:
            Hint a nested for loops helps and arange helps
            
        Returns a list of moves:
            - [[m1],[m2],[m3],...[mn]]
        '''
        # Creating empty list called move_list
        move_list = []
        gs_x_bounds = np.arange(-gs, gs+gs, gs)
        gs_y_bounds = np.arange(-gs, gs+gs, gs)

        for dx in gs_x_bounds:
            for dy in gs_y_bounds:
                x_next = current_x + dx
                y_next = current_y + dy  # took out -gs
                
                if [x_next, y_next] == [current_x, current_y]:
                    #print("no bueno")
                    continue # will continue onto next iteration
                
                # print(x_next, y_next)
                
                # insert it
                move = [x_next,y_next]
                move_list.append(move)  # move.append([Xn,Yn])
        return move_list

    # Creating function that checks if current location is inside or near an
    # obstacle and/or outside of grid space.

    def is_not_valid(obst_list:list, x_min:int, y_min:int, x_max:int, y_max:int,
                    x_curr:float, y_curr:float, agent_radius:float=0.0):
    
    # Checking if near obstacle or inside
        for obs in obst_list:
            if obs.is_inside(x_curr, y_curr,agent_radius):
                return True
    
    # Checking to make sure current location is within grid space
    # Return of True = NOT Valid. False = Valid 
    
        if x_min > x_curr: #orig: x_curr
            return True
        if x_max < x_curr: #orig: x_curr
            return True
        if y_min > y_curr:    #orig: y_curr
            return True
        if y_max < y_curr:    #orig: y_curr
            return True

        return False

    # Looping through obstacles, now that they are defined
    for obs_pos in obstacle_positions:
        obstacle = Obstacle(obs_pos[0], obs_pos[1], obstacle_radius)
        obstacle_list.append(obstacle)
        
    # Divide nodes up into two bins using dictionaries

    unvisited = {}
    visited = {}

    # Initialize current_node w/ starting params
        # current_node = Node(start_x, start_y, cost =0, parent_idx=-1)

    current_node = Node(start_x, start_y, 0, int(-1))

    # Initialize current_index by utilizing compute index function

    current_idx = int(compute_index(min_x, max_x, min_y, max_y,
                    gs, start_x, start_y))

    # Insert current_node into unvisited dictionary, using current_idx as key

    unvisited[current_idx] = current_node

    '''
    #sanity check:
    #if 0 in unvisited:
    #   print("yes",unvisited[0].x, unvisited[0].y) 
    '''

    # Djikstra Exploration

    while [current_node.x, current_node.y] != [goal_x, goal_y]: # != means not equal          
            
        # Setting current index to min value of unvisited dictionary
        current_idx = min(unvisited, key=lambda x:unvisited[x].cost)
        
        # Setting current_node to unvisited[current_index]
        current_node = unvisited[current_idx]
        
        # Placing current_node into visited dictionary
        visited[current_idx] = current_node
        
        # Deleting current_index from unvisited dictionary
        del unvisited[current_idx]
        
        #- Checking if current_node is equal to goal position
        if[current_node.x,current_node.y] == [goal_x,goal_y]:
            # If so return path and break
            print("Path Found :D")
            
            # Return path by storing waypoints
            wp_node = current_node
            wp_list = []
            wp_list.append([wp_node.x, wp_node.y])
            
            while wp_node.parent_idx != -1:
                next_idx = wp_node.parent_idx
                wp_node = visited[next_idx]            
                wp_list.append([wp_node.x, wp_node.y])
            break

    # Begin Search:
        
    # Based on current postion, use "get_all_moves" to return all moves made

        all_moves = get_all_moves(current_node.x,current_node.y,gs) #x_curr or current_node.x

    # Initializing a filtered moves list

        filtered_moves = []  # list uses [], dictionaries use {}
        
    # Filtering all_moves list: Check to make sure all moves are valid. Meaning:
        # - Not in obstacle - Not Outside of Grid Space - Not on top of ourselves -
        # 3rd parameter has already been checked through get_all_moves

        for move in all_moves:
            if (is_not_valid(obstacle_list, min_x, min_y, max_x, max_y, move[0], move[1]) == True):
                continue
            else:
                print("valid move", move[0], move[1])   
                # If move is valid we append to filtered
                filtered_moves.append(move)
            
            # Loop through all filtered moves
        for move in filtered_moves:
        
                # Calculate current filtered/new index using current filtered move
            new_index = int(compute_index(min_x, max_x, min_y, max_y, gs, move[0], move[1]))
            
            # Calculate the filtered/new cost from + to new_nodes
            new_cost = current_node.cost + m.dist(move,[current_node.x, current_node.y]) \
                + m.dist(move, [goal_x, goal_y]) # Adding Heuristic to make A* (How far from goal)
    
        # Checking if the new_index is inside visited
            if new_index in visited:
                continue
        
        # Checking if the new_index inside unvisited:
            if new_index in unvisited:
                    
            # First, comparing the new_cost to unvisited cost:
                if new_cost < unvisited[new_index].cost:
                        
                # Then, update the cost value
                    unvisited[new_index].cost = new_cost
                        
                # Finally, update the parent index
                    unvisited[new_index].parent_idx = current_idx
                        
                continue
    
        # Last Step           
        # Making a new node
            new_node = Node(move[0], move[1], new_cost, current_idx)

        # Appending said new node to unvisited list. 
            unvisited[new_index] = new_node
            
    # Plotting path, assuming no indent needed
    # for __ in __ list:
    fig, ax = plt.subplots()    
    ax.set_xlim(min_x, max_x)
    ax.set_ylim(min_y, max_y)  

    plt.title("Hwk 5, Prob 3: A*\nKaitie Butler =D", fontsize = 14, color = 'navy')
    plt.grid()

    # Plotting obstacles in way
    for obs in obstacle_list:
        obs_plot = plt.Circle((obs.x_pos, obs.y_pos), obs.radius, color='dimgrey')
        ax.add_patch(obs_plot)
        
    # Plot path
    #for wp in wp_list:
        #wp_plot = plt.plot([wp_node.x, wp_node.y], linewidth=2.0)
        
    #z = np.array(wp_list)
    #x,y = z.T

    wp_x,wp_y = np.array(wp_list).T
        # Essentially, variable = np.array(wp_list) and x,y = variable.T
        # The dot T is to transpose the array so that it can be graphed

    #ax.plot(x, y, linewidth=2.0)
    plt.plot(wp_x,wp_y,linewidth=2.0, color = 'limegreen' )
    plt.show()

    return wp_list

