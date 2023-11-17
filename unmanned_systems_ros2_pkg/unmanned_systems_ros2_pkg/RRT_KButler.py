# -*- coding: utf-8 -*-
"""
Created on Wed Sep 13 08:32:09 2023

RRT Method (Rapidly Exploring Random Tree) - Hwk 3, Problem 3

- Kaitie Butler

- Goal: To Modify Djikstra's/A* methods to become RRT Method
    - Create dictionary for tree
        tree = []
    - Add starting location to dictionary/tree
        ???
    - While our newest node != goal   line 197
        - get random configuration -> rand_loc = (x_rand,y_rand)
        - x_rand = r.randrange(x_min,x_max) -> may need input to be gs_x_bounds
        - y_rand = r.randrange(y_min, y_max) 
    
        - Find nearest node in tree (using dist) - (use min function)
        - Calculate new node location in direction of rand_loc by jumping dl from 
      closes node
        - Check if new node is valid
            - if yes, store in tree
    - Create a plot showing tree (valid nodes) and corresponding path

"""

import numpy as np
import math as m
import matplotlib.pyplot as plt
import random as r

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
              robot_radius:float = 0) -> bool:  #orig:curr_x and curr_y
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
                 x_curr:float, y_curr:float, agent_radius:float=0):
   
   # Checking if near obstacle or inside
   for obs in obst_list:
       if obs.is_inside(x_curr, y_curr, agent_radius):
           print("You're dead at ", obs.x_pos, obs.y_pos)
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


'''
#sanity check:
#if 0 in unvisited:
 #   print("yes",unvisited[0].x, unvisited[0].y) 
'''

# Djikstra Exploration
 
'''   
While Current location =! Goal Location  
  - get random configuration -> rand_loc = (x_rand,y_rand)
      - x_rand = r.randrange(x_min,x_max) -> may need input to be gs_x_bounds
      - y_rand = r.randrange(y_min, y_max) 

  - Find nearest node in tree (using dist) - (use min function)
      dist_from = np.sqrt((curr_x - self.x_pos)**2 + (curr_y - self.y_pos)**2)
  - Calculate new node location in direction of rand_loc by jumping dl from 
closest node
  - Check if new node is valid
      - if yes, store in tree

'''
tree = []

# Defining compute distance function btwn current and random node
def search_for_min(pos_x:int ,pos_y:int, tree_list:list) ->int:
    

    min_dist = np.inf  # or could set to really high number
    
    min_idx= -1
    
    
    for i, point in enumerate(tree):  # allow you to get index
        curr_pos = [pos_x,pos_y]
        # tree_pos = [point[0],point[1]]
        tree_pos = [point.x, point.y]
        curr_dist =  m.dist(tree_pos,curr_pos)
    
        
        if curr_dist < min_dist:
            min_dist = curr_dist
            min_idx = i
            
    return min_idx

def compute_dist(current_node:int, x_rand:int, y_rand:int) -> float:
    
    distance = np.sqrt((x_rand - current_node.x)**2 + (y_rand - current_node.y)**2)
    
    return distance

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

def run_RRT(min_x:float, max_x:float, min_y:float, max_y:float, gs:float, goal_x:float, goal_y:float):


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

    # obstacle_positions = []
    obstacle_list = [] # store obstacle classes
    obstacle_radius = 0.25

    # Looping through obstacles, now that they are defined
    for obs_pos in obstacle_positions:
        obstacle = Obstacle(obs_pos[0], obs_pos[1], obstacle_radius)
        obstacle_list.append(obstacle)
        
    # Divide nodes up into two bins using dictionaries

    unvisited = {}
    visited = {}

    # tree = []   # Initializing tree dictionary

    # Initialize current_node w/ starting params
        # current_node = Node(start_x, start_y, cost =0, parent_idx=-1)

    current_node = Node(start_x, start_y, 0, int(-1))

    # Initialize current_index by utilizing compute index function

    current_idx = int(compute_index(min_x, max_x, min_y, max_y,
                    gs, start_x, start_y))

    # Insert current_node into unvisited dictionary, using current_idx as key

    tree.append(current_node)

    min_dist = 0
    min_idx = 0
    x_rand = 0
    y_rand = 0

    x_bounds = [min_x, max_x]
    y_bounds = [min_y, max_y]

    delta_l = 0.25 #step size
    idx_counter = 0
    while [current_node.x, current_node.y] != [goal_x, goal_y]: # != means not equal            

        # Randomizer 
        x_rand = r.randrange(min_x, max_x)
        y_rand = r.randrange(min_y, max_y)
        
            
        min_idx = search_for_min(x_rand, y_rand, tree)
        
        # current_dist = compute_dist(current_node.x, current_node.y, x_rand, y_rand)
        # min_dist = min(tree, key=lambda x:tree[x].distance)

        current_node = tree[min_idx]
        print("current node is", current_node.x, current_node.y)
        current_position = [current_node.x, current_node.y]
        
        if m.dist(current_position, [goal_x, goal_y]) < delta_l + 1:
            wp_node = current_node 
            wp_list = []
            while wp_node.parent_idx != -1:
                current_position = [wp_node.x, wp_node.y]
                wp_list.append(current_position)
                wp_node = tree[wp_node.parent_idx]
            break

        print(current_node.x, current_node.y)
        
        dx = x_rand - current_node.x
        dy = y_rand - current_node.y
        
        theta_rad = np.arctan2(dy,dx)
        
        
        x_new  = current_node.x  + delta_l*np.sin(theta_rad)
        y_new = current_node.y + delta_l*np.cos(theta_rad)
        
        x_new = round(x_new, 4)
        y_new = round(y_new, 4)


        if (is_not_valid(obstacle_list, min_x, min_y, max_x, max_y, x_new, y_new) == True):
            continue
        
        for point in tree:
            if point.x == x_new and point.y == y_new:
                continue
        
        new_node = Node(x_new, y_new, 0, min_idx)
        tree.append(new_node)

        idx_counter = idx_counter + 1
            
    # Plotting path, assuming no indent needed
    # for __ in __ list:
    fig, ax = plt.subplots()    
    ax.set_xlim(min_x, max_x)
    ax.set_ylim(min_y, max_y)  

    plt.title("Hwk 5, Prob 3: RRT\nKaitie Butler =D", fontsize = 14, color = 'navy')
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
    plt.plot(wp_x,wp_y,'-o', linewidth=1.0, color = 'limegreen' )
    plt.show()

    return wp_list
