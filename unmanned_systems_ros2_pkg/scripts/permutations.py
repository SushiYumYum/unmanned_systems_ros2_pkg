# Unique Permutations


# A Python program to print all 
# permutations using library function 
from itertools import permutations 
 
 
start_location = (0,0)

# Get all permutations of: 
perm = permutations([(1,2), (3,4), (6,6), (15,15)]) # Tuples

# Pseudo to get total cost
# Designate Grid Space and Bounds. Start and end location are re-evaluated each time
# Initiate obstacles, if any
# 
 
# Print the obtained permutations 

print("Number of Combinations is", len(list(perm)))

#for i in list(perm): 
    #print (i) 