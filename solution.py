#Look for #IMPLEMENT tags in this file. These tags indicate what has
#to be implemented to complete the Sokoban warehouse domain.

#   You may add only standard python imports---i.e., ones that are automatically
#   available on TEACH.CS
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

# import os for time functions
import os
from search import * #for search engines
from sokoban import SokobanState, Direction, PROBLEMS, sokoban_goal_state #for Sokoban specific classes and problems
from numpy import *
#SOKOBAN HEURISTICS
def heur_displaced(state):
  '''trivial admissible sokoban heuristic'''
  '''INPUT: a sokoban state'''
  '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''       
  count = 0
  for box in state.boxes:
    if box not in state.storage:
      count += 1
  return count

def heur_manhattan_distance(state):
#IMPLEMENT
  '''admissible sokoban heuristic: manhattan distance'''
  '''INPUT: a sokoban state'''
  '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''      
  #We want an admissible heuristic, which is an optimistic heuristic. 
  #It must always underestimate the cost to get from the current state to the goal.
  #The sum Manhattan distance of the boxes to their closest storage spaces is such a heuristic.  
  #When calculating distances, assume there are no obstacles on the grid and that several boxes can fit in one storage bin.
  #You should implement this heuristic function exactly, even if it is tempting to improve it.
  #Your function should return a numeric value; this is the estimate of the distance to the goal.
  
  total_distance = 0  
  
  def calc_manhattan_dist(box, storage):
    return (abs(box[0]-storage[0])+abs(box[1]-storage[1]))

  for box, restrict in state.boxes.items():
    distance = float('inf')
    if (state.restrictions):
      for storage in state.restrictions[restrict]:
        if calc_manhattan_dist(box, storage) < distance:
          distance = calc_manhattan_dist(box, storage)
    else:
      for storage in state.storage:
        if calc_manhattan_dist(box, storage) < distance:
          distance = calc_manhattan_dist(box, storage)
    
    total_distance += distance
        
  return total_distance


def next_to_wall(box, obstacles):
  if box[0]-1  in obstacles or box[0]+1 in obstacles:
    return True
  if box[1]-1  in obstacles or box[1] +1 in obstacles:
    return True
  else: 
    return False
    

def heur_alternate(state):
  '''a better sokoban heuristic'''
  '''INPUT: a sokoban state'''
  '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''        
  #heur_manhattan_distance has flaws.   
  #Write a heuristic function that improves upon heur_manhattan_distance to estimate distance between the current state and the goal.
  #Your function should return a numeric value for the estimate of the distance to the goal.  
  global walls
  global prev_states
  total_distance = 0  
  
  
  if not state.parent:
    walls = []
    prev_states = []
  
  # if state.hashable_state in prev_states:
  #   return 100000
  # else:
  #   prev_states.append(state.hashable_state())
  #   
  
  def calc_dist(box, storage):
    return (abs(box[0]-storage[0])+abs(box[1]-storage[1]))

  def in_corner(box, obstacles):
    if box[0]-1  in obstacles or box[0]+1 in obstacles:
      if box[1]-1  in obstacles or box[1] +1 in obstacles:
        return True
    else:
      return False
      
  def next_to_wall(box, obstacles):
    if box[0]-1  in obstacles or box[0]+1 in obstacles:
      return True
    if box[1]-1  in obstacles or box[1] +1 in obstacles:
      return True
    else: 
      return False
    
  # def obstruction_in_between(box, storage):
  #   distance = calc_dist(box, storage)
  #   for obstacle in state.obstacles:
  #     if calc_dist(box, obstacle)+calc_dist(storage, obstacle) == distance:
  #       distance += 4
  #   
  #   return distance
    

  #Initilizing walls
  if not walls:
    for i in range(state.width):
        walls.append((i, -1))
        walls.append((i, state.height))
        
    for j in range(state.height):
        walls.append((-1, j))
        walls.append((state.width, j))
        
        
  # #wall checking      
  # for storage in state.storage:
  #   if not next_to_wall(storage, walls):
  #     for box in state.boxes:
  #       if next_to_wall(box, walls):
  #         return float('inf')
          
  #corner checking
  tmp = []
  for box in state.boxes:
    if box not in state.storage:
      if in_corner(box, walls+tmp+list(state.obstacles)):
          return float('inf')
            
      if next_to_wall(box, walls+list(state.obstacles)):
          tmp.append(box)

            
  # Distance from robot to boxes to their storage
  tmp =[]
  for box, restrict in state.boxes.items():
    distance = float('inf')
    if (state.restrictions):
      for storage in state.restrictions[restrict]:
        if calc_dist(box, storage) + calc_dist(box, state.robot) < distance:
          distance = calc_dist(box, storage) + calc_dist(box, state.robot)
    else:
      for storage in state.storage:
        if calc_dist(box, storage) + calc_dist(box, state.robot) < distance:
          distance = calc_dist(box, storage) + calc_dist(box, state.robot)

          
    total_distance += distance
    
    
  # #Distance of robot to nearest box
  # for box in state.boxes:
  #   distance = float('inf')
  #   if calc_manhattan_dist(state.robot, box) < distance:
  #     distance = calc_manhattan_dist(state.robot, box)
  #     
  # total_distance += distance
  #       
  return total_distance


def heur_alternate_1(state):
#IMPLEMENT
  '''a better sokoban heuristic'''
  '''INPUT: a sokoban state'''
  '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''        
  #heur_manhattan_distance has flaws.   
  #Write a heuristic function that improves upon heur_manhattan_distance to estimate distance between the current state and the goal.
  #Your function should return a numeric value for the estimate of the distance to the goal.  
  
  '''Let's take into consideration the walls and obstacles'''
  
  total_distance = 0
  
  global walls
  global states
  
  def calc_dist(box, storage):
    return (abs(box[0]-storage[0])+abs(box[1]-storage[1]))
    
  if not state.parent:
    walls = []
    for i in range(state.width):
      walls.append((i, -1))
      walls.append((i, state.height))
      
    for j in range(state.height):
      walls.append((-1, j))
      walls.append((state.width, j))
    
    walls += list(state.obstacles)

  def is_storageable(box):
    n = {}
    n['UP'] = (box[0]+1, box[1])
    n['DOWN']= (box[0]-1, box[1])
    n['RIGHT'] = (box[0], box[1]+1)
    n['LEFT'] = (box[0], box[1]-1)
    
    storagable = False
    
    for storage in state.storage:
      h = box[0] - storage[0] #Storage is Right if Negative, Left if Positive
      v = box[1] - storage[1] #Storage is Down if Negative, Up if Positive
      k = 0
      
      if h < 0:
        if n['LEFT'] in walls:
          k+=1
          
      else:
        if n['RIGHT'] in walls:
          k+=1
  
      if v < 0:
        if n['UP'] in walls:
          k+=1
      else:
        if n['DOWN'] in walls:
          k+=1
    
      if k<2:
        storageble = True
      
    if storagable:
      return 0
    else:
      return 10000
      
  distance = float('inf')    
  if (state.restrictions):
    #has restrictions
    for box, restrict in state.boxes.items():
      for storage in state.restrictions[restrict]:
        dist_temp = calc_dist(box, storage)
        if dist_temp < distance:
          distance = dist_temp
          
      total_distance += distance + is_storageable(box)
      
  else:
    # problem has no restrictions
    for box, restrict in state.boxes.items():
      for storage in state.storage:
        dist_temp = calc_dist(box, storage)
        if dist_temp < distance:
          distance = dist_temp + is_storageable(box)
          
      total_distance += distance
      
  return total_distance
  


def fval_function(sN, weight):
#IMPLEMENT
    """
    Provide a custom formula for f-value computation for Anytime Weighted A star.
    Returns the fval of the state contained in the sNode.

    @param sNode sN: A search node (containing a SokobanState)
    @param float weight: Weight given by Anytime Weighted A star
    @rtype: float
    """
  
    #Many searches will explore nodes (or states) that are ordered by their f-value.
    #For UCS, the fvalue is the same as the gval of the state. For best-first search, the fvalue is the hval of the state.
    #You can use this function to create an alternate f-value for states; this must be a function of the state and the weight.
    #The function must return a numeric f-value.
    #The value will determine your state's position on the Frontier list during a 'custom' search.
    #You must initialize your search engine object as a 'custom' search engine if you supply a custom fval function.
    return sN.gval + weight*sN.hval
    
def anytime_gbfs(initial_state, heur_fn, timebound = 10):
#IMPLEMENT
  '''Provides an implementation of anytime greedy best-first search, as described in the HW1 handout'''
  '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
  '''OUTPUT: A goal state (if a goal is found), else False''' 
  
  max_time = os.times()[0] + timebound
  start_time = os.times()[0]
  #print("Starting at:", os.times()[0])
  
  se = SearchEngine('best_first', 'full')
  se.init_search(initial_state, sokoban_goal_state, heur_fn)
  current_best = se.search(timebound)
  if not current_best:
    return False
  
  while os.times()[0] < max_time:
    new_result = se.search(timebound-(os.times()[0]-start_time), (current_best.gval, float('inf'), float('inf')))
    if not new_result:
      return current_best
    current_best = new_result
    
  return current_best

def anytime_weighted_astar(initial_state, heur_fn, weight=1., timebound = 10):
#IMPLEMENT
  '''Provides an implementation of anytime weighted a-star, as described in the HW1 handout'''
  '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
  '''OUTPUT: A goal state (if a goal is found), else False''' 
    
  max_time = os.times()[0] + timebound
  start_time = os.times()[0]
  #print("Starting at:", os.times()[0])
  
  se = SearchEngine('custom', 'full')
  wrapped_fval_function = (lambda sN: fval_function(sN, weight))
  se.init_search(initial_state, sokoban_goal_state, heur_fn, wrapped_fval_function)
  current_best = se.search(timebound)
  if not current_best:
    return False
  
  while os.times()[0] < max_time:
    new_result = se.search(timebound-(os.times()[0]-start_time), (float('inf'), float('inf'), current_best.gval))
    if not new_result:
      return current_best
    current_best = new_result
    
  return current_best

if __name__ == "__main__":
  
  '''
  solved = 0; unsolved = []; counter = 0; percent = 0; timebound = 8; #8 second time limit 
  print("Running Anytime Weighted A-star")   

  for i in range(0, 10):
    print("*************************************")  
    print("PROBLEM {}".format(i))

    s0 = PROBLEMS[i] #Problems get harder as i gets bigger
    weight = 10
    final = anytime_gbfs(s0, heur_fn=heur_manhattan_distance, timebound=timebound)

    if final:
      final.print_path()   
      solved += 1 
    else:
      unsolved.append(i)
    counter += 1      

  if counter > 0:  
    percent = (solved/counter)*100   
      
  print("*************************************")  
  print("{} of {} problems ({} %) solved in less than {} seconds.".format(solved, counter, percent, timebound))  
  print("Problems that remain unsolved in the set are Problems: {}".format(unsolved))      
  print("*************************************") 

  #TEST CODE
  solved = 0; unsolved = []; counter = 0; percent = 0; timebound = 2; #2 second time limit for each problem
  print("*************************************")  
  print("Running A-star")     

  for i in range(0, 10): #note that there are 40 problems in the set that has been provided.  We just run through 10 here for illustration.

    print("*************************************")  
    print("PROBLEM {}".format(i))
    
    s0 = PROBLEMS[i] #Problems will get harder as i gets bigger

    se = SearchEngine('astar', 'full')
    se.init_search(s0, goal_fn=sokoban_goal_state, heur_fn=heur_alternate)
    final = se.search(timebound)

    if final:
      final.print_path()
      solved += 1
    else:
      unsolved.append(i)    
    counter += 1

  if counter > 0:  
    percent = (solved/counter)*100
  print("*************************************")  
  print("{} of {} problems ({} %) solved in less than {} seconds.".format(solved, counter, percent, timebound))  
  print("Problems that remain unsolved in the set are Problems: {}".format(unsolved))      
  print("*************************************") 

  '''
  solved = 0; unsolved = []; counter = 0; percent = 0; timebound = 8; #8 second time limit
  print("Running Anytime Weighted A-star")

  for i in range(0, 10):
    print("*************************************")
    print("PROBLEM {}".format(i))

    s0 = PROBLEMS[i] #Problems get harder as i gets bigger
    weight = 10
    final = anytime_weighted_astar(s0, heur_fn=heur_displaced, weight=weight, timebound=timebound)

    if final:
      final.print_path()
      solved += 1
    else:
      unsolved.append(i)
    counter += 1

  if counter > 0:
    percent = (solved/counter)*100   
      
  print("*************************************")  
  print("{} of {} problems ({} %) solved in less than {} seconds.".format(solved, counter, percent, timebound))  
  print("Problems that remain unsolved in the set are Problems: {}".format(unsolved))      
  print("*************************************") 
