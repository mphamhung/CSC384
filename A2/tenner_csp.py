#Look for #IMPLEMENT tags in this file. These tags indicate what has
#to be implemented to complete the warehouse domain.  

'''
Construct and return Tenner Grid CSP models.
'''

from cspbase import *
import itertools

def tenner_csp_model_1(initial_tenner_board):
    '''Return a CSP object representing a Tenner Grid CSP problem along 
       with an array of variables for the problem. That is return

       tenner_csp, variable_array

       where tenner_csp is a csp representing tenner grid using model_1
       and variable_array is a list of lists

       [ [  ]
         [  ]
         .
         .
         .
         [  ] ]

       such that variable_array[i][j] is the Variable (object) that
       you built to represent the value to be placed in cell i,j of
       the Tenner Grid (only including the first n rows, indexed from 
       (0,0) to (n,9)) where n can be 3 to 8.
       
       
       The input board is specified as a pair (n_grid, last_row). 
       The first element in the pair is a list of n length-10 lists.
       Each of the n lists represents a row of the grid. 
       If a -1 is in the list it represents an empty cell. 
       Otherwise if a number between 0--9 is in the list then this represents a 
       pre-set board position. E.g., the board
    
       ---------------------  
       |6| |1|5|7| | | |3| |
       | |9|7| | |2|1| | | |
       | | | | | |0| | | |1|
       | |9| |0|7| |3|5|4| |
       |6| | |5| |0| | | | |
       ---------------------
       would be represented by the list of lists
       
       [[6, -1, 1, 5, 7, -1, -1, -1, 3, -1],
        [-1, 9, 7, -1, -1, 2, 1, -1, -1, -1],
        [-1, -1, -1, -1, -1, 0, -1, -1, -1, 1],
        [-1, 9, -1, 0, 7, -1, 3, 5, 4, -1],
        [6, -1, -1, 5, -1, 0, -1, -1, -1,-1]]
       
       
       This routine returns model_1 which consists of a variable for
       each cell of the board, with domain equal to {0-9} if the board
       has a -1 at that position, and domain equal {i} if the board has
       a fixed number i at that cell.
       
       model_1 contains BINARY CONSTRAINTS OF NOT-EQUAL between
       all relevant variables (e.g., all pairs of variables in the
       same row, etc.).
       model_1 also constains n-nary constraints of sum constraints for each 
       column.
    '''
    
#IMPLEMENT
    def get_nb(w,h,row,col):
        neighbours = []
        for i in [-1,1]:
            for j in [-1,0,1]:
                if 0<=row+i<h:
                    if 0<=col+j<w:
                        neighbours.append((row+i,col+j))
        return neighbours
        
        
    n_grid = initial_tenner_board[0]
    last_row = initial_tenner_board[1]
    height = len(n_grid)
    width = len(last_row)
        
    empty_domain = [0,1,2,3,4,5,6,7,8,9]
    
    vars = []
    for i in range(height):
        row = []
        for j in range(width):
            if n_grid[i][j] == -1:
                row.append(Variable('({},{})'.format(i,j), empty_domain))
            else:
                row.append(Variable('({},{})'.format(i,j), [n_grid[i][j]]))
        vars.append(row)
    
    cons = []
    
    #Row Constraints:
    for i in range(height):
        for j in range(width):
            for k in range(j+1, width):
                con = Constraint("Row{}_Pairs({},{})".format(i,j,k), [vars[i][j], vars[i][k]])
                sat_tuples = []
                for t in itertools.product(vars[i][j].domain(), vars[i][k].domain()):
                    if t[0] != t[1]:
                        sat_tuples.append(t)
                con.add_satisfying_tuples(sat_tuples)
                cons.append(con)
    
    #Adjacency Cosntraints: only need to check column and diagonals
    for i in range(height):
        for j in range(width):
            nbs = get_nb(width,height,i,j)
            for n in nbs:
                con = Constraint("AdjBwn({},{})_({},{})".format(i,j,n[0],n[1]), [vars[i][j], vars[n[0]][n[1]]])
                sat_tuples = []
                for t in itertools.product(vars[i][j].domain(), vars[n[0]][n[1]].domain()):
                    if t[0]!=t[1]:
                        sat_tuples.append(t)
                con.add_satisfying_tuples(sat_tuples)
                cons.append(con)

    #Column N-ary Constraints
    vars_T = [list(a) for a in zip(*vars)] #transposed
    for i in range(width):
        con = Constraint("Col{}_Sum".format(i), vars_T[i])
        doms = []
        sat_tuples = []
        for d in vars_T[i]:
            doms.append(d.domain())
        for t in itertools.product(*doms):
            if sum(t) == last_row[i]:
                sat_tuples.append(t)
        con.add_satisfying_tuples(sat_tuples)
        cons.append(con)
        
    flatvars = []
    for row in vars:
        for var in row:
            flatvars.append(var)
            
    csp = CSP("{}x10 Tenner Grid Model".format(height, vars), flatvars)
    
    for c in cons:
        csp.add_constraint(c)
    
    return csp, vars

##############################

def tenner_csp_model_2(initial_tenner_board):
    '''Return a CSP object representing a Tenner Grid CSP problem along 
       with an array of variables for the problem. That is return

       tenner_csp, variable_array

       where tenner_csp is a csp representing tenner using model_1
       and variable_array is a list of lists

       [ [  ]
         [  ]
         .
         .
         .
         [  ] ]

       such that variable_array[i][j] is the Variable (object) that
       you built to represent the value to be placed in cell i,j of
       the Tenner Grid (only including the first n rows, indexed from 
       (0,0) to (n,9)) where n can be 3 to 8.

       The input board takes the same input format (a list of n length-10 lists
       specifying the board as tenner_csp_model_1.
    
       The variables of model_2 are the same as for model_1: a variable
       for each cell of the board, with domain equal to {0-9} if the
       board has a -1 at that position, and domain equal {i} if the board
       has a fixed number i at that cell.

       However, model_2 has different constraints. In particular,
       model_2 has a combination of n-nary 
       all-different constraints and binary not-equal constraints: all-different 
       constraints for the variables in each row, binary constraints for  
       contiguous cells (including diagonally contiguous cells), and n-nary sum 
       constraints for each column. 
       Each n-ary all-different constraint has more than two variables (some of 
       these variables will have a single value in their domain). 
       model_2 should create these all-different constraints between the relevant 
       variables.
    '''

#IMPLEMENT
    #_____________________________________________________________________________#
    def get_nb(w,h,row,col):
        neighbours = []
        for i in [-1,1]:
            for j in [-1,0,1]:
                if 0<=row+i<h:
                    if 0<=col+j<w:
                        neighbours.append((row+i,col+j))
        return neighbours
        
    def alldiff(tuple):
        return len(set(tuple)) == len(tuple)
    #_____________________________________________________________________________#
    
    n_grid = initial_tenner_board[0]
    last_row = initial_tenner_board[1]
    height = len(n_grid)
    width = len(last_row)
        
    empty_domain = [0,1,2,3,4,5,6,7,8,9]
    
    vars = []
    for i in range(height):
        row = []
        for j in range(width):
            if n_grid[i][j] == -1:
                row.append(Variable('({},{})'.format(i,j), empty_domain))
            else:
                row.append(Variable('({},{})'.format(i,j), [n_grid[i][j]]))
        vars.append(row)
    
    cons = []
    
    #Row n-ary all-dif constraint
    for i in range(height):
        con = Constraint("Row{}_N-ary AllDiff".format(i), vars[i])
        doms = []
        non = []
        sat_tuples = []
        for k in range(width):
            if n_grid[i][k] != -1:
                non += vars[i][k].domain()
        for d in vars[i]:
            if d.domain_size()>1:
                doms.append(list(set(d.domain()).difference(set(non))))
            else:
                doms.append(d.domain())
        for t in itertools.product(*doms):
            if alldiff(t):
                sat_tuples.append(t)
        con.add_satisfying_tuples(sat_tuples)
        cons.append(con)
        
        
    #Column n-ary sum constraint
    vars_T = [list(a) for a in zip(*vars)] #transposed
    for i in range(width):
        con = Constraint("Col{}_Sum".format(i), vars_T[i])
        doms = []
        sat_tuples = []
        for d in vars_T[i]:
            doms.append(d.domain())
        for t in itertools.product(*doms):
            if sum(t) == last_row[i]:
                sat_tuples.append(t)
        con.add_satisfying_tuples(sat_tuples)
        cons.append(con)
        
    #Adjacency Constraints: only need to check column neighbours and diagonals
    for i in range(height):
        for j in range(width):
            nbs = get_nb(width,height,i,j)
            for n in nbs:
                con = Constraint("AdjBwn({},{})_({},{})".format(i,j,n[0],n[1]), [vars[i][j], vars[n[0]][n[1]]])
                sat_tuples = []
                for t in itertools.product(vars[i][j].domain(), vars[n[0]][n[1]].domain()):
                    if t[0]!=t[1]:
                        sat_tuples.append(t)
                con.add_satisfying_tuples(sat_tuples)
                cons.append(con)
          
    #Creating csp object
    flatvars = []
    for row in vars:
        for var in row:
            flatvars.append(var)
            
    csp = CSP("{}x10 Tenner Grid Model".format(height, vars), flatvars)
    
    for c in cons:
        csp.add_constraint(c)
    
    return csp, vars