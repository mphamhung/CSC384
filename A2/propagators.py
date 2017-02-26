#Look for #IMPLEMENT tags in this file. These tags indicate what has
#to be implemented to complete problem solution.  

'''This file will contain different constraint propagators to be used within 
   bt_search.

   propagator == a function with the following template
      propagator(csp, newVar=None)
           ==> returns (True/False, [(Variable, Value), (Variable, Value) ...]

      csp is a CSP object---the propagator can use this to get access
      to the variables and constraints of the problem. The assigned variables
      can be accessed via methods, the values assigned can also be accessed.

      newVar (newly instaniated variable) is an optional argument.
      if newVar is not None:
          then newVar is the most
           recently assigned variable of the search.
      else:
          progator is called before any assignments are made
          in which case it must decide what processing to do
           prior to any variables being assigned. SEE BELOW

       The propagator returns True/False and a list of (Variable, Value) pairs.
       Return is False if a deadend has been detected by the propagator.
       in this case bt_search will backtrack
       return is true if we can continue.

      The list of variable values pairs are all of the values
      the propagator pruned (using the variable's prune_value method). 
      bt_search NEEDS to know this in order to correctly restore these 
      values when it undoes a variable assignment.

      NOTE propagator SHOULD NOT prune a value that has already been 
      pruned! Nor should it prune a value twice

      PROPAGATOR called with newVar = None
      PROCESSING REQUIRED:
        for plain backtracking (where we only check fully instantiated 
        constraints) 
        we do nothing...return true, []

        for forward checking (where we only check constraints with one
        remaining variable)
        we look for unary constraints of the csp (constraints whose scope 
        contains only one variable) and we forward_check these constraints.

        for gac we establish initial GAC by initializing the GAC queue
        with all constaints of the csp


      PROPAGATOR called with newVar = a variable V
      PROCESSING REQUIRED:
         for plain backtracking we check all constraints with V (see csp method
         get_cons_with_var) that are fully assigned.

         for forward checking we forward check all constraints with V
         that have one unassigned variable left

         for gac we initialize the GAC queue with all constraints containing V.
   '''
   
import queue 

def prop_BT(csp, newVar=None):
    '''Do plain backtracking propagation. That is, do no 
    propagation at all. Just check fully instantiated constraints'''
    
    if not newVar:
        return True, []
    for c in csp.get_cons_with_var(newVar):
        if c.get_n_unasgn() == 0:
            vals = []
            vars = c.get_scope()
            for var in vars:
                vals.append(var.get_assigned_value())
            if not c.check(vals):
                return False, []
    return True, []

def prop_FC(csp, newVar=None):
    '''Do forward checking. That is check constraints with 
       only one uninstantiated variable. Remember to keep 
       track of all pruned variable,value pairs and return '''
#IMPLEMENT
    tb_pruned = []
    
    if newVar:
        constraints = csp.get_cons_with_var(newVar)
    else:
        constraints = csp.get_all_cons()
        
    for c in constraints:
        if c.get_n_unasgn() == 1:
            vals = []
            vars = c.get_scope()
            var = c.get_unasgn_vars()[0]
            
            for i in range(len(vars)):
                if vars[i].is_assigned():
                    vals.append(vars[i].get_assigned_value())
                else:
                    vals.append(None)
                    unasgn_index = i
            
            if var.cur_domain_size() != 0:
                for x in var.cur_domain():
                    vals[unasgn_index] = x
                    if not c.check(vals):
                        tb_pruned.append((var,x))
                        var.prune_value(x)
                        if var.cur_domain_size() ==0:
                            return False, tb_pruned
            else:
                return False, tb_pruned
    

    return True, tb_pruned   
                
def prop_GAC(csp, newVar=None):
    '''Do GAC propagation. If newVar is None we do initial GAC enforce 
       processing all constraints. Otherwise we do GAC enforce with
       constraints containing newVar on GAC Queue'''
#IMPLEMENT
    tb_pruned = []
    s = set([])
    GAC_Queue = queue.Queue()
    
    if newVar:
        constraints = csp.get_cons_with_var(newVar)
    else:
        constraints = csp.get_all_cons()
    
    for i in constraints:
        GAC_Queue.put(i)
        s.add(i)
        
    while not GAC_Queue.empty():
        C = GAC_Queue.get()
        s.remove(C)
        for var in C.get_scope(): #each member of scope (C)
            if var.cur_domain_size() == 0:
                return False, tb_pruned
            
            for val in var.cur_domain(): #for d in CurDom(V)
                if C.has_support(var, val) == False:
                    tb_pruned.append((var,val))
                    var.prune_value(val)
                    if var.cur_domain_size() == 0:
                        return False , tb_pruned#DWO
                    else:
                        for con in csp.get_cons_with_var(var):
                            if con not in s:
                                GAC_Queue.put(con)
                                s.add(con)
    

    
    return True, tb_pruned
        
                            
                    
                        
    
            
    

    