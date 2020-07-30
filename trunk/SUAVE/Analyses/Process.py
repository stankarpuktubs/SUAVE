## @ingroup Analyses
# Process.py
#
# Created:  
# Modified: Feb 2016, Andrew Wendorff

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

from SUAVE.Core import ContainerOrdered, DataOrdered, Data

# ----------------------------------------------------------------------
#  Process
# ----------------------------------------------------------------------

## @ingroup Analyses
class Process(ContainerOrdered):
    """ SUAVE.Analyses.Process()
    
        The Top Level Process Container Class
        
            Assumptions:
            None
            
            Source:
            N/A
    """    
    
    verbose = False
    
    def evaluate(self,*args,**kwarg):
        """This is used to execute the evaluate functions of the analyses
            stored in the container.
        
                Assumptions:
                None
        
                Source:
                N/A
        
                Inputs:
                None
        
                Outputs:
                Results of the Evaluate Functions
        
                Properties Used:
                N/A
            """        
        
        results = Data()
        
        if self.verbose:
            print('process start')
        
        for tag,step in self.items(): 
            
            #if self.verbose:
                #print('step :' , tag)
            
            #if not callable(step): continue
            
            if hasattr(step,'evaluate'): 
                self = step.evaluate(*args,**kwarg)
            else:
                self = step(*args,**kwarg)
                
            #results[tag] = result
        
        #: for each step
        
        #if self.verbose:
            #print('process end')        
        
        return self
        
    def __call__(self,*args,**kwarg):
        """This is used to set the class' call behavior to the evaluate functions.
        
                Assumptions:
                None
        
                Source:
                N/A
        
                Inputs:
                None
        
                Outputs:
                None
        
                Properties Used:
                N/A
            """                        
        return self.evaluate(*args,**kwarg) 
    
