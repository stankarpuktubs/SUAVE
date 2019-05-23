#!/usr/bin/env python3
# -*- coding: utf-8 -*-

## @ingroup components-cabin
# Cabin.py
# 
# Created:  April 2019, M. Dethy

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

import SUAVE
from SUAVE.Components import Physical_Component, Lofted_Body
from SUAVE.Core import Data, Container, ContainerOrdered, Units

# ------------------------------------------------------------
#  Cabin
# ------------------------------------------------------------

## @ingroup components-cabin
class Cabin(Lofted_Body):
    """ This is a standard cabin for a tube and wing aircraft.
    
    Assumptions:
    Conventional cabin
    
    Source:
    N/A
    """
    
    def __defaults__(self):
        """ This sets the default values for the component to function.
        
        Assumptions:
        None
    
        Source:
        N/A
    
        Inputs:
        None
    
        Outputs:
        None
    
        Properties Used:
        None
        """      
        
        self.tag                =  'cabin'
        self.origin             = [[0.0,0.0,0.0]]
        self.aerodynamic_center = [0.0,0.0,0.0]
        self.Sections           = Lofted_Body.Section.Container()
        self.Segments           = ContainerOrdered()
        
        self.seat_pitch                 = 38.0 * Units.inch
        self.seat_width                 = 20.0 * Units.inch
        self.n_aisles                   = 1
        self.aisle_width                = 20.0 * Units.inch
        self.total_seats                = 0.0
        self.total_cargo_area           = 0.0
        self.total_seat_area            = 0.0
        
        self.Segments                   = SUAVE.Core.ContainerOrdered()
        self.i_start_segment            = 0
        self.i_end_segment              = 0
        self.length_scale               = 0.0
        
    def append_segment(self,segment):
        """ Adds a segment to the cabin. 
    
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

        # Assert database type
        if not isinstance(segment,Data):
            raise Exception('input component must be of type Data()')

        # Store data
        self.Segments.append(segment)

        return
        
    def get_total_length(self):
        """ Calculates the total cabin length based on segments. 
    
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
        start_segment = self.Segments[self.i_start_segment].percent_x_location
        end_segment   = self.Segments[self.i_end_segment].percent_x_location
        
        return (end_segment - start_segment)*self.length_scale
        

class Container(Physical_Component.Container):
    pass
        
        

# ------------------------------------------------------------
#  Handle Linking
# ------------------------------------------------------------

Cabin.Container = Container