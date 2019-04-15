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
from SUAVE.Core import Data, Container, ContainerOrdered
from SUAVE.Components import Physical_Component, Lofted_Body

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
        
        self.tag = 'cabin'
        self.origin             = [[0.0,0.0,0.0]]
        self.aerodynamic_center = [0.0,0.0,0.0]
        self.Sections    = Lofted_Body.Section.Container()
        self.Segments    = ContainerOrdered()
        
        self.seats_abreast_options      = [1, 2, 3, 4]
        self.seat_pitch                 = 38.0
        self.seat_width                 = 20.0
        self.aisle_width                = 20.0
        self.total_seats                = 0.0
        self.total_cargo_area           = 0.0
        self.total_seat_area            = 0.0
        self.sections                   = []

        self.areas = Data()
        self.areas.front_projected = 0.0
        self.areas.side_projected  = 0.0
        self.areas.wetted          = 0.0
        
        self.effective_diameter = 0.0
        self.width              = 0.0
        
        self.heights = Data()
        self.heights.maximum                        = 0.0
        self.heights.at_quarter_length              = 0.0
        self.heights.at_three_quarters_length       = 0.0
        self.heights.at_vertical_root_quarter_chord = 0.0
        
        self.lengths = Data()
        self.lengths.total      = 0.0
        self.lengths.fore_space = 0.0
        self.lengths.aft_space  = 0.0
             
        self.differential_pressure = 0.0


        # For VSP
        self.vsp_data                = Data()
        self.vsp_data.xsec_surf_id   = ''    # There is only one XSecSurf in each VSP geom.
        self.vsp_data.xsec_num       = None  # Number if XSecs in cabin geom.
        
        self.Segments           = SUAVE.Core.ContainerOrdered()
        
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
        

class Container(Physical_Component.Container):
    pass
        
        

# ------------------------------------------------------------
#  Handle Linking
# ------------------------------------------------------------

Cabin.Container = Container