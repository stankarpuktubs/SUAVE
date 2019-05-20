## @ingroup Methods-Aerodynamics-Fidelity_Zero-Lift
# wing_compressibility_correction.py
# 
# Created:  Dec 2013, A. Variyar 
# Modified: Feb 2014, A. Variyar, T. Lukaczyk, T. Orra 
#           Apr 2014, A. Variyar
#           Jan 2015, E. Botero

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

import numpy as np

# ----------------------------------------------------------------------
#  The Function
# ----------------------------------------------------------------------

## @ingroup Methods-Aerodynamics-Fidelity_Zero-Lift
def wing_compressibility_correction(state,settings,geometry):
    """Corrects a wings lift based on compressibility

    Assumptions:
    subsonic

    Source:
    https://stanford.edu/~cantwell/AA200_Course_Material/AA200_Course_Notes/
    
    Inputs:
    settings.fuselage_lift_correction  [-]
    state.conditions.
      freestream.mach_number           [-]
      aerodynamics.angle_of_attack     [radians]
      aerodynamics.lift_coefficient    [-]

    Outputs:
    state.conditions.aerodynamics.
      lift_breakdown.compressible_wings [-] CL for the wings
      lift_coefficient                  [-]
    wings_lift_comp                     [-]

    Properties Used:
    N/A
    """     
   
    # unpack
    fus_correction = settings.fuselage_lift_correction
    Mc             = state.conditions.freestream.mach_number
    AoA            = state.conditions.aerodynamics.angle_of_attack
    wings_lift     = state.conditions.aerodynamics.lift_coefficient
    
    # for 3D compressibility correction
    # Need this but this isn't the right call; also files that call this aren't passing geometry to it (Fidelity_Zero.py, line 100)
    span = geometry.wing.spans.projected # wrong; fix this
    Sref = geometry.wing.areas.reference # wrong; fix this 
    
    # compressibility correction
    beta = np.sqrt(1.-Mc**2.)
    AR = span**2/Sref
    compress_corr = (2/AR)/(beta+2/AR) #3D compressibility correction
    
    # 2D compressibility correction
    compress_corr_2D = 1./(beta) #2D compressibility correction
    
    
    # correct lift
    wings_lift_comp = wings_lift * compress_corr
    
    state.conditions.aerodynamics.lift_breakdown.compressible_wings = wings_lift_comp
    state.conditions.aerodynamics.lift_coefficient= wings_lift_comp

    return wings_lift_comp
