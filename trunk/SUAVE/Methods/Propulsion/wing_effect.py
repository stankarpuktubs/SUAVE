## @ingroup Methods-Propulsion
# VLM.py
# 
# Created:  September 2020, R. Erhard

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------
import SUAVE
import math
import copy
import numpy as np
from SUAVE.Core import Units, Data
from SUAVE.Methods.Aerodynamics.Common.Fidelity_Zero.Lift.VLM import VLM
from SUAVE.Methods.Aerodynamics.Common.Fidelity_Zero.Lift.compute_induced_velocity_matrix import compute_induced_velocity_matrix


def wing_effect(vehicle,conditions):
    """
     Appends the induced velocities at the propeller plane from the wing as components of the vehicle
    """     
    
    #-------------------------------------------------------------------------
    #          Extracting variables:
    #-------------------------------------------------------------------------
    R_tip    = vehicle.propulsors.prop_net.propeller.tip_radius
    r_hub    = vehicle.propulsors.prop_net.propeller.hub_radius    
    prop_loc = vehicle.propulsors.prop_net.propeller.prop_loc
    N        = conditions.N    
    aoa      = conditions.aerodynamics.angle_of_attack
    mach     = conditions.freestream.mach_number
    
    prop_x_center = np.array([vehicle.wings.main_wing.origin[0][0] + prop_loc[0]])  
        
    # --------------------------------------------------------------------------------
    #          Settings and Calling for VLM:  
    # --------------------------------------------------------------------------------
    vortices            = conditions.vortices
    VLM_settings        = Data()
    VLM_settings.number_panels_spanwise   = vortices **2
    VLM_settings.number_panels_chordwise  = vortices
    VLM_settings.use_surrogate             = True
    VLM_settings.include_slipstream_effect = False     

    CL, CDi, CM, CL_wing, CDi_wing, cl_y , cdi_y , CP, VLM_outputs   = VLM(conditions, VLM_settings, vehicle)
    VD       = copy.deepcopy(VLM_outputs.VD)
    gammaT   = VLM_outputs.gamma
    
    # Change settings to allow for more z-direction evaluation points by changing VD
    zlocs = np.linspace(vehicle.z_center-1.2*R_tip,vehicle.z_center+1.2*R_tip,VLM_settings.number_panels_chordwise)
    VLM_settings_copy = copy.deepcopy(VLM_settings)
    # Declare new evaluation points:
    # Keep spanwise locations, but add new ones outside of the wingtip to capture the effect over there without interfering with vortices
    y     = np.reshape(VD.YC,(2*vortices**2,vortices))
    y     = y[y[:,0]<0]
    half_span = vehicle.wings.main_wing.spans.projected/2
    xn    = 2*len(y)
    
    cp_YC =  np.append(y,np.flipud(y[-1]-(half_span+y))) #VD.YC #np.linspace(-1.3*vehicle.wings.main_wing.spans.projected,1.3*vehicle.wings.main_wing.spans.projected, len(VD.YC) ) #VD.YC             # Same evaluation point values, repeated to allow for more z-evaluations
    cp_ZC = np.tile(zlocs,xn)
    cp_XC = np.ones_like(cp_YC)*prop_x_center
    
    # Need to parse it so there are only n_sw*n_cw*2  control points per evaluation of computing induced velocities:
    count  = 0
    max_val_per_loop = 2*VLM_settings.number_panels_spanwise*VLM_settings.number_panels_chordwise
    num_loops_required = math.ceil(len(cp_XC)/(max_val_per_loop))
    remainder = len(cp_XC)%(2*VLM_settings.number_panels_spanwise*VLM_settings.number_panels_chordwise)

    u  = np.zeros_like(cp_YC)
    v  = np.zeros_like(cp_YC)
    w  = np.zeros_like(cp_YC)
    
    for i in range(num_loops_required):    
        VD.XC = cp_XC[count:count+max_val_per_loop]
        VD.YC = cp_YC[count:count+max_val_per_loop]
        VD.ZC = cp_ZC[count:count+max_val_per_loop]   
        
        # Compute induced velocity for the given yz plane:
        C_mn, DW_mn = compute_induced_velocity_matrix(VD,VLM_settings.number_panels_spanwise,VLM_settings.number_panels_chordwise,aoa,mach)   
        MCM = VD.MCM      
    
        u[count:count+max_val_per_loop] = (C_mn[:,:,:,0]*MCM[:,:,:,0]@gammaT)[:,:,0]
        v[count:count+max_val_per_loop] = (C_mn[:,:,:,1]*MCM[:,:,:,1]@gammaT)[:,:,0]
        w[count:count+max_val_per_loop] = (C_mn[:,:,:,2]*MCM[:,:,:,2]@gammaT)[:,:,0] 
        
        count = count +max_val_per_loop
    
    VD.XC = cp_XC
    VD.YC = cp_YC
    VD.ZC = cp_ZC
    vehicle.propulsors.prop_net.propeller.disturbed_u = u #np.reshape(u,(2*VLM_settings.number_panels_spanwise,VLM_settings.number_panels_chordwise))
    vehicle.propulsors.prop_net.propeller.disturbed_v = v #np.reshape(v,(2*VLM_settings.number_panels_spanwise,VLM_settings.number_panels_chordwise))
    vehicle.propulsors.prop_net.propeller.disturbed_w = w #np.reshape(w,(2*VLM_settings.number_panels_spanwise,VLM_settings.number_panels_chordwise))

    vehicle.VD = VD
                           
    return vehicle
