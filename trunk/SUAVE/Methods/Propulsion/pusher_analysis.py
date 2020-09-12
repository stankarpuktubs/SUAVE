## @ingroup Methods-Propulsion
# tractor_analysis.py
# 
# Created:  September 2020, R. Erhard

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------
import SUAVE
import scipy
import numpy as np
from SUAVE.Core import Units, Data
from SUAVE.Methods.Propulsion.wing_effect import wing_effect
from SUAVE.Methods.Propulsion.isolated_analysis import residual_thrust_equal_drag
from SUAVE.Methods.Propulsion.isolated_analysis import isolated_analysis

def pusher_analysis(vehicle, conditions, ylocs,Drag_iso):
    # Evaluates propeller performance for the given vehicle
    
    # Set VLM settings to use surrogate instead of slipstream:
    VLM_settings = VLM_setup(conditions)
    
    # First find alpha to meet L=W: (unless drag_iso is already passed in)
    #iso_results, Drag_iso, aoa_iso, omega_iso = isolated_analysis(vehicle, VLM_settings, conditions)
    
    #--------------------------------------------------------------
    # Specify the analysis settings for the problem:
    #--------------------------------------------------------------
    vehicle.propulsors.prop_net.propeller.analysis_settings.case = 'disturbed_freestream'   
    vehicle.propulsors.prop_net.propeller.prop_loc[0] = vehicle.wings.main_wing.chords.root + 0.2
    VLM_settings.use_surrogate             = True
    VLM_settings.include_slipstream_effect = False
    
    # Compute the propeller-plane flow field in the wing wake:
    vehicle  = wing_effect(vehicle,conditions)
    
    #-----------------------------------------------------------------------------------------
    # Evaluate propeller performance along the span of the wing:
    #-----------------------------------------------------------------------------------------
    F_vals     = np.ones_like(ylocs)
    Q_vals     = np.ones_like(ylocs)
    P_vals     = np.ones_like(ylocs)
    Cp_vals    = np.ones_like(ylocs)    
    etap_vals  = np.ones_like(ylocs)
    omega_vals = np.ones_like(ylocs) 
    
    for i in range(len(ylocs)):
        # Update propeller y-location:
        vehicle.propulsors.prop_net.propeller.prop_loc[1] = ylocs[i] 
        prop_y_center    = np.array([vehicle.propulsors.prop_net.propeller.prop_loc[1]])
        vehicle.y_center = prop_y_center
        
        # Find omega to produce the thrust that equals the wing drag:
        omega_guess = vehicle.propulsors.prop_net.propeller.inputs.omega
        omega_new =  scipy.optimize.newton(residual_thrust_equal_drag, omega_guess[0][0], args=(Drag_iso[0],conditions,vehicle))
        vehicle.propulsors.prop_net.propeller.inputs.omega = np.array([[ omega_new ]])
        
        # Evaluate the propulsive performance of the trimmed solution:
        F, Q, P, Cp , outputs , etap = vehicle.propulsors.prop_net.propeller.spin(conditions,vehicle)          
        
        etap_vals[i]  = etap[0][0]
        omega_vals[i] = omega_new
        Cp_vals[i]    = Cp[0][0]
        Q_vals[i]     = Q[0][0]
        F_vals[i]     = F[0][0]
        P_vals        = P[0][0]
    
    plot_results = Data()
    plot_results.omegas     = omega_vals
    plot_results.etaps      = etap_vals
    plot_results.Qs         = Q_vals
    plot_results.ylocs      = ylocs
    
    return plot_results


def VLM_setup(conditions):
    # --------------------------------------------------------------------------------
    #          Settings for VLM:  
    # --------------------------------------------------------------------------------
    vortices            = conditions.vortices
    VLM_settings        = Data()
    VLM_settings.number_panels_spanwise   = vortices **2
    VLM_settings.number_panels_chordwise  = vortices
    
    # Default is no slipstream:
    VLM_settings.use_surrogate             = True
    VLM_settings.include_slipstream_effect = False
    return VLM_settings   