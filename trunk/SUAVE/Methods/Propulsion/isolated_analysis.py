## @ingroup Methods-Propulsion
# isolated_analysis.py
# 
# Created:  September 2020, R. Erhard

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------
import SUAVE
import copy
import math
import scipy
import numpy as np
from SUAVE.Core import Units, Data
from SUAVE.Methods.Aerodynamics.Common.Fidelity_Zero.Lift.VLM import VLM
 
def isolated_analysis(vehicle, conditions, omega_guess = 2200*Units.rpm):
    #------------------------------------------------------------
    # Change analysis setting case to be uniform freestream:
    #------------------------------------------------------------
    vehicle.propulsors.prop_net.propeller.analysis_settings.case = 'uniform_freestream'
    VLM_settings = VLM_setup(conditions)
    
    Weight = vehicle.mass_properties.takeoff
    
    #-------------------------------------------------------------------
    # Find alpha to converge on L=W for isolated wing
    #-------------------------------------------------------------------
    aoa_guess = conditions.aerodynamics.angle_of_attack
    aoa_new = scipy.optimize.newton(residual_lift_equal_weight, aoa_guess[0][0], args=(conditions,VLM_settings,vehicle))
    conditions.aerodynamics.angle_of_attack = np.array([[ aoa_new ]])
    
    # Lift and Drag for Isolated wing:    
    CL, CDi, CM, CL_wing, CDi_wing, cl_y , cdi_y , CP, VLM_outputs  = VLM(conditions, VLM_settings, vehicle) 
    CD_wing      = 0.012 + CDi_wing[0][0]   
    Drag_iso     = CD_wing*0.5*conditions.freestream.density*conditions.freestream.velocity**2*vehicle.reference_area 
    Lift_iso     = CL*0.5*conditions.freestream.density*conditions.freestream.velocity**2*vehicle.reference_area 
    
    #-------------------------------------------------------------------    
    # Now, we find omega to meet Thrust = Drag_iso
    #-------------------------------------------------------------------
    omega_guess = np.array([[ omega_guess  ]])
    omega_new = scipy.optimize.newton(residual_thrust_equal_drag, omega_guess[0][0], args=(Drag_iso[0],conditions,vehicle))
    vehicle.propulsors.prop_net.propeller.inputs.omega = np.array([[ omega_new ]])
    
    F, Q, P, Cp , outputs , etap = vehicle.propulsors.prop_net.propeller.spin(conditions,vehicle)
    vehicle.propulsors.prop_net.propeller.outputs = outputs
    
    iso_results = Data()
    iso_results.omega_Iso = omega_new
    iso_results.aoa_Iso   = aoa_new
    iso_results.Q_Iso     = Q[0][0]
    iso_results.power     = P[0][0]
    iso_results.etap_Iso  = etap[0][0]
    iso_results.CL_iso    = CL[0][0]
    iso_results.CD_iso    = CD_wing
    iso_results.J         = conditions.freestream.velocity/((2/(2*np.pi))*omega_new*vehicle.propulsors.prop_net.propeller.tip_radius*2)        
    
    return iso_results, Drag_iso, aoa_new, omega_new



def residual_lift_equal_weight(aoa_guess, conditions, VLM_settings, vehicle):
    Weight = vehicle.mass_properties.takeoff
    conditions.aerodynamics.angle_of_attack = np.array([[ aoa_guess ]])  
    
    # Analyze the wing:
    CL, CDi, CM, CL_wing, CDi_wing, cl_y , cdi_y , CP, VLM_outputs  = VLM(conditions, VLM_settings, vehicle) 
    Lift = CL*0.5*conditions.freestream.density*conditions.freestream.velocity**2*vehicle.reference_area
    
    # Compute the residual:
    diff = abs(Lift[0][0] - Weight)
    
    return diff

def residual_thrust_equal_drag(omega_guess, Drag, conditions, vehicle ):
    vehicle.propulsors.prop_net.propeller.inputs.omega = np.array([[ omega_guess ]])
    
    # Spin propeller:
    F, Q, P, Cp , outputs , etap = vehicle.propulsors.prop_net.propeller.spin(conditions,vehicle)
    vehicle.propulsors.prop_net.propeller.outputs = outputs
    Thrust = vehicle.propulsors.prop_net.number_of_engines*F
    
    # Compute the residual:
    diff = abs(Thrust[0][0] - Drag[0])
    
    return diff

def VLM_setup(conditions):
    # --------------------------------------------------------------------------------
    #          Settings for VLM:  
    # --------------------------------------------------------------------------------
    vortices            = conditions.vortices
    VLM_settings        = Data()
    VLM_settings.number_spanwise_vortices   = vortices **2
    VLM_settings.number_chordwise_vortices  = vortices
    
    # Default is no slipstream:
    VLM_settings.use_surrogate             = True
    VLM_settings.propeller_wake_model      = False
    VLM_settings.wake_development_time     = 0.05
    
    return VLM_settings   