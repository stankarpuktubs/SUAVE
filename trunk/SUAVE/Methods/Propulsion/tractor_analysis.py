## @ingroup Methods-Propulsion
# tractor_analysis.py
# 
# Created:  September 2020, M. Clarke

# ----------------------------------------------------------------------
#  Imports
# ---------------------------------------------------------------------- 
import numpy as np
from SUAVE.Core import Units, Data  
from SUAVE.Methods.Propulsion.isolated_analysis import isolated_analysis
from scipy.optimize import minimize 
from SUAVE.Methods.Aerodynamics.Common.Fidelity_Zero.Lift.VLM import VLM 
import pickle

def tractor_cruise_optimization(vehicle, conditions,Nprops ):  
    
    # create data structure 
    VLM_settings                                 = Data()
    VLM_settings.number_spanwise_vortices        = 25  
    VLM_settings.number_chordwise_vortices       = 5  
    VLM_settings.use_surrogate                   = False
    VLM_settings.propeller_wake_model            = True   
    VLM_settings.wake_development_time           = 0.05
     
    # Determine the isolated performance of the wing and propeller in a steady and level condition:
    if Nprops > 10: 
        omega_guess = 1750*Units.rpm 
    else:
        omega_guess = 1000*Units.rpm 
    iso_results, Drag_iso, aoa_iso, omega_iso = isolated_analysis(vehicle, conditions,omega_guess)
     
    # optimizaion  
    # bounds of variables
    aoa_lb   = 0.0 * Units.degrees
    aoa_ub   = 10  * Units.degrees 
    omega_lb = 100 * Units.rpm
    omega_ub = 2500 * Units.rpm
    
    args  = (VLM_settings, conditions, vehicle) 
    cons = [{'type':'eq', 'fun': cruise_residual_lift_equal_weight,'args': args},
            {'type':'eq', 'fun': cruise_residual_thrust_equal_drag,'args': args}] 
    
    bnds  = ((aoa_lb,aoa_ub), (omega_lb , omega_ub))
    tolerance  = 1e-3
    sol   = minimize(cruise_objective, [ 6. * Units.degrees , 600 * Units.rpm], args  = (VLM_settings, conditions, vehicle) , method='SLSQP', bounds=bnds, tol= tolerance, constraints=cons) 
    
    # optimized values 
    AoA   = sol.x[0]
    Omega = sol.x[1]   
     
    # Evaluate the aerodynamic and propulsive performance of the trimmed solution  
    vehicle.propulsors.prop_net.propeller.inputs.omega = np.array([[ Omega ]]) 
    conditions.aerodynamics.angle_of_attack = np.array([[ AoA]])  
    
    # spin the propeller 
    F, Q, P, Cp , outputs , etap = vehicle.propulsors.prop_net.propeller.spin(conditions,vehicle)    
     
    # run VLM
    CL, CDi, CM, CL_wing, CDi_wing, cl_y , cdi_y , CP, VLM_outputs  = VLM(conditions, VLM_settings, vehicle)   

    # Determine the isolated performance of the wing and propeller in a steady and level condition:      
    iso_results, Drag_iso, aoa_iso, omega_iso = isolated_analysis(vehicle, conditions)
    
    results = Data()
    results.omega     = Omega
    results.etap      = etap[0][0]
    results.Q         = Q[0][0]
    results.AoA       = AoA 
    results.power     = P[0][0]
    results.CL        = CL[0][0]
    results.CDi       = (CDi  + 0.012)[0][0] 
    results.etap_tot  = iso_results.etap_Iso
    results.L_to_D    = results.CL/results.CDi
    
    # save results in pickle file
    filename = 'Tractor_Cruise_Res_' + str(Nprops) + '_Props'
    save_results(results,filename) 
    
    return  results 

def tractor_climb_optimization(vehicle, conditions,Nprops,aoa_range ):  
    
    # create data structure 
    VLM_settings                           = Data()
    VLM_settings.number_spanwise_vortices  = 25  
    VLM_settings.number_chordwise_vortices = 5  
    VLM_settings.use_surrogate             = False
    VLM_settings.propeller_wake_model      = True
    VLM_settings.wake_development_time     = 0.05
    
    results = Data()
    results.omega     = np.zeros(len(aoa_range))
    results.etap      = np.zeros(len(aoa_range))
    results.Q         = np.zeros(len(aoa_range))
    results.AoA       = np.zeros(len(aoa_range))
    results.power     = np.zeros(len(aoa_range))
    results.CL        = np.zeros(len(aoa_range))
    results.CDi       = np.zeros(len(aoa_range))
    results.etap_tot  = np.zeros(len(aoa_range))
    results.L_to_D    = np.zeros(len(aoa_range))
    
    for i in range(len(aoa_range)):
        
        # Determine the isolated performance of the wing and propeller in a steady and level condition:
        if Nprops > 10: 
            omega_guess = 1750*Units.rpm 
        else:
            omega_guess = 1000*Units.rpm     
        iso_results, Drag_iso, aoa_iso, omega_iso = isolated_analysis(vehicle, conditions,omega_guess)
        
        AoA = aoa_range[i]
        conditions.aerodynamics.angle_of_attack = np.array([[aoa_range[i]]])     
        
        # bounds of variables 
        omega_lb = 100 * Units.rpm
        omega_ub = 2500 * Units.rpm
        
        args  = (VLM_settings, conditions, vehicle) 
        cons = [{'type':'eq', 'fun': climb_residual_thrust_equal_drag,'args': args}] 
        
        bnds  = ((omega_lb , omega_ub),)
        tolerance  = 1e-3
        x0 = (600 * Units.rpm)
        sol2   = minimize(climb_objective, x0 , args  = (VLM_settings, conditions, vehicle) , method='SLSQP', bounds=bnds, tol= tolerance, constraints=cons) 
        
        # optimized values 
        Omega  = sol2.x[0]   
         
        # Evaluate the aerodynamic and propulsive performance of the trimmed solution  
        vehicle.propulsors.prop_net.propeller.inputs.omega = np.array([[ Omega ]]) 
        conditions.aerodynamics.angle_of_attack = np.array([[ AoA]])  
        
        # spin the propeller 
        F, Q, P, Cp , outputs , etap = vehicle.propulsors.prop_net.propeller.spin(conditions,vehicle)    
         
        # run VLM
        CL, CDi, CM, CL_wing, CDi_wing, cl_y , cdi_y , CP, VLM_outputs  = VLM(conditions, VLM_settings, vehicle)   
         
        results.omega[i]     = Omega
        results.etap[i]      = etap[0][0]
        results.Q[i]         = Q[0][0]
        results.AoA[i]       = AoA 
        results.power[i]     = P[0][0]
        results.CL[i]        = CL[0][0]
        results.CDi[i]       = (CDi  + 0.012)[0][0] 
        results.etap_tot[i]  = iso_results.etap_Iso
        results.L_to_D[i]    = results.CL[i]/results.CDi[i]
    
    # save results in pickle file
    filename = 'Tractor_Climb_Res_' + str(Nprops) + '_Props'
    save_results(results,filename) 
    
    return  results

def cruise_objective(x, VLM_settings, conditions, vehicle ):  
    omega  = x[1]  
    
    # update values
    vehicle.propulsors.prop_net.propeller.inputs.omega = np.array([[omega]]) 
    
    # run propeller model
    F, Q, P, Cp , outputs , etap = vehicle.propulsors.prop_net.propeller.spin(conditions,vehicle)   
    
    power  = P[0][0] 
    return power  

# --------------------------------------------------------------------------------
#         Lift-Weight Residual
# --------------------------------------------------------------------------------
def cruise_residual_lift_equal_weight(x, VLM_settings, conditions, vehicle):
    aoa    = x[0]
    omega  = x[1] 
    
    Weight = vehicle.mass_properties.takeoff
    
    # update values
    vehicle.propulsors.prop_net.propeller.inputs.omega = np.array([[omega]])
    conditions.aerodynamics.angle_of_attack = np.array([[ aoa ]])  
    
    # run propeller model
    _, _, _, _ , _ , _ = vehicle.propulsors.prop_net.propeller.spin(conditions,vehicle)     
    
    # run VLM
    CL, CDi, CM, CL_wing, CDi_wing, cl_y , cdi_y , CP, VLM_outputs  = VLM(conditions, VLM_settings, vehicle)  
    Lift = CL*0.5*(conditions.freestream.density*conditions.freestream.velocity**2)*vehicle.reference_area
    
    # Compute the residual:
    lift_residual = abs(Lift[0][0] - Weight)   
    
    return lift_residual


# --------------------------------------------------------------------------------
#         Thrust-Drag Residual
# --------------------------------------------------------------------------------
def cruise_residual_thrust_equal_drag(x, VLM_settings, conditions, vehicle ):
    aoa    = x[0]
    omega  = x[1]  
    
    # update values
    vehicle.propulsors.prop_net.propeller.inputs.omega = np.array([[omega]])
    conditions.aerodynamics.angle_of_attack = np.array([[ aoa ]])  
    
    # run propeller model
    F, Q, P, Cp , outputs , etap = vehicle.propulsors.prop_net.propeller.spin(conditions,vehicle)     
    Thrust = vehicle.propulsors.prop_net.number_of_engines*F
    
    # run VLM
    CL, CDi, CM, CL_wing, CDi_wing, cl_y , cdi_y , CP, VLM_outputs  = VLM(conditions, VLM_settings, vehicle) 
    Drag = (CDi  + 0.012)*0.5*(conditions.freestream.density*conditions.freestream.velocity**2)*vehicle.reference_area
    
    # compute residual 
    drag_residual = abs(Thrust[0][0] - Drag[0][0])  
    return drag_residual 


def climb_objective(x, VLM_settings, conditions, vehicle ):  
    omega  = x[0]  
    
    # update values
    vehicle.propulsors.prop_net.propeller.inputs.omega = np.array([[omega]]) 
    
    # run propeller model
    F, Q, P, Cp , outputs , etap = vehicle.propulsors.prop_net.propeller.spin(conditions,vehicle)   
    
    power  = P[0][0] 
    return power    

# --------------------------------------------------------------------------------
#         Thrust-Drag Residual
# --------------------------------------------------------------------------------
def climb_residual_thrust_equal_drag(x, VLM_settings, conditions, vehicle ): 
    omega  = x[0]  
    
    # update values
    vehicle.propulsors.prop_net.propeller.inputs.omega = np.array([[omega]]) 
    
    # run propeller model
    F, Q, P, Cp , outputs , etap = vehicle.propulsors.prop_net.propeller.spin(conditions,vehicle)     
    Thrust = vehicle.propulsors.prop_net.number_of_engines*F
    
    # run VLM
    CL, CDi, CM, CL_wing, CDi_wing, cl_y , cdi_y , CP, VLM_outputs  = VLM(conditions, VLM_settings, vehicle) 
    Drag = (CDi  + 0.012)*0.5*(conditions.freestream.density*conditions.freestream.velocity**2)*vehicle.reference_area
    
    # compute residual 
    drag_residual = abs(Thrust[0][0] - Drag[0][0])  
    return drag_residual 

def save_results(results,filename):
   
    save_file = filename + '.pkl'
    with open(save_file, 'wb') as file:
        pickle.dump(results, file)
        
    return  
