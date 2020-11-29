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

def tractor_analysis(vehicle, conditions, ylocs,Drag_iso):  
    
    # create data structure 
    VLM_settings                           = Data()
    VLM_settings.number_spanwise_vortices  = 25 # vortices **2
    VLM_settings.number_chordwise_vortices = 5 #  vortices 
    VLM_settings.use_surrogate             = False
    VLM_settings.propeller_wake_model      = True  
        
    
    #-----------------------------------------------------------------------------------------
    # Evaluate propeller performance along the span of the wing:
    #-----------------------------------------------------------------------------------------
    F_vals     = np.ones_like(ylocs)
    Q_vals     = np.ones_like(ylocs)
    P_vals     = np.ones_like(ylocs)
    Cp_vals    = np.ones_like(ylocs)    
    etap_vals  = np.ones_like(ylocs)
    omega_vals = np.ones_like(ylocs) 
    AoA_vals   = np.ones_like(ylocs) 
    CL_vals    = np.ones_like(ylocs) 
    CDi_vals   = np.ones_like(ylocs) 
    
    for i in range(len(ylocs)):
        # Update propeller y-location:
        vehicle.propulsors.prop_net.propeller.prop_loc[1] = ylocs[i] 
        prop_y_center    = np.array([vehicle.propulsors.prop_net.propeller.prop_loc[1]])
        vehicle.prop_y_center = prop_y_center
        
        # ---------------------------------------------------------------------
        # optimizaion 
        # ---------------------------------------------------------------------   
        # Determine the isolated performance of the wing and propeller in a steady and level condition:  
        iso_results, Drag_iso, aoa_iso, omega_iso = isolated_analysis(vehicle, conditions)
        
        # ---------------------------------------------------------------------
        # optimizaion 
        # ---------------------------------------------------------------------       
        # bounds of variables
        aoa_lb   = 0.0 * Units.degrees
        aoa_ub   = 10  * Units.degrees 
        omega_lb = 1500 * Units.rpm
        omega_ub = 2500 * Units.rpm
        
        args  = (VLM_settings, conditions, vehicle) 
        cons1 = [{'type':'eq', 'fun': residual_lift_equal_weight,'args': args},
                 {'type':'eq', 'fun': residual_thrust_equal_drag,'args': args}] 
        
        bnds  = ((aoa_lb,aoa_ub), (omega_lb , omega_ub))
        sol   = minimize(objective, [ 1. * Units.degrees , 1800 * Units.rpm], args, method='SLSQP', bounds=bnds, tol=1e-6, constraints=cons1) 
        
        # optimized values 
        AoA   = sol.x[0]
        Omega = sol.x[1]   
        
        # --------------------------------------------------------------------- 
        # Evaluate the aerodynamic and propulsive performance of the trimmed solution 
        # --------------------------------------------------------------------- 
        vehicle.propulsors.prop_net.propeller.inputs.omega = np.array([[ Omega ]]) 
        conditions.aerodynamics.angle_of_attack = np.array([[ AoA]])  
        
        # spin the propeller 
        F, Q, P, Cp , outputs , etap = vehicle.propulsors.prop_net.propeller.spin(conditions,vehicle)    
         
        # run VLM
        CL, CDi, CM, CL_wing, CDi_wing, cl_y , cdi_y , CP, VLM_outputs  = VLM(conditions, VLM_settings, vehicle)   
              
        etap_vals[i]  = etap[0][0]
        omega_vals[i] = Omega
        Cp_vals[i]    = Cp[0][0]
        Q_vals[i]     = Q[0][0]
        F_vals[i]     = F[0][0]
        P_vals[i]     = P[0][0]
        AoA_vals[i]   = AoA[0][0]
        CL_vals[i]    = CL[0][0]
        CDi_vals[i]   = (CDi  + 0.012)[0][0]
        
    plot_results = Data()
    plot_results.omegas     = omega_vals
    plot_results.etaps      = etap_vals
    plot_results.Qs         = Q_vals
    plot_results.ylocs      = ylocs
    plot_results.AoAs       = AoA_vals
    plot_results.powers     = P_vals
    plot_results.CLs        = CL_vals
    plot_results.CDis       = CDi_vals
    
    return plot_results  


def objective(x, VLM_settings, conditions, vehicle ):  
    omega  = x[1]  
    
    # update values
    vehicle.propulsors.prop_net.propeller.inputs.omega = np.array([[omega]]) 
    
    # run propeller model
    F, Q, P, Cp , outputs , etap = vehicle.propulsors.prop_net.propeller.spin(conditions,vehicle)   
     
    return P

# --------------------------------------------------------------------------------
#         Lift-Weight Residual
# --------------------------------------------------------------------------------
def residual_lift_equal_weight(x, VLM_settings, conditions, vehicle):
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
def residual_thrust_equal_drag(x, VLM_settings, conditions, vehicle ):
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
    drag_residual = abs(Thrust[0][0] - Drag[0])
    
    return drag_residual 