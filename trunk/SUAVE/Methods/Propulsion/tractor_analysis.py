## @ingroup Methods-Propulsion
# tractor_analysis.py
# 
# Created:  September 2020, R. Erhard

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

import SUAVE
import copy
import math
import time
import scipy
import numpy as np
from SUAVE.Core import Units, Data
from SUAVE.Methods.Aerodynamics.Common.Fidelity_Zero.Lift.VLM import VLM


def tractor_analysis(vehicle, VLM_settings, conditions, ylocs, aoa_guess, omega_guess):
    
    #---------------------------------------------------------------------------------------------
    # Specify the analysis settings for the problem:
    #---------------------------------------------------------------------------------------------
    vehicle.propulsors.prop_net.propeller.analysis_settings.case = 'uniform_freestream' 
    vehicle.propulsors.prop_net.propeller.prop_loc[0] = -0.2
    
    VLM_settings = VLM_setup(conditions)
    
    #-----------------------------------------------------------------------------------------
    # Evaluate propeller performance along the span of the wing:
    #-----------------------------------------------------------------------------------------
    F_vals     = np.ones_like(ylocs)
    Q_vals     = np.ones_like(ylocs)
    P_vals     = np.ones_like(ylocs)
    Cp_vals    = np.ones_like(ylocs)    
    etap_vals  = np.ones_like(ylocs)
    omega_vals = np.ones_like(ylocs) 
    aoa_vals   = np.ones_like(ylocs) 
    CL_vals   = np.ones_like(ylocs) 
    CD_vals   = np.ones_like(ylocs) 
    
    # For two propellers
    vehicle.propulsors.prop_net.propeller.rotation = np.array([1,-1])      
    
    for i in range(len(ylocs)):
        
        # Update propeller y-location:
        vehicle.propulsors.prop_net.propeller.prop_loc[1] = ylocs[i] 
        prop_y_center    = np.array([vehicle.propulsors.prop_net.propeller.prop_loc[1]])
        vehicle.y_center = prop_y_center
        
        # Use Gauss-Seidel to converge on trimmed solution for propeller y-location:
        aoa_new, omega_new, converged, Lift, Drag, Thrust = gauss_seidel_optimization(conditions, vehicle, VLM_settings, aoa_guess, omega_guess)
        
        if converged:
            conditions.aerodynamics.angle_of_attack = np.array([[ aoa_new ]])
            vehicle.propulsors.prop_net.propeller.inputs.omega = np.array([[ omega_new ]])
            
            # Evaluate the propulsive performance of the trimmed solution:
            F, Q, P, Cp , outputs , etap = vehicle.propulsors.prop_net.propeller.spin(conditions,vehicle)  
            vehicle.propulsors.prop_net.propeller.outputs = outputs
            
            # Evaluate the wing for the trimmed solution:
            CL, CDi, CM, CL_wing, CDi_wing, cl_y , cdi_y , CP , VLM_outputs  = VLM(conditions, VLM_settings, vehicle) 
            CD_wing      = 0.012 + CDi_wing[0][0]   
     
            etap_vals[i]  = etap[0][0]
            omega_vals[i] = omega_new
            aoa_vals[i]   = aoa_new
            Cp_vals[i]    = Cp[0][0]
            Q_vals[i]     = Q[0][0]
            F_vals[i]     = F[0][0]
            P_vals[i]     = P[0][0]
            CL_vals[i]    = CL[0][0]
            CD_vals[i]    = CD_wing
        else:
            etap_vals[i]  = np.nan
            omega_vals[i] = np.nan
            aoa_vals[i]   = np.nan
            Cp_vals[i]    = np.nan
            Q_vals[i]     = np.nan
            F_vals[i]     = np.nan
            P_vals[i]     = np.nan   
            CL_vals[i]    = np.nan
            CD_vals[i]    = np.nan         
    
    plot_results = Data()
    plot_results.omegas     = omega_vals
    plot_results.etaps      = etap_vals
    plot_results.Qs         = Q_vals
    plot_results.ylocs      = ylocs
    plot_results.cl_y       = cl_y
    plot_results.CL_vals    = CL_vals
    plot_results.CD_vals    = CD_vals
    plot_results.aoa_vals   = aoa_vals
    plot_results.ylocs      = ylocs

    
    return plot_results



def gauss_seidel_optimization(conditions, vehicle, VLM_settings, aoa_guess, omega_guess):
    Weight       = vehicle.mass_properties.takeoff
    
    # Initial Variable Guess (from isolated case):
    #aoa_guess   = 4.56866447 * Units.deg
    #omega_guess = 306.93160108
    x0 = np.array([aoa_guess, omega_guess])

    # Set the design variables:
    conditions.aerodynamics.angle_of_attack = np.array([[ aoa_guess ]])
    vehicle.propulsors.prop_net.propeller.inputs.omega = np.array([[ omega_guess ]])
    
    # Spin propeller:
    F, Q, P, Cp , outputs , etap = vehicle.propulsors.prop_net.propeller.spin(conditions,vehicle)
    vehicle.propulsors.prop_net.propeller.outputs = outputs       

    N  = 30
    res_tot = np.zeros([N,1])
    res1    = np.zeros([N,1])
    res2    = np.zeros([N,1])
    Lift1   = np.zeros([N,1])
    Lift    = np.zeros([N,1])
    Lift2   = np.zeros([N,1])
    Drag    = np.zeros([N,1])
    Drag1   = np.zeros([N,1])
    Drag2   = np.zeros([N,1])
    Thrust  = np.zeros([N,1])
    Thrust1 = np.zeros([N,1])
    Thrust2 = np.zeros([N,1])
    des_pts = np.zeros([N,2])
    

    diff, tol, i, converged = 1, 1e-4, 0, False
    t_start = time.time()
    
    while diff>tol and i<N:
        #-----------------------------------------------------        
        # Wing Analysis
        #-----------------------------------------------------
              
        aoa_new = scipy.optimize.newton(residual_lift_equal_weight, aoa_guess, args=(conditions, VLM_settings,vehicle))
        res1[i], Lift1[i], Drag1[i], Thrust1[i] = residual(conditions, VLM_settings, vehicle)
        
        # Set the design variables:      
        aoa_guess = aoa_new
        conditions.aerodynamics.angle_of_attack = np.array([[ aoa_guess ]])  
        
        
        #-----------------------------------------------------
        # Propeller Analysis
        #-----------------------------------------------------
        omega_new = scipy.optimize.newton(residual_thrust_equal_drag, omega_guess, args=(Drag1[i], conditions, vehicle))
        res2[i], Lift2[i], Drag2[i], Thrust2[i] = residual(conditions,VLM_settings,vehicle)

        # Set the design variables:        
        omega_guess = omega_new
        vehicle.propulsors.prop_net.propeller.inputs.omega = np.array([[ omega_guess ]])           

        
        #-----------------------------------------------------        
        # Store and update design variables
        #-----------------------------------------------------        
        des_pts[i]  = np.array([aoa_guess, omega_guess])
        diff = res2[i]
        i = i +1
        
    if diff<=tol:
        converged = True
    
    t_elapsed = time.time() - t_start
    
    #counts = np.linspace(1,N,N)
    #fig  = plt.figure()
    #axes = fig.add_subplot(1,1,1)
    #axes.plot(counts,res2)
    #axes.set_xlabel('Count')
    #axes.set_ylabel('Residual for Thrust-Drag')
    
    #fig  = plt.figure()
    #axes = fig.add_subplot(1,1,1)
    #axes.plot(counts,res_tot)
    #axes.set_xlabel('Count')
    #axes.set_ylabel('Total Residual')
    
    #fig  = plt.figure()
    #axes = fig.add_subplot(1,1,1)
    #axes.plot(counts,res1)
    #axes.set_xlabel('Count')
    #axes.set_ylabel('Residual for Lift-Weight')     

    return aoa_guess, omega_guess, converged, Lift2[i-1], Drag2[i-1], Thrust2[i-1]

def residual_lift_equal_weight(aoa_guess, conditions, VLM_settings, vehicle):
    Weight = vehicle.mass_properties.takeoff
    conditions.aerodynamics.angle_of_attack = np.array([[ aoa_guess ]])  
    
    # Analyze the wing:
    CL, CDi, CM, CL_wing, CDi_wing, cl_y , cdi_y , CP , VLM_outputs  = VLM(conditions, VLM_settings, vehicle) 
    Lift = CL*0.5*conditions.freestream.density*vehicle.cruise_speed**2*vehicle.reference_area
    
    # Compute the residual:
    diff = abs(Lift[0][0] - Weight)
    
    return diff

def residual_thrust_equal_drag(omega_guess, Drag, conditions, vehicle ):
    vehicle.propulsors.prop_net.propeller.inputs.omega = np.array([[ omega_guess ]])
    
    # Spin propeller:
    F, Q, P, Cp , outputs , etap = vehicle.propulsors.prop_net.propeller.spin(conditions,vehicle)
    vehicle.propulsors.prop_net.propeller.outputs = outputs
    Thrust = 2*F
    
    # Compute the residual:
    diff = abs(Thrust[0][0] - Drag[0])
    
    return diff


def residual(conditions, VLM_settings,vehicle):
    # Computes the total residual for the current parameters
    Weight = vehicle.mass_properties.takeoff
    
    # Spin propeller:
    F, Q, P, Cp , outputs , etap = vehicle.propulsors.prop_net.propeller.spin(conditions,vehicle)
    vehicle.propulsors.prop_net.propeller.outputs = outputs 
    Thrust = 2*F
    
    # Compute new drag due to new aoa_guess:
    CL, CDi, CM, CL_wing, CDi_wing, cl_y , cdi_y , CP , VLM_outputs  = VLM(conditions, VLM_settings, vehicle) 
    CD_wing  = 0.012 + CDi_wing[0][0]
    Drag     = CD_wing*0.5*conditions.freestream.density*vehicle.cruise_speed**2*vehicle.reference_area 
    Lift     = CL*0.5*conditions.freestream.density*vehicle.cruise_speed**2*vehicle.reference_area 
    
    constraint_val = (abs(Lift-Weight) +abs(Thrust-Drag))
    return constraint_val[0][0], Lift, Drag, Thrust


def VLM_setup(conditions):
    # --------------------------------------------------------------------------------
    #          Settings for VLM:  
    # --------------------------------------------------------------------------------
    vortices            = conditions.vortices
    VLM_settings        = Data()
    VLM_settings.number_panels_spanwise   = vortices **2
    VLM_settings.number_panels_chordwise  = vortices
    
    # Default is no slipstream:
    VLM_settings.use_surrogate             = False
    VLM_settings.include_slipstream_effect = True  
    
    return VLM_settings    
