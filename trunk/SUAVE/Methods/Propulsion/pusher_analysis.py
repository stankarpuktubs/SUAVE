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

import pylab as plt

def pusher_analysis(vehicle, conditions, ylocs,Drag_iso):
    # Evaluates propeller performance for the given vehicle
    
    # Set VLM settings to use surrogate instead of slipstream:
    VLM_settings                           = VLM_setup(conditions)
    VLM_settings.use_surrogate             = False
    VLM_settings.include_slipstream_effect = False    
    
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
        vehicle.prop_y_center = prop_y_center
        
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
        P_vals[i]     = P[0][0]
    
    plot_results = Data()
    plot_results.omegas     = omega_vals
    plot_results.etaps      = etap_vals
    plot_results.Qs         = Q_vals
    plot_results.ylocs      = ylocs
    plot_results.powers     = P_vals
    
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



def plot_disks(vehicle,outputs):
    # Setting Latex Font style
    import matplotlib
    font = {'family' : 'normal',
               'weight' : 'normal',
               'size'   : 22}
    
    matplotlib.rc('font', **font)
    matplotlib.rcParams['mathtext.fontset'] = 'stix'
    matplotlib.rcParams['font.family'] = 'STIXGeneral'
    matplotlib.rc('lines', lw=3)
    
    # Now plotting:
    psi   = outputs.disc_azimuthal_distribution[0,:,:] # outputs.azimuthal_distribution_2d[0,:,:] #
    r     = outputs.disc_radial_distribution[0,:,:] #  outputs.blade_radial_distribution_normalized_2d[0,:,:] #
    
    # Adjust so that the hub is included in the plot:
    rh = vehicle.propulsors.prop_net.propeller.hub_radius
    
    fig0, axis0 = plt.subplots(subplot_kw=dict(projection='polar'))
    CS_0 = axis0.contourf(psi, r, outputs.disc_axial_velocity[0]/outputs.velocity[0][0],100,cmap=plt.cm.jet)  # (psi, r, outputs.axial_velocity_distribution_2d[0]/outputs.velocity[0][0],100,cmap=plt.cm.jet)    #
    cbar0 = plt.colorbar(CS_0, ax=axis0, format=matplotlib.ticker.FormatStrFormatter('%.2f'))
    cbar0.ax.set_ylabel('$\dfrac{V_a}{V_\infty}$',rotation=0,labelpad=25)
    axis0.set_title('Axial Velocity of Propeller',pad=15) 
    thetaticks = np.arange(0,360,45)
    axis0.set_thetagrids(thetaticks, frac=5.2)    
    axis0.set_rorigin(-rh)
    axis0.set_yticklabels([])
    
    fig0, axis0 = plt.subplots(subplot_kw=dict(projection='polar'))
    CS_0 = axis0.contourf(psi, r, outputs.disc_tangential_velocity[0]/outputs.velocity[0][0],100,cmap=plt.cm.jet) #    (psi, r, outputs.tangential_velocity_distribution_2d[0]/outputs.velocity[0][0],100,cmap=plt.cm.jet) #
    cbar0 = plt.colorbar(CS_0, ax=axis0,format=matplotlib.ticker.FormatStrFormatter('%.2f'))
    cbar0.ax.set_ylabel('$\dfrac{V_t}{V_\infty}$',rotation=0,labelpad=25)
    axis0.set_title('Tangential Velocity of Propeller',pad=15)   
    thetaticks = np.arange(0,360,45)
    axis0.set_thetagrids(thetaticks, frac=2.0)    
    axis0.set_rorigin(-rh)
    axis0.set_yticklabels([])
    
    fig0, axis0 = plt.subplots(subplot_kw=dict(projection='polar'))
    CS_0 = axis0.contourf(psi, r, outputs.disc_thrust_distribution[0],100,cmap=plt.cm.jet) #(psi, r, outputs.thrust_distribution_2d[0],100,cmap=plt.cm.jet) ##,cmap=plt.cm.jet)    # -np.pi+psi turns it 
    cbar0 = plt.colorbar(CS_0, ax=axis0, format=matplotlib.ticker.FormatStrFormatter('%.2f'))
    cbar0.ax.set_ylabel('Thrust (N)',labelpad=25)
    axis0.set_title('Thrust Distribution of Propeller',pad=15)  
    axis0.set_rorigin(-rh)
    axis0.set_yticklabels([])
    
    fig0, axis0 = plt.subplots(subplot_kw=dict(projection='polar'))
    CS_0 = axis0.contourf(psi, r, outputs.disc_torque_distribution[0],100,cmap=plt.cm.jet) #(psi, r, outputs.torque_distribution_2d[0],100,cmap=plt.cm.jet) ##,cmap=plt.cm.jet)    # -np.pi+psi turns it 
    cbar0 = plt.colorbar(CS_0, ax=axis0, format=matplotlib.ticker.FormatStrFormatter('%.2f'))
    cbar0.ax.set_ylabel('Torque (Nm)',labelpad=25)
    axis0.set_title('Torque Distribution of Propeller',pad=15) 
    axis0.set_rorigin(-rh)
    axis0.set_yticklabels([])
    
    plt.show()
    return
