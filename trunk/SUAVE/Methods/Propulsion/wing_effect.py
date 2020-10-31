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
    R_tip     = vehicle.propulsors.prop_net.propeller.tip_radius
    r_hub     = vehicle.propulsors.prop_net.propeller.hub_radius    
    prop_loc  = vehicle.propulsors.prop_net.propeller.prop_loc
    case      = vehicle.propulsors.prop_net.propeller.analysis_settings.case
    wake_type = vehicle.propulsors.prop_net.propeller.analysis_settings.wake_type
    
    N         = conditions.N    
    aoa       = conditions.aerodynamics.angle_of_attack
    mach      = conditions.freestream.mach_number
    rho       = conditions.freestream.density
    mu        = conditions.freestream.dynamic_viscosity    
    nu        = mu/rho
    Vv        = conditions.freestream.velocity
    
    span      = vehicle.wings.main_wing.spans.projected
    croot     = vehicle.wings.main_wing.chords.root
    ctip      = vehicle.wings.main_wing.chords.tip
    
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
    z_in_bl = 0.2*np.concatenate([-np.flipud((1-np.cos(np.linspace(1e-6,1,35)*np.pi/2))),(1-np.cos(np.linspace(0,1,35)*np.pi/2))])
    zlocs = np.concatenate([np.linspace(-12*R_tip,-0.2,10), z_in_bl, np.linspace(0.2,12*R_tip,10)])
    #zlocs = np.linspace(vehicle.prop_z_center-12*R_tip,vehicle.prop_z_center+12*R_tip,15*VLM_settings.number_panels_chordwise)
    
    VLM_settings_copy = copy.deepcopy(VLM_settings)
    # Declare new evaluation points:
    # Keep spanwise locations, but add new ones outside of the wingtip to capture the effect over there without interfering with vortices
    half_span = vehicle.wings.main_wing.spans.projected/2
    y_vals_Rwing = np.append(VD.YC[0::VD.n_cw][0:int(len(VD.YC[0::VD.n_cw])/2)],np.linspace(1.05,1.2,15)*half_span)
    y_vals_full_wing = np.concatenate([np.flipud(-y_vals_Rwing),y_vals_Rwing])
    cp_YC  = np.array([ele for idx, ele in enumerate(y_vals_full_wing) for i in range(len(zlocs))])
    
    #y     = np.reshape(VD.YC,(2*vortices**2,vortices))
    #y     = y[y[:,0]<0]
    #extra_span = vehicle.wings.main_wing.spans.projected/5
    #xn    = 2*len(y)
    
    #cp_YC =  np.append(y,np.flipud(y[-1]-(extra_span+y))) #VD.YC #np.linspace(-1.3*vehicle.wings.main_wing.spans.projected,1.3*vehicle.wings.main_wing.spans.projected, len(VD.YC) ) #VD.YC             # Same evaluation point values, repeated to allow for more z-evaluations
    cp_ZC = np.tile(zlocs,len(y_vals_full_wing))
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
    
      
        Va_deficit = np.zeros_like(VD.YC)
        #----------------------------------------------------------------------------------------------
        # Include wake deficit from BL of wing:
        #----------------------------------------------------------------------------------------------
        if case == 'disturbed_freestream' and wake_type == 'viscous':
            wake_method = 'ramparian'
            chord_distribution = np.zeros_like(VD.YC)
            delta_bl           = np.zeros_like(VD.YC)
            
            Rex_prop_plane     = Vv[0][0]*(prop_loc[0])/nu[0][0] # Reynolds number at the propeller plane            
            
            # =====================================================================================================            
            # Using the 1/7th power law (inaccurate because propeller isn't actually located at the trailing edge)
            # =====================================================================================================
            if wake_method =='power_law':
                # if the y-locations are in the wing wake:
                
                chord_distribution[abs(VD.YC)<0.5*span] = croot - (croot-ctip)*(abs(VD.YC[abs(VD.YC)<0.5*span])/(0.5*span))
                Rex_te         = Vv[0][0]*(chord_distribution)/nu[0][0]
                
                #delta_bl[Rex_te<5e5]  = 5.2*(prop_loc[0])/np.sqrt(Rex_te)
                delta_bl[Rex_te>=5e5] = 0.37*(chord_distribution[Rex_te>=5e5])/(Rex_te[Rex_te>=5e5]**(1/5))
                
                # Using the 1/7th power law for turbulent velocity profile:
                Va_deficit[abs(VD.ZC)<=delta_bl] = 1-(abs(VD.ZC[abs(VD.ZC)<=delta_bl])/delta_bl[abs(VD.ZC)<=delta_bl])**(1/7)
            
            # =====================================================================================================            
            # Using the a correlation from Ramaprian et al.
            # =====================================================================================================            
            
            else:
                #Turbulent flow (currently assumes fully turbulent, eventually incorporate transition)
                chord_distribution[abs(VD.YC)<0.5*span] = croot - (croot-ctip)*(abs(VD.YC[abs(VD.YC)<0.5*span])/(0.5*span))
                Rex_te         = Vv[0][0]*(chord_distribution)/nu[0][0]
                
                x_prop = np.zeros_like(chord_distribution)
                ones_vec = np.ones_like(chord_distribution)
                x_prop[chord_distribution>0] = chord_distribution[chord_distribution>0]+(0.2)*ones_vec[chord_distribution>0] # fix the hard coded 2 value later; the 2-proploc should be the difference between wing origin and prop plane
                delta_bl[Rex_te>=5e5] = 0.37*(chord_distribution[Rex_te>=5e5])/(Rex_te[Rex_te>=5e5]**(1/5)) # Approximate the bl as that at the trailing edge #[Rex_prop_plane>=5e5] = 0.37*x_prop/(Rex_prop_plane**(1/5)) 
                theta_turb= 0.036*x_prop[chord_distribution>0]/(Rex_prop_plane**(1/5))
                x_theta = (x_prop[chord_distribution>0]-chord_distribution[chord_distribution>0])/theta_turb

                #Apply BL axial velocity deficit due to turbulent BL at the TE of wing (using 1/7th power law)
                W0 = Vv[0][0]/np.sqrt(4*np.pi*0.032*x_theta)
                b = 2*theta_turb*np.sqrt(16*0.032*np.log(2)*x_theta)
                #Va_deficit[abs(VD.ZC)<=delta_bl] = W0[abs(VD.ZC)<=delta_bl]*np.exp(-4*np.log(2)*(abs(VD.ZC[abs(VD.ZC)<=delta_bl])/b[abs(VD.ZC)<=delta_bl])**2)
                Va_deficit[chord_distribution>0] = W0*np.exp(-4*np.log(2)*(abs(VD.ZC[chord_distribution>0])/b)**2)
                        # Edge: Va_deficit[psi_i][r_i] = va_2d[0][psi_i][r_i]*(1-(y/delta_bl)**(1/7))#Vv[0][0]*(1-(y/delta_bl)**(1/7))
                if len(Va_deficit[chord_distribution>0])>=1:
                    
                    astophere = 1
                            
            #va_2d = va_2d - Va_deficit#np.array(Va_deficit*(1-(prop_loc[0]-c_wing))**3)               
            
            ## Using the method from the bl paper:
            #W0 = Vv[0][0]/np.sqrt(4*np.pi*0.032*x_theta)
            #b = 2*theta_turb*np.sqrt(16*0.032*np.log(2)*x_theta)            
            #Va_deficit[abs(VD.ZC)<=delta_bl] =W0*np.exp(-4*np.log(2)*(abs(VD.ZC)/b)**2)
            
            # If laminar:
            #if Rex_te<5e5:
                ###Flow is fully laminar
                ##delta_bl = 5.2*(prop_loc[0])/np.sqrt(Rex_prop_plane)
                ##for psi_i in range(len(va_2d[0])):
                    ##for r_i in range(len(va_2d[0][0])):
                        ##y = r_dim[r_i]*np.sin(psi[psi_i])
                        ##if y<delta_bl: #within the boundary layer height and wing is in front of this location of propeller
                            ###Apply BL axial velocity deficit due to laminar BL at the TE of wing
                            ##Va_deficit[psi_i][r_i] = Vv[0][0]*(1-y*np.sqrt(Vv[0][0]/(nu*c_wing)))
     
        u[count:count+max_val_per_loop] = (C_mn[:,:,:,0]*MCM[:,:,:,0]@gammaT)[:,:,0] - Va_deficit/Vv[0][0]
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
    
    # Visualization of the flow field behind the wing
    import pylab as plt
    fig  = plt.figure()
    axes = fig.add_subplot(1,1,1)
    a = axes.contourf(y_vals_full_wing/(0.5*vehicle.wings.main_wing.spans.projected),zlocs,np.reshape(w,(len(y_vals_full_wing),len(zlocs))).T)
    axes.set_xlabel('y-location')
    axes.set_ylabel("z-location")
    axes.set_title("Downwash, w")
    cbar = plt.colorbar(a,ax=axes)
    #plt.legend()
    
    
    return vehicle
