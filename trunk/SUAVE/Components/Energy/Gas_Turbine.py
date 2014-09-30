# Gas_Turbine.py
# 
# Created:  Anil, July 2014
      
#--put in a folder

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

# SUAVE imports

import SUAVE

from SUAVE.Structure import Data
from SUAVE.Attributes import Units

# python imports
import os, sys, shutil
from copy import deepcopy
from warnings import warn

# package imports
import numpy as np
import scipy as sp

from SUAVE.Structure import (
Data, Container, Data_Exception, Data_Warning,
)

from SUAVE.Structure import Data, Data_Exception, Data_Warning
from SUAVE.Components import Component, Physical_Component, Lofted_Body
from SUAVE.Components import Component_Exception
#from SUAVE.Components.Energy.Gas_Turbine import Network


# ----------------------------------------------------------------------
#  Class
# ----------------------------------------------------------------------
"""
Build a network with manual linking of components
Gas Turbine, with Nozzle Compressor, Fan, Combustor, Turbine, Nozzle, (Afterburner)
    in a test script, with manually set conditions
Use conditions data structure, as would be expected by mission segments
Sketch out the array of different propulsors (ie turboprop, hyrbric electric, 
    ducted fan, turbofan, turbojet, scramjet)
Maybe start thinking about general network
"""


# ----------------------------------------------------------------------
#  Energy Component Class
# ----------------------------------------------------------------------
from SUAVE.Components import Physical_Component

def fm_id(M):

    R=287.87
    g=1.4
    m0=(g+1)/(2*(g-1))
    m1=((g+1)/2)**m0
    m2=(1+(g-1)/2*M**2)**m0
    fm=m1*M/m2
    return fm

#-----------Energy component----------------------------------------------------------

class Energy_Component(Physical_Component):
    def __defaults__(self):
        
        # function handles for input
        self.inputs  = Data()
        
        # function handles for output
        self.outputs = Data()
        
        return

#--------------------------------------------------------------------------------------

class Ram(Energy_Component):
    """ SUAVE.Components.Energy.Gas_Turbine.Ram
        a Ram class that is used to convert static properties into
        stagnation properties
        
        this class is callable, see self.__call__
        
    """    
    
    def __defaults__(self):
        

        self.tag = 'Ram'

        self.etapold = 1.0
        self.pid = 1.0
        
        
        self.outputs.Tt =1.0
        self.outputs.Pt =1.0

        

        
    def compute(self,conditions):
        
        #unpack the variables

        Po = conditions.freestream.pressure
        To = conditions.freestream.temperature
    

    
        
        #method
        conditions.freestream.gamma =1.4
        conditions.freestream.Cp =1.4*287.87/(1.4-1)
        
        
        #Compute the stagnation quantities from the input static quantities
        conditions.freestream.stagnation_temperature = conditions.freestream.temperature*(1+((conditions.freestream.gamma-1)/2 *conditions.freestream.mach_number**2))
        
        conditions.freestream.stagnation_pressure = conditions.freestream.pressure* ((1+(conditions.freestream.gamma-1)/2 *conditions.freestream.mach_number**2 )**3.5 )  
        conditions.freestream.Cp                 = 1.4*287.87/(1.4-1)
        conditions.freestream.R                  = 287.87             
         
        
        #pack outputs
        self.outputs.Tt =conditions.freestream.stagnation_temperature
        self.outputs.Pt =conditions.freestream.stagnation_pressure
        
        

        

        
    __call__ = compute


#--------------------------------------------------------------------------------------

class Compression_Nozzle(Energy_Component):
    """ SUAVE.Components.Energy.Gas_Turbine.Nozzle
        a nozzle component 
        
        this class is callable, see self.__call__
        
    """    
    
    def __defaults__(self):
        

        self.tag = 'Nozzle'

        self.etapold = 1.0
        self.pid = 1.0
        self.inputs
        self.outputs
        self.inputs.Tt=0.
        self.inputs.Pt=0.
        
        self.outputs.Tt=0.
        self.outputs.Pt=0.
        self.outputs.ht=0.
        

        
    def compute(self,conditions):
        
        #unpack the variables
        gamma=conditions.freestream.gamma
        Cp = conditions.freestream.Cp
        Po = conditions.freestream.pressure
        R = conditions.freestream.R
        Tt_in = self.inputs.Tt
        Pt_in = self.inputs.Pt

    
        
        #Computing the output modules
        
        #--Getting the outptu stagnation quantities
        Pt_out=Pt_in*self.pid
        Tt_out=Tt_in*self.pid**((gamma-1)/(gamma*self.etapold))
        ht_out=Cp*Tt_out 
        
        
        #compute the output Mach number, static quantities and the output velocity
        Mach=np.sqrt((((Pt_out/Po)**((gamma-1)/gamma))-1)*2/(gamma-1))
        T_out=Tt_out/(1+(gamma-1)/2*Mach**2)
        h_out=Cp*T_out
        u_out=np.sqrt(2*(ht_out-h_out))  
        
        
        
        #if np.linalg.norm(Mach) < 1.0:
        ## nozzle unchoked
        
            #P_out=Po
            
            #Mach=np.sqrt((((Pt_out/Po)**((gamma-1)/gamma))-1)*2/(gamma-1))
            #Tt_out=Tt_out/(1+(gamma-1)/2*Mach**2)
            #h_out=Cp*T_out
        
        #else:
            #Mach=1
            #T_out=Tt_out/(1+(gamma-1)/2*Mach**2)
            #P_out=Pt_out/(1+(gamma-1)/2*Mach**2)**(gamma/(gamma-1))
            #h_out=Cp*T_out
          
        ## 
        #u_out=np.sqrt(2*(ht_out-h_out))
        #rho_out=P_out/(R*T_out)
        
   
         
        
        #pack outputs
        self.outputs.Tt=Tt_out
        self.outputs.Pt=Pt_out
        self.outputs.ht=ht_out
        self.outputs.M = Mach
        self.outputs.T = T_out
        self.outputs.h = h_out
        self.outputs.u = u_out
        
        

        

        
    __call__ = compute
        
        
class Expansion_Nozzle(Energy_Component):
    """ SUAVE.Components.Energy.Gas_Turbine.Nozzle
        a nozzle component 
        
        this class is callable, see self.__call__
        
    """    
    
    def __defaults__(self):
        

        self.tag = 'Nozzle'

        self.etapold = 1.0
        self.pid = 1.0
        self.inputs
        self.outputs
        self.inputs.Tt=0.
        self.inputs.Pt=0.
        
        self.outputs.Tt=0.
        self.outputs.Pt=0.
        self.outputs.ht=0.
        

        
    def compute(self,conditions):
        
        #unpack the variables
        gamma=conditions.freestream.gamma
        Cp = conditions.freestream.Cp
        Po = conditions.freestream.pressure
        Pto = conditions.freestream.stagnation_pressure
        Tto = conditions.freestream.stagnation_temperature
        R = conditions.freestream.R
        Mo =  conditions.freestream.mach_number
        Tt_in = self.inputs.Tt
        Pt_in = self.inputs.Pt

    
        
        #Computing the output modules
        
        #--Getting the outptu stagnation quantities
        Pt_out=Pt_in*self.pid
        Tt_out=Tt_in*self.pid**((gamma-1)/(gamma*self.etapold))
        ht_out=Cp*Tt_out 
        
        
        #compute the output Mach number, static quantities and the output velocity
        Mach=np.sqrt((((Pt_out/Po)**((gamma-1)/gamma))-1)*2/(gamma-1))
        T_out=Tt_out/(1+(gamma-1)/2*Mach**2)
        h_out=Cp*T_out
        u_out=np.sqrt(2*(ht_out-h_out))  
        
        
        
        if np.linalg.norm(Mach) < 1.0:
        # nozzle unchoked
        
            P_out=Po
            
            Mach=np.sqrt((((Pt_out/Po)**((gamma-1)/gamma))-1)*2/(gamma-1))
            Tt_out=Tt_out/(1+(gamma-1)/2*Mach**2)
            h_out=Cp*T_out
        
        else:
            Mach=1
            T_out=Tt_out/(1+(gamma-1)/2*Mach**2)
            P_out=Pt_out/(1+(gamma-1)/2*Mach**2)**(gamma/(gamma-1))
            h_out=Cp*T_out
          
        # 
        u_out=np.sqrt(2*(ht_out-h_out))
        rho_out=P_out/(R*T_out)
        
        
        area_ratio=(fm_id(Mo)/fm_id(Mach)*(1/(Pt_out/Pto))*(np.sqrt(Tt_out/Tto)))
        

         
        
        #pack outputs
        self.outputs.Tt=Tt_out
        self.outputs.Pt=Pt_out
        self.outputs.ht=ht_out
        self.outputs.M = Mach
        self.outputs.T = T_out
        self.outputs.h = h_out
        self.outputs.u = u_out
        self.outputs.P = P_out
        self.outputs.area_ratio = area_ratio
        
        

        

        
    __call__ = compute
        
#--------------------------------------------------------------------------------------
    
class Compressor(Energy_Component):
    """ SUAVE.Components.Energy.Gas_Turbine.Compressor
        a compressor component
        
        this class is callable, see self.__call__
        
    """    
    
    def __defaults__(self):
        

        self.tag = 'Compressor'

        self.etapold = 1.0
        self.pid = 1.0
        
        self.inputs.Tt=0.
        self.inputs.Pt=0.
        
        self.outputs.Tt=0.
        self.outputs.Pt=0.
        self.outputs.ht=0.        
 
        
        

    def compute(self,conditions):
        
        #unpack the variables
        gamma=conditions.freestream.gamma
        Cp = conditions.freestream.Cp  
        Tt_in = self.inputs.Tt
        Pt_in = self.inputs.Pt        
    
        #Compute the output stagnation quantities based on the pressure ratio of the component
        Pt_out=Pt_in*self.pid
        Tt_out=Tt_in*self.pid**((gamma-1)/(gamma*self.etapold))
        ht_out=Cp*Tt_out 
        
        #pack outputs
        self.outputs.Tt=Tt_out
        self.outputs.Pt=Pt_out
        self.outputs.ht=ht_out        
        

    __call__ = compute
    
    
#--------------------------------------------------------------------------------------
    
class Fan(Energy_Component):
    """ SUAVE.Components.Energy.Gas_Turbine.Fan
        a Fan component
        
        this class is callable, see self.__call__
        
    """    
    
    def __defaults__(self):
        

        self.tag ='Fan'

        self.etapold = 1.0
        self.pid = 1.0
        
        self.inputs.Tt=0.
        self.inputs.Pt=0.
        
        self.outputs.Tt=0.
        self.outputs.Pt=0.
        self.outputs.ht=0.        
      
        
        
    def compute(self,conditions):
        
        gamma=conditions.freestream.gamma
        Cp = conditions.freestream.Cp  
        Tt_in = self.inputs.Tt
        Pt_in = self.inputs.Pt        
    
    
        #Compute the output stagnation quantities based on the pressure ratio of the component
        Pt_out=Pt_in*self.pid
        Tt_out=Tt_in*self.pid**((gamma-1)/(gamma*self.etapold))
        ht_out=Cp*Tt_out    #h(Tt1_8)
        
        
        #pack outputs
        self.outputs.Tt=Tt_out
        self.outputs.Pt=Pt_out
        self.outputs.ht=ht_out   
        

        
        
    __call__ = compute  
        

#--------------------------------------------------------------------------------------


    
class Combustor(Energy_Component):
    """ SUAVE.Components.Energy.Gas_Turbine.Combustor
        a combustor component
        
        this class is callable, see self.__call__
        
    """    
    
    def __defaults__(self):
        

        self.tag = 'Combustor'
        self.eta = 1.0
        self.tau = 1.0
        self.To = 273.0
        self.alphac = 0.0 
        self.Tt4 = 1.0
        self.fuel = SUAVE.Attributes.Propellants.Jet_A()
        
        self.inputs.Tt = 1.0
        self.inputs.Pt = 1.0  
        self.inputs.nozzle_temp = 1.0     
        
        self.outputs.Tt=1.0
        self.outputs.Pt=1.0
        self.outputs.ht=1.0
        self.outputs.f = 1.0        
        
        

        
        
    def compute(self,conditions):
        
        #unpack the variables
        gamma=conditions.freestream.gamma
        Cp = conditions.freestream.Cp
        To = conditions.freestream.temperature
        
        Tt_in = self.inputs.Tt 
        Pt_in = self.inputs.Pt   
        Tt_n = self.inputs.nozzle_temp 
        Tto = self.inputs.freestream_stag_temp
        
        htf=self.fuel.specific_energy
        ht4 = Cp*self.Tt4
        ho = Cp*To
        
        
        
        
        
        #Using the Turbine exit temperature, the fuel properties and freestream temperature to compute the fuel to air ratio f        
        tau = htf/(Cp*To)
        tau_freestream=Tto/To
        
    
        #f=(((self.Tt4/To)-tau_freestream*(Tt_in/Tt_n))/(self.eta*tau-(self.Tt4/To)))
        
        f = (ht4 - ho)/(htf-ht4)
        #print ((self.Tt4/To)-tau_freestream*(Tt_in/Tt_n))
        
        ht_out=Cp*self.Tt4
        Pt_out=Pt_in*self.pib  
        
        #pack outputs
        self.outputs.Tt=self.Tt4
        self.outputs.Pt=Pt_out
        self.outputs.ht=ht_out 
        self.outputs.f = 0.0215328 #f
        

        
    __call__ = compute
   
        
#--------------------------------------------------------------------------------------


class Turbine(Energy_Component):
    """ SUAVE.Components.Energy.Gas_Turbine.Turbine
        a Turbine component
        
        this class is callable, see self.__call__
        
    """    
    
    def __defaults__(self):
        

        self.tag ='Turbine'
        self.eta_mech =1.0
        self.Cp =1004
        self.gamma =1.4
        self.etapolt = 1.0
        
        self.inputs.Tt = 1.0
        self.inputs.Pt = 1.0  
        self.inputs.h_compressor_out = 1.0
        self.inputs.h_compressor_in = 1.0
        self.inputs.f = 1.0
        

        self.outputs.Tt=1.0
        self.outputs.Pt=1.0
        self.outputs.ht=1.0    
        
        
     
        
    def compute(self,conditions):
        
        #unpack inputs
        gamma=conditions.freestream.gamma
        Cp = conditions.freestream.Cp 
        
        Tt_in =self.inputs.Tt 
        Pt_in =self.inputs.Pt   
        h_compressor_out =self.inputs.h_compressor_out 
        h_compressor_in=self.inputs.h_compressor_in
        h_fan_out =  self.inputs.h_fan_out
        h_fan_in = self.inputs.h_fan_in
        alpha =  self.inputs.alpha
        f =self.inputs.f        
        
        #Using the stagnation enthalpy drop across the corresponding turbine and the fuel to air ratio to compute the energy drop across the turbine
        deltah_ht=-1/(1+f)*1/self.eta_mech*((h_compressor_out-h_compressor_in)+ alpha*(h_fan_out-h_fan_in))
        
        #Compute the output stagnation quantities from the inputs and the energy drop computed above 
        Tt_out=Tt_in+deltah_ht/Cp
        Pt_out=Pt_in*(Tt_out/Tt_in)**(gamma/((gamma-1)*self.etapolt))
        ht_out=Cp*Tt_out   #h(Tt4_5)
        
        
        print 'turbofan hpt out deltah_lt', deltah_ht
        print 'turbofan hpt out h_compressor_out', h_compressor_out
        print 'turbofan hpt out h_compressor_in', h_compressor_in
        
        #pack outputs
        self.outputs.Tt=Tt_out
        self.outputs.Pt=Pt_out
        self.outputs.ht=ht_out        

        
    __call__ = compute      
    

#--------------------------------------------------------------------------------------


   


    
class Thrust(Energy_Component):
    """ SUAVE.Components.Energy.Gas_Turbine.Thrust
        a component that computes the thrust and other output properties
        
        this class is callable, see self.__call__
        
    """    
    
    def __defaults__(self):
        

        self.tag ='Thrust'
        self.alpha=1.0
        self.mdhc =1.0
        self.Tref =1.0
        self.Pref=1.0
        self.no_eng =1.0

        
        self.inputs.fan_exit_velocity =1.0
        self.inputs.core_exit_velocity =1004.
        self.inputs.f = 1.0
        
  

        self.outputs.Thrust=1.0
        self.outputs.sfc=1.0
        self.outputs.Isp=1.0    
        self.outputs.non_dim_thrust=1.0
        self.outputs.mdot_core = 1.0
        self.outputs.fuel_rate=1.0
        self.outputs.mfuel = 1.0
        self.outputs.power = 1.0
        
        
        
     
        
    def compute(self,conditions):
        
        #unpack inputs
        gamma=conditions.freestream.gamma
        Cp = conditions.freestream.Cp 
        
        fan_exit_velocity= self.inputs.fan_exit_velocity 
        core_exit_velocity=self.inputs.core_exit_velocity 
        f= self.inputs.f  
        stag_temp_lpt_exit=self.inputs.stag_temp_lpt_exit
        stag_press_lpt_exit=self.inputs.stag_press_lpt_exit
        fan_area_ratio = self.inputs.fan_area_ratio
        core_area_ratio = self.inputs.core_area_ratio        
        core_nozzle = self.inputs.core_nozzle
        fan_nozzle = self.inputs.fan_nozzle

        u0 = conditions.freestream.velocity
        a0=conditions.freestream.speed_of_sound
        M0 = conditions.freestream.mach_number
        p0 = conditions.freestream.pressure
        
        g = conditions.freestream.gravity
        throttle = conditions.propulsion.throttle
        


        #Computing the engine output properties, the thrust, SFC, fuel flow rate--------------

        ##----drela method--------------
        


        ##--specific thrust
        #specific_thrust=((1+f)*core_exit_velocity-u0+self.alpha*(fan_exit_velocity-u0))/((1+self.alpha)*a0)

        ##Specific impulse
        #Isp=specific_thrust*a0*(1+self.alpha)/(f*g)
    
        ##thrust specific fuel consumption
        #TSFC=3600/Isp  
        
        ##mass flow sizing
        #mdot_core=self.mdhc*np.sqrt(self.Tref/stag_temp_lpt_exit)*(stag_press_lpt_exit/self.Pref)

        ##fuel flow rate computation
        #fuel_rate=mdot_core*f*self.no_eng
        
        ##dimensional thrust
        #thrust=specific_thrust*a0*(1+self.alpha)*mdot_core*self.no_eng*throttle
        
        ##--fuel mass flow rate
        #mfuel=0.1019715*thrust*TSFC/3600            
        
        ##--Output power based on freestream velocity
        #power = thrust*u0
        
        

        ##--------Cantwell method---------------------------------

        Ae_b_Ao=1/(1+self.alpha)*core_area_ratio
        
        print 'Ae_b_Ao',Ae_b_Ao        
        
        A1e_b_A1o=self.alpha/(1+self.alpha)*fan_area_ratio
         
         
        print 'A1e_b_A1o',A1e_b_A1o          
         
         
        Thrust_nd=gamma*M0**2*(1/(1+self.alpha)*(core_nozzle.u/u0-1)+(self.alpha/(1+self.alpha))*(fan_nozzle.u/u0-1))+Ae_b_Ao*(core_nozzle.P/p0-1)+A1e_b_A1o*(fan_nozzle.P/p0-1)
        
        
        print 'self.alpha',self.alpha
        print 'core_nozzle.u/u0-1',core_nozzle.u/u0-1
        print 'fan_nozzle.u/u0-1',fan_nozzle.u/u0-1
        print 'core_nozzle.P/p0-1',core_nozzle.P/p0-1
        print 'fan_nozzle.P/p0-1 ',fan_nozzle.P/p0-1
        
        
        ##calculate actual value of thrust 
        
        Fsp=1/(gamma*M0)*Thrust_nd
        
        print 'Fsp ',Fsp

      ##overall engine quantities
        
        Isp=Fsp*a0*(1+self.alpha)/(f*g)
        TSFC=3600/Isp  # for the test case 


        print 'TSFC ',TSFC
        
        #mass flow sizing
        mdot_core=self.mdhc*np.sqrt(self.Tref/stag_temp_lpt_exit)*(stag_press_lpt_exit/self.Pref)
        
        ##mdot_core=FD/(Fsp*ao*(1+aalpha))
        ##print mdot_core
        print 'mdot_core ',stag_temp_lpt_exit
      
        ##-------if areas specified-----------------------------
        fuel_rate=mdot_core*f*self.no_eng
        
        FD2=Fsp*a0*(1+self.alpha)*mdot_core*self.no_eng*throttle
        mfuel=0.1019715*FD2*TSFC/3600
        ###State.config.A_engine=A22
        
        print 'Thrust' , FD2        
        
        power = FD2*u0
        
        #pack outputs

        self.outputs.Thrust= FD2 #thrust
        self.outputs.sfc=TSFC
        self.outputs.Isp=Isp   
        self.outputs.non_dim_thrust=Fsp #specific_thrust
        self.outputs.mdot_core = mdot_core
        self.outputs.fuel_rate= fuel_rate
        self.outputs.mfuel = mfuel     
        self.outputs.power = power  

        
    __call__ = compute         
        
      
        
#--------------------------------------------------------------------------------------


        
# the network
class Network(Data):
    def __defaults__(self):
        
        self.tag = 'Network'
        #self.Nozzle       = SUAVE.Components.Energy.Gas_Turbine.Nozzle()
        #self.Compressor   = SUAVE.Components.Energy.Gas_Turbine.Compressor()
        #self.Combustor    = SUAVE.Components.Energy.Gas_Turbine.Combustor()
        #self.Turbine      = SUAVE.Components.Energy.Gas_Turbine.Turbine()
      

        self.nacelle_dia = 0.0
        self.tag         = 'Network'
        
    _component_root_map = None
        
        
        

    
    # manage process with a driver function
    def evaluate(self,conditions,numerics):
    
        # unpack to shorter component names
        # table the equal signs
  

        self.ram(conditions)
        

        
        
        self.inlet_nozzle.inputs.Tt = self.ram.outputs.Tt #conditions.freestream.stagnation_temperature
        self.inlet_nozzle.inputs.Pt = self.ram.outputs.Pt #conditions.freestream.stagnation_pressure
        
    
        print 'ram out temp ', self.ram.outputs.Tt
        print 'ram out press', self.ram.outputs.Pt    
        
        
        self.inlet_nozzle(conditions)   
        
        
        print 'inlet nozzle out temp ', self.inlet_nozzle.outputs.Tt
        print 'inlet nozzle out press', self.inlet_nozzle.outputs.Pt         
        print 'inlet nozzle out h', self.inlet_nozzle.outputs.ht         

        #---Flow through core------------------------------------------------------
        
        #--low pressure compressor
        self.low_pressure_compressor.inputs.Tt = self.inlet_nozzle.outputs.Tt
        self.low_pressure_compressor.inputs.Pt = self.inlet_nozzle.outputs.Pt
        
        self.low_pressure_compressor(conditions) 
        
        print 'low_pressure_compressor out temp ', self.low_pressure_compressor.outputs.Tt
        print 'low_pressure_compressor out press', self.low_pressure_compressor.outputs.Pt 
        print 'low_pressure_compressor out h', self.low_pressure_compressor.outputs.ht
        #--high pressure compressor
        
        self.high_pressure_compressor.inputs.Tt = self.low_pressure_compressor.outputs.Tt
        self.high_pressure_compressor.inputs.Pt = self.low_pressure_compressor.outputs.Pt        
        
        self.high_pressure_compressor(conditions) 
        
        print 'high_pressure_compressor out temp ', self.high_pressure_compressor.outputs.Tt
        print 'high_pressure_compressor out press', self.high_pressure_compressor.outputs.Pt            
        print 'high_pressure_compressor out h', self.high_pressure_compressor.outputs.ht
        
        
        #Fan
        
        
        self.fan.inputs.Tt = self.inlet_nozzle.outputs.Tt
        self.fan.inputs.Pt = self.inlet_nozzle.outputs.Pt
        
        self.fan(conditions) 
        
        print 'fan out temp ', self.fan.outputs.Tt
        print 'fan out press', self.fan.outputs.Pt     
        print 'fan out h', self.fan.outputs.ht
        

        
        #--Combustor
        self.combustor.inputs.Tt = self.high_pressure_compressor.outputs.Tt
        self.combustor.inputs.Pt = self.high_pressure_compressor.outputs.Pt   
        self.combustor.inputs.nozzle_temp = self.inlet_nozzle.outputs.Tt
        self.combustor.inputs.freestream_stag_temp = self.ram.outputs.Tt
        
        self.combustor(conditions)
        
        print 'combustor out temp ', self.combustor.outputs.Tt
        print 'combustor out press', self.combustor.outputs.Pt          
        print 'combustor out f', self.combustor.outputs.f
        print 'combustor out h', self.combustor.outputs.ht
        #high pressure turbine
        
        self.high_pressure_turbine.inputs.Tt = self.combustor.outputs.Tt
        self.high_pressure_turbine.inputs.Pt = self.combustor.outputs.Pt    
        self.high_pressure_turbine.inputs.h_compressor_out = self.high_pressure_compressor.outputs.ht
        self.high_pressure_turbine.inputs.h_compressor_in = self.low_pressure_compressor.outputs.ht
        self.high_pressure_turbine.inputs.f = self.combustor.outputs.f
        self.high_pressure_turbine.inputs.h_fan_out =  0.0
        self.high_pressure_turbine.inputs.h_fan_in = 0.0
        self.high_pressure_turbine.inputs.alpha =0.0    
        
        self.high_pressure_turbine(conditions)
        
        print 'high_pressure_turbine out temp ', self.high_pressure_turbine.outputs.Tt
        print 'high_pressure_turbine out press', self.high_pressure_turbine.outputs.Pt        
        print 'high_pressure_turbine out h', self.high_pressure_turbine.outputs.ht
        #high pressure turbine        
        
        self.low_pressure_turbine.inputs.Tt = self.high_pressure_turbine.outputs.Tt
        self.low_pressure_turbine.inputs.Pt = self.high_pressure_turbine.outputs.Pt    
        self.low_pressure_turbine.inputs.h_compressor_out = self.low_pressure_compressor.outputs.ht
        self.low_pressure_turbine.inputs.h_compressor_in = self.inlet_nozzle.outputs.ht
        self.low_pressure_turbine.inputs.f = self.combustor.outputs.f   
        self.low_pressure_turbine.inputs.h_fan_out =  self.fan.outputs.ht
        self.low_pressure_turbine.inputs.h_fan_in =  self.inlet_nozzle.outputs.ht
        self.low_pressure_turbine.inputs.alpha =  self.thrust.alpha   
        
        self.low_pressure_turbine(conditions)
        
        print 'low_pressure_turbine out temp ', self.low_pressure_turbine.outputs.Tt
        print 'low_pressure_turbine out press', self.low_pressure_turbine.outputs.Pt        
        print 'low_pressure_turbine out h', self.low_pressure_turbine.outputs.ht        
        
        #core nozzle  
        
        self.core_nozzle.inputs.Tt = self.low_pressure_turbine.outputs.Tt
        self.core_nozzle.inputs.Pt = self.low_pressure_turbine.outputs.Pt     
        
        self.core_nozzle(conditions)   
        
        print 'core_nozzle out temp ', self.core_nozzle.outputs.Tt
        print 'core_nozzle out press', self.core_nozzle.outputs.Pt        
        print 'core_nozzle out h', self.core_nozzle.outputs.ht        

        

        
        
        #fan nozzle
        
        self.fan_nozzle.inputs.Tt = self.fan.outputs.Tt
        self.fan_nozzle.inputs.Pt = self.fan.outputs.Pt        
        
        self.fan_nozzle(conditions)   
         
        print 'fan_nozzle out temp ', self.fan_nozzle.outputs.Tt
        print 'fan_nozzle out press', self.fan_nozzle.outputs.Pt        
        print 'fan_nozzle out h', self.fan_nozzle.outputs.ht         
        
        #compute thrust
        
        self.thrust.inputs.fan_exit_velocity = self.fan_nozzle.outputs.u
        self.thrust.inputs.core_exit_velocity = self.core_nozzle.outputs.u 
        self.thrust.inputs.f  = self.combustor.outputs.f
        self.thrust.inputs.stag_temp_lpt_exit  = self.low_pressure_compressor.outputs.Tt
        self.thrust.inputs.stag_press_lpt_exit = self.low_pressure_compressor.outputs.Pt
        self.thrust.inputs.fan_area_ratio = self.fan_nozzle.outputs.area_ratio
        self.thrust.inputs.core_area_ratio = self.core_nozzle.outputs.area_ratio
        self.thrust.inputs.fan_nozzle = self.fan_nozzle.outputs
        self.thrust.inputs.core_nozzle = self.core_nozzle.outputs
        self.thrust(conditions)
        
 
        #getting the output data from the thrust outputs
        
        F = self.thrust.outputs.Thrust
        mdot = self.thrust.outputs.mfuel
        Isp = self.thrust.outputs.Isp
        P = self.thrust.outputs.power


       # return F,mdot,Isp
        return F[:,0],mdot[:,0],P[:,0]  #return the 2d array instead of the 1D array

            
    __call__ = evaluate
    
    
    
    
# ----------------------------------------------------------------------
#   Module Tests
# ----------------------------------------------------------------------
#make 2 test scripts
#one like below
#one full mission

def test():
    

    #-------Conditions---------------------
    
    # --- Conditions        
    ones_1col = np.ones([1,1])    
    
    
    
    # setup conditions
    conditions = Data()
    conditions.frames       = Data()
    conditions.freestream   = Data()
    conditions.aerodynamics = Data()
    conditions.propulsion   = Data()
    conditions.weights      = Data()
    conditions.energies     = Data()
  #  self.conditions = conditions
    

    # freestream conditions
    conditions.freestream.velocity           = ones_1col*223.
    conditions.freestream.mach_number        = ones_1col*0.8
    conditions.freestream.pressure           = ones_1col*20000.
    conditions.freestream.temperature        = ones_1col*215.
    conditions.freestream.density            = ones_1col* 0.8
    conditions.freestream.speed_of_sound     = ones_1col* 300.
    conditions.freestream.viscosity          = ones_1col* 0.000001
    conditions.freestream.altitude           = ones_1col* 10.
    conditions.freestream.gravity            = ones_1col*9.8

    

    
    # propulsion conditions
    conditions.propulsion.throttle           =  1.0

    

    
    
    
    #----------engine propulsion-----------------
    
    
    


    gt_engine = SUAVE.Components.Energy.Gas_Turbine.Network()

    

    
    #Ram
    ram = SUAVE.Components.Energy.Gas_Turbine.Ram()
    ram.tag = 'ram'
    gt_engine.ram = ram
    
    
    
    
    #inlet nozzle
    inlet_nozzle = SUAVE.Components.Energy.Gas_Turbine.Compression_Nozzle()
    inlet_nozzle.tag = 'inlet nozzle'
    gt_engine.inlet_nozzle = inlet_nozzle
    gt_engine.inlet_nozzle.etapold = 1.0
    gt_engine.inlet_nozzle.pid = 1.0
 
    
    
    #low pressure compressor    
    low_pressure_compressor = SUAVE.Components.Energy.Gas_Turbine.Compressor()
    low_pressure_compressor.tag = 'lpc'
    gt_engine.low_pressure_compressor = low_pressure_compressor
    gt_engine.low_pressure_compressor.etapold = 0.94
    gt_engine.low_pressure_compressor.pid = 1.14
    
    

      
    #high pressure compressor  
    high_pressure_compressor = SUAVE.Components.Energy.Gas_Turbine.Compressor()
    high_pressure_compressor.tag = 'hpc'
    gt_engine.high_pressure_compressor = high_pressure_compressor
    gt_engine.high_pressure_compressor.etapold = 0.91
    gt_engine.high_pressure_compressor.pid = 13.2
    
 

    
    #low pressure turbine  
    low_pressure_turbine = SUAVE.Components.Energy.Gas_Turbine.Turbine()
    low_pressure_turbine.tag='lpt'
    gt_engine.low_pressure_turbine = low_pressure_turbine
    gt_engine.low_pressure_turbine.eta_mech =0.99
    gt_engine.low_pressure_turbine.etapolt = 0.87       
    
    
    #high pressure turbine  
    high_pressure_turbine = SUAVE.Components.Energy.Gas_Turbine.Turbine()
    high_pressure_turbine.tag='hpt'
    gt_engine.high_pressure_turbine = high_pressure_turbine   
    gt_engine.high_pressure_turbine.eta_mech =0.99
    gt_engine.high_pressure_turbine.etapolt = 0.91       
    
    
    #combustor  
    combustor = SUAVE.Components.Energy.Gas_Turbine.Combustor()
    combustor.tag = 'Comb'
    gt_engine.combustor = combustor
    gt_engine.combustor.eta = 0.95
    #gt_engine.combustor.To = To
    gt_engine.combustor.alphac = 1.0     
    gt_engine.combustor.Tt4 =   1400
    gt_engine.combustor.pib =   0.99
    gt_engine.fuel = SUAVE.Attributes.Propellants.Jet_A()
    
    
    #core nozzle
    core_nozzle = SUAVE.Components.Energy.Gas_Turbine.Expansion_Nozzle()
    core_nozzle.tag = 'core nozzle'
    gt_engine.core_nozzle = core_nozzle
    gt_engine.core_nozzle.etapold = 1.0
    gt_engine.core_nozzle.pid = 1.0
     



    #fan nozzle
    fan_nozzle = SUAVE.Components.Energy.Gas_Turbine.Expansion_Nozzle()
    fan_nozzle.tag = 'fan nozzle'
    gt_engine.fan_nozzle = fan_nozzle
    gt_engine.fan_nozzle.etapold = 1.0
    gt_engine.fan_nozzle.pid = 1.0

    
    #power out as an output
    #fan    
    fan = SUAVE.Components.Energy.Gas_Turbine.Fan()
    fan.tag = 'fan'
    gt_engine.fan = fan
    gt_engine.fan.etapold = 0.98
    gt_engine.fan.pid = 1.7
    
    #thrust
    thrust = SUAVE.Components.Energy.Gas_Turbine.Thrust()
    thrust.tag ='compute_thrust'
    
    gt_engine.thrust = thrust
    gt_engine.thrust.alpha=6.2
    gt_engine.thrust.mdhc =1.0
    gt_engine.thrust.Tref =273.0
    gt_engine.thrust.Pref=101325
    gt_engine.thrust.no_eng =1.0    


    #byoass ratio  closer to fan
    
    numerics = Data()
    
    eta=1.0
    [F,mdot,Isp] = gt_engine(conditions,numerics)
    
    print F
    
    



if __name__ == '__main__':   
    test()
    #raise RuntimeError , 'module test failed, not implemented'



# ------------------------------------------------------------
#  Handle Linking
# ------------------------------------------------------------

#Nozzle.Container = Container
#Compressor.Container = Container
#Combustor.Container = Container
#Turbine.Container = Container
