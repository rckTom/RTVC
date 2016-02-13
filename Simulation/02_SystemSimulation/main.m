clear all

% Setup
setup;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% module time stepping control %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tsim = 10;                                                %simulation time [s]
dt_base = 10e-3;                                           %baseclock timestep [s]

%%%%%%%%%%%%%%%%%%%%%
% BusInitialization %
%%%%%%%%%%%%%%%%%%%%%
BusAero  = class_BusAero();
BusAtmos = class_BusAtmos();
BusProp  = class_BusProp();
BusEoM   = class_BusEoM();
BusAct   = class_BusAct();

%%%%%%%%%%%%%%%%%%%%%%
% Initial Conditions %
%%%%%%%%%%%%%%%%%%%%%%
BusEoM.v_f     = [0;0;0];                                 %velocity in body frame
BusEoM.r_e     = [0;0;0];                                 %position vector Earth->Rocket
BusEoM.omega_f = [0;0;0];                                 %angular velocity
q = rotm2q(func_T_fg([0 pi/2 0]));                                 
BusEoM.q       = [q.x;q.y;q.z;q.w];                       %orientation (upright position)

BusEoM.a = 0;
BusEoM.b = 0;
%%%%%%%%%%%%%%%%
% Module setup %
%%%%%%%%%%%%%%%%

engine = func_importEng(fopen('Propulsion/thrustCurves/AeroTech_D2.eng'));
J_Pp = eye(3)*10^-15;
propulsion = class_propulsion(J_Pp,engine.propWeight,engine.totalWeight,[0;0;0],[0;0;0]);
propulsion = propulsion.setThrustCurve(engine.time,engine.thrust);

%aerodynamics = class_aerodynamics();

equationsOfMotion = class_equationsOfMotion(dt_base,'rk4');
equationsOfMotion.m_struct = 0.146;
equationsOfMotion.I_struct = [7.5e-5  ,-4.8e-7  ,-5.0e-7;
                              -4.8e-7 , 8.1e-4  ,-3.1e-7;
                              -5.0e-7 ,-3.1e-7  , 8.1e-4];

%%%%%%%%%%%%%%%%%%%%
% Datalogger setup %
%%%%%%%%%%%%%%%%%%%%

%Datalogging        %Variable          %Name        %Size
loggingVariables = {'t'                   ,'t'         ,[1,1];
                    'BusProp.R_Pp'        ,'R_Pp'      ,[3,1];
                    'BusProp.m'           ,'m_prop'    ,[1,1];
                    'BusProp.mdot'        ,'mdot_prop' ,[1,1];
                    'BusEoM.q'            ,'q'         ,[4,1];
                    'BusEoM.v_f'          ,'v_f'       ,[3,1];
                    'BusEoM.r_e'          ,'r_e'       ,[3,1];
                    'BusEoM.omega_f'      ,'omega_f'   ,[3,1];
                    'BusEoM.a'            ,'a'         ,[1,1];
                    'BusEoM.b'            ,'b'         ,[1,1];
                    'BusEoM.v_e'          ,'v_e'       ,[3,1];
                    'BusEoM.vdot_f'       ,'vdot_f'    ,[3,1];
                    'BusEoM.omegadot_f'   ,'omegadot_f',[3,1];
                    'BusEoM.qdot'         ,'qdot'      ,[4,1]
                    };
                    
logger = dataLogger(loggingVariables,floor(tsim/dt_base));


%% Simulation

tic
for t=0:dt_base:tsim
%Sim
%BusRail = rail(BusKin);
%BusAtmos = atmosphere(BusKin);
%BusAero = aerodynamics(BusKin,BusAtmos);
BusProp = propulsion.update(t);

%BusAct = control();
%BusRecovery = flightController(BusEoM);

BusAero.R_Aa = [0;0;0];
BusAero.Q_Aa = [0;0;0];
BusAero.Aref = 0;

BusAct.iota = 0;
BusAct.kappa = 0;

BusEoM = equationsOfMotion.update(BusEoM,BusAero,BusProp,BusAct);
%Log
logger = logger.log();
end
toc
%% Post Processing

