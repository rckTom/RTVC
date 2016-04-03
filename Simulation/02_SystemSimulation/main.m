clear all

% Setup
setup;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% time stepping control %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  tsim = 3;                                                  %simulation time [s]
  dt_base = 1e-3;                                            %baseclock timestep [s]

%%%%%%%%%%%%%%%%%%%%%
% BusInitialization %
%%%%%%%%%%%%%%%%%%%%%
  BusAero  = class_BusAero();
  BusAtmos = class_BusAtmos();
  BusProp  = class_BusProp();
  BusEoM   = class_BusEoM();
  BusAct   = class_BusAct();
  BusEvent = class_BusEvent();

%%%%%%%%%%%%%%%%%%%%%%
% Initial Conditions %
%%%%%%%%%%%%%%%%%%%%%%
  BusEoM.v_f     = [0;0;0];                                   %velocity in body frame
  BusEoM.r_e     = [0;0;0];                                   %position vector Earth->Rocket in earth system
  BusEoM.omega_f = [0;0;0];                                   %angular velocity
  q = rotm2q(func_T_fg([0 pi/2 0]));                                 
  BusEoM.q       = [q.w;q.x;q.y;q.z];                         %orientation (upright position)
  q = BusEoM.q;
  
  BusEoM.a = 0;
  BusEoM.b = 0;
%%%%%%%%%%%%%%%%
% Module setup %
%%%%%%%%%%%%%%%%
  %Atmosphere
  atmosphere = class_atmosphere();

  %Aerodynamics
  aerodynamics = class_aerodynamics(3.6317e-3,6.7e-2,[5.79e-2;0;0],15*pi/180,50); %Aref and lref
  aerodynamics.p_cl =  [-1.4252e+000;3.7060e+000;1.2636e-004;-7.5468e-003;8.4986e-002];
  aerodynamics.p_cd =  [4.6140e+000;1.1744e-001;2.5704e-004;-1.4295e-002;6.6742e-001];
  aerodynamics.p_cm =  [6.2454e-001;-9.9629e+000;-2.8123e-004;1.6856e-002;-1.9062e-001];

  %Propulsion
  engine = func_importEng(fopen('Propulsion/thrustCurves/AeroTech_D2.eng'));
  J_Pp = eye(3)*10^-15;
  propulsion = class_propulsion(J_Pp,engine.propWeight,engine.totalWeight,[-0.10;0;0],[0;0;0]);
  propulsion = propulsion.setThrustCurve(engine.time,engine.thrust);

  %Equations of Motion
  equationsOfMotion = class_equationsOfMotion(dt_base,'rk4');
  equationsOfMotion.m_struct = 0.146;
  equationsOfMotion.I_struct = [7.5e-5  ,-4.8e-7  ,-5.0e-7;
                                -4.8e-7 , 8.1e-4  ,-3.1e-7;
                                -5.0e-7 ,-3.1e-7  , 8.1e-4];
                                
  %Rail and Ground
  rail = class_Rail();
  
  %Control
  baseControl = class_attitudeControl();

%%%%%%%%%%%%%%%%%%%%
% Datalogger setup %
%%%%%%%%%%%%%%%%%%%%
  %Datalogging        %Variable             %Name        %Size
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
                      'BusEoM.qdot'         ,'qdot'      ,[4,1];
                      'BusEoM.a'            ,'alpha'     ,[1,1];
                      'BusEoM.b'            ,'beta'      ,[1,1];
                      'BusEoM.R_Pf'         ,'R_Pf'      ,[3,1];
                      'BusEoM.Q_Pf'         ,'Q_Pf'      ,[3,1];
                      'BusEoM.R_Af'         ,'R_Af'      ,[3,1];
                      'BusEoM.Q_Af'         ,'Q_Af'      ,[3,1];
                      'BusAct.iota'         ,'iota'      ,[1,1];
                      'BusAct.kappa'        ,'kappa'     ,[1,1];
                      };
                      
  logger = dataLogger(loggingVariables,floor(tsim/dt_base));


%% Simulation
tic
for t=0:dt_base:tsim
  %try
    %Sim
    BusAtmos = atmosphere.update(BusEoM);
    BusAero = aerodynamics.update(BusEoM,BusAtmos);
    BusProp = propulsion.update(t);
    BusAct = baseControl.update(q,BusEoM);
    %BusRecovery = flightController(BusEoM);
    BusEoM = equationsOfMotion.update(BusEoM,BusAero,BusProp,BusAct);
    BusEoM = rail.update(BusEoM);
    %Log
    logger = logger.log();
  %catch err
   % disp(['Critical Error: ' err.message]);
   % break;
  %end_try_catch
end
logger = logger.endLogging();
toc

%% Post Processing
plot3(logger.data.r_e(1,:),logger.data.r_e(2,:),logger.data.r_e(3,:))
axis equal
title('Trajectory')
xlabel('x [m]')
