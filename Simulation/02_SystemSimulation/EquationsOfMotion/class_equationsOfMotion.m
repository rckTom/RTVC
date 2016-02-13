classdef class_equationsOfMotion
  properties
    m_struct;
    I_struct;
    dt;
    solver;
  end
  methods
    function obj = class_equationsOfMotion(dt,solver)
      obj.dt = dt;
      if(strcmp(solver,'rk4'))
        disp('Equations of motion solver type: Runge Kutta 4')
        obj.solver = class_rk4(dt);
      elseif(strcmp(solver,'euler'))
        disp('Equations of motion solver type: Newton Euler')
        obj.solver = class_euler(dt);
      else
        error('Equations of motion: no valid solver specified')
      end
    end
    
    function BusEoM = update(obj,BusEoM,BusAero,BusProp,BusAct)
        %Transformations
        T_fa = func_T_fa(BusEoM.a,BusEoM.b); %Transformation Aerodynamic system to body system
        T_fp = func_T_fp(BusAct.kappa,BusAct.iota); %Transformation propulsion to body system
        T_fg = q2rotm(BusEoM.q)';
        
        %Aeroforces and Aerotorque
        R_Af = T_fa*BusAero.R_Aa;
        Q_Af = T_fa*BusAero.Q_Aa;
        
        %Thrust and controltorque
        R_Pf = T_fp * BusProp.R_Pp;
        Q_Pf = tilde(BusProp.r_fpf+T_fp*BusProp.r_np)*R_Pf;
        
        %rocket mass
        m = BusProp.m + obj.m_struct;
        
        %inertia tensor
        I_Sf = obj.I_struct + T_fp'*BusProp.I_Pp*T_fp + BusProp.m*tilde(BusProp.r_fpf)'*tilde(BusProp.r_fpf);
        
        y0 = [BusEoM.r_e;BusEoM.v_f;BusEoM.q;BusEoM.omega_f];
        %[BusEoM.r_e;BusEoM.v_f;qvec;BusEoM.omega_f] =obj.solver.solve(@(x)(dgls(x,R_Af+R_Pf,Q_Af+Q_Pf,I_Sf,m,9.81,T_fg,T_fg')),y0);
        [y,ydot] =obj.solver.solve(@(x)(obj.dgls(x,R_Af+R_Pf,Q_Af+Q_Pf,I_Sf,m,9.81,T_fg,T_fg')),y0);
        
        BusEoM.r_e = y(1:3);
        BusEoM.v_f = y(4:6);
        BusEoM.q = y(7:10);
        BusEoM.omega_f = y(11:13);
        BusEoM.v_e = ydot(1:3);
        BusEoM.vdot_f = ydot(4:6);
        BusEoM.qdot = ydot(7:10);
        BusEoM.omegadot_f = ydot(11:13);

        
        if(norm(BusEoM.v_f) <= 0.05)
          BusEoM.a = 0;
          BusEoM.b = 0;
        else
          BusEoM.a = asin(BusEoM.v_f(3)/norm(BusEoM.v_f));
          BusEoM.b = asin(BusEoM.v_f(2)/norm(BusEoM.v_f));
         end
    end
  
  end
  
  methods(SetAccess=private)
    function xdot = dgls(obj,x,R_f,Q_f,I_Sf,m,g,T_fg,T_ef)
      xdot = [T_ef*x(4:6);
             1/m*R_f+T_fg*[0;0;g]-tilde(x(11:13))*x(4:6);
             1/2*[-tilde(x(11:13)),x(11:13);-x(11:13)',0]*x(7:10);
             -inv(I_Sf)*tilde(x(11:13))*I_Sf*x(11:13)+inv(I_Sf)*Q_f];
    end
  end


end
