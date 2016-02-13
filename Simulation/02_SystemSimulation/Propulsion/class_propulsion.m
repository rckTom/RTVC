classdef class_propulsion
  properties (SetAccess = private)
    x_T;
    y_T;
    y_m;
    x_m;
    y_mdot;
    x_mdot;
  end
  
  properties (SetAccess = public)
    dt;
    J_Pp;
    m_prop;
    m_total;
    r_fpf;
    r_np;
  end
  
  methods
    function obj = class_propulsion(J_Pp,m_prop,m_total,r_fpf,r_np);
      obj.J_Pp = J_Pp;
      obj.m_prop = m_prop;
      obj.m_total = m_total;
      obj.r_fpf = r_fpf;
      obj.r_np = r_np;
    end
    
    function BusProp = update(obj,t);
      BusProp.R_Pp = [interp1(obj.x_T,obj.y_T,t,'linear',0);0;0];
      BusProp.m = interp1(obj.x_m,obj.y_m,t,'linear',obj.y_m(end));
      BusProp.mdot = interp1(obj.x_mdot,obj.y_mdot,t,'linear',0);
      BusProp.I_Pp = zeros(3);
      BusProp.r_fpf = obj.r_fpf;
      BusProp.r_np = obj.r_np;
    end
    
    function obj = setThrustCurve(obj,time,thrust)
      obj.x_T = time;
      obj.y_T = thrust;
      obj.x_m = time;
      obj.x_mdot = time;
      Isp = trapz(time,thrust)/obj.m_prop;
      obj.y_mdot = 1/Isp * thrust;
      obj.y_m = obj.m_total - cumtrapz(time,obj.y_mdot);
    end
    
    
  end

end