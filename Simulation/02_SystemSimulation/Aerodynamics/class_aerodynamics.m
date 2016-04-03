classdef class_aerodynamics
  properties (SetAccess = public)
    p_cl;           %Poly Coefficients Cl
    p_cd;           %Poly Coefficients Cd
    p_cm;           %Poly Coefficients Cm
    lref;           %Reference length
    Aref;           %Reference Area
    S_z;            %Referencepoint in Zero System
    absAlphaMax;
    absVMax;
  end

  methods
    function obj = class_aerodynamics(Aref,lref,S_z,maxAlpha,maxV)
      %Aref = Refrence Area
      %Lref = Refrence Length
      %S_z =  Position Refrence Point from Base
      obj.Aref = Aref;
      obj.lref = lref;
      obj.S_z = S_z;
      obj.absAlphaMax = abs(maxAlpha);
      obj.absVMax = abs(maxV);
    end
    function BusAero = update(obj,BusEoM,BusAtmos)
      v = norm(BusEoM.v_f);
      if(v > obj.absVMax)
        warning('velocity out of interpolation range. Simulation invalid');
      end
      if(abs(BusEoM.a) > obj.absAlphaMax || abs(BusEoM.b) > obj.absAlphaMax)
        %error('angle of out of interpolation range. Simulation invalid');
      end
      ca = obj.p_cl(1)*power(v,2)+obj.p_cl(2)*v+obj.p_cl(3)*power(BusEoM.a,2)+obj.p_cl(4)*BusEoM.a+obj.p_cl(5);
      cq = obj.p_cl(1)*power(v,2)+obj.p_cl(2)*v+obj.p_cl(3)*power(BusEoM.b,2)+obj.p_cl(4)*BusEoM.b+obj.p_cl(5);
      cwa = obj.p_cd(1)*power(v,2)+obj.p_cd(2)*v+obj.p_cd(3)*power(BusEoM.a,2)+obj.p_cd(4)*BusEoM.a+obj.p_cd(5);
      cwq = obj.p_cd(1)*power(v,2)+obj.p_cd(2)*v+obj.p_cd(3)*power(BusEoM.b,2)+obj.p_cd(4)*BusEoM.b+obj.p_cd(5);
      cmy = obj.p_cm(1)*power(v,2)+obj.p_cm(2)*v+obj.p_cm(3)*power(BusEoM.a,2)+obj.p_cm(4)*BusEoM.a+obj.p_cm(5);
      cmz = obj.p_cm(1)*power(v,2)+obj.p_cm(2)*v+obj.p_cm(3)*power(BusEoM.b,2)+obj.p_cm(4)*BusEoM.b+obj.p_cm(5);
      
      BusAero.R_Aa = 1/2*BusAtmos.rho*v^2*obj.Aref*[-(cwa+cwq);cq;-ca];
      BusAero.Q_Aa = 1/2*BusAtmos.rho*v^2*obj.lref*obj.Aref*[0;cmy;cmz];
      BusAero.Aref = obj.Aref;
      BusAero.S_z = obj.S_z;
    end
  end

end