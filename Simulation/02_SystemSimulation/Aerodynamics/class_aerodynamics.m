classdef class_aerodynamics
  properties (SetAccess = public)
    cl;
    cd;
    cm;
    lref;
    Aref;
    S0;
    alphaRange;
    vRange;
  end

  methods
    function obj = class_aerodynamics(Aref,lref)
    
    end
    function BusAero = update(obj,BusAtmos,busEoM)
      Va = norm(BusEoM.v_aA);
      if(Va > vRange(end))
        warning('velocity out of interpolation range. Simulation invalid');
      end
      if(abs(BusEoM.alpha) > alphaRange(end) || abs(BusEoM.beta) > alphaRange(end))
        warnung('angle of out of interpolation range. Simulation invalid');
      end
      BusAero.ca = interp2(alphaRange,vRange,cl,BusEoM.alpha,Va);
      BusAero.cq = interp2(alphaRange,vRange,cl,BusEoM.beta,Va);
      BusAero.cwa = interp2(alphaRange,vRange,cd,BusEoM.alpha,Va);
      BusAero.cwq = interp2(alphaRange,vRange,cd,BusEoM.beta,Va);
      BusAero.cmy = interp2(alphaRange,vRange,cm,BusEoM.alpha,Va);
      BusAero.cmz = interp2(alphaRange,vRange,cm,BusEoM.beta,Va);
      
      BusAero.R_Aa = 1/2*BusAtmos.rho*Va^2*obj.Aref*[-(cwa+cwq);cq;-ca];
      BusAero.Q_Aa = 1/2*BusAtmos.rho*Va^2*obj.lref*A.ref*[0,cmy,cmz];
    end
  end

end