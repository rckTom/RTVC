classdef class_attitudeControl
  properties
    Kp;
    Kd;
  end

  methods
    function obj = class_attitudeControl()

    end
    
    function BusAct = update(obj,qref,BusEoM)
      qRB = qmult(qref) * qconj(BusEoM.q);
      eulRB = [qRB(1) * sign(qRB(4))*2; qRB(2) * sign(qRB(4))*2; qRB(3) * sign(qRB(4))*2];
      wRIB  = eulRB .* 0.0005000;
      
      wRBB = wRIB + ([0,0,0]-BusEoM.omega_f) .* 0.0001;
      
      BusAct.kappa = wRBB(2);
      BusAct.iota = wRBB(3);
    end
  end
end