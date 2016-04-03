classdef class_Rail
  properties
    l;
    isRailCleared;
    isGroundHit;
    isStarted
  end
  
  methods
    function obj = class_Rail()
    
    end
    function BusEoM = update(obj,BusEoM)
      
      %check if rail is allready cleared
      if(BusEoM.r_e(3) <= -1)
        BusEoM.q =    [0.70711;   0.00000;  -0.70711;   0.00000];
        
      end
    
      %check if we are hitting ground
      if(BusEoM.r_e(3) >= 0)
        BusEoM.r_e(3) = 0;
        BusEoM.v_f = [0;0;0];
        obj.isGroundHit = true;
      else
        obj.isGroundHit = false;
      end
    end
  end

end