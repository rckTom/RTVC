classdef class_euler< class_odeSolver
  properties
    dt;
  end
  
  methods
    function obj = class_euler(dt)
      obj.dt = dt;
    end
    function y1 = solve(obj,yDotFunc,y0)
      y1 = y0 + yDotFunc(y0)*obj.dt;
    end
  end

end