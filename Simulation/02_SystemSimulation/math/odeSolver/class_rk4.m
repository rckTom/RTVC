classdef class_rk4 < class_odeSolver
  properties
    dt;
  end
  
  methods
    function obj = class_rk4(dt)
      obj.dt = dt;
    end
    function [y1,ydot] = solve(obj,yDotFunc,y0)
      k1 = yDotFunc(y0);
      k2 = yDotFunc(y0+obj.dt/2*k1);
      k3 = yDotFunc(y0+obj.dt/2*k2);
      k4 = yDotFunc(y0+obj.dt*k3);
      ydot = 1/6*(k1+2*k2+2*k3+k4);
      y1 = y0 + obj.dt*ydot;
    end
  end

end