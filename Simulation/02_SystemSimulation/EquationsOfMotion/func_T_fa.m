function T = func_T_fa(alpha,beta)
  T = [cos(alpha)*cos(beta), -cos(alpha)*sin(beta), -sin(alpha);
       sin(beta)           , cos(beta)            , 0;
       sin(alpha)*cos(beta), -sin(alpha)*sin(beta), cos(alpha)];
end