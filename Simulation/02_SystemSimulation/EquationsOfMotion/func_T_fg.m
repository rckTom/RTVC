function T = func_T_fg(phi_f)
  phi = phi_f(1);
  theta = phi_f(2);
  psi = phi_f(3);
  T = [cos(theta)*cos(psi)                            , cos(theta)*sin(psi)                            , -sin(theta)        ;
       -cos(phi)*sin(psi)+sin(phi)*sin(theta)*cos(psi), cos(phi)*cos(psi)+sin(phi)*sin(theta)*sin(psi) , sin(phi)*cos(theta);
       sin(phi)*sin(psi)+cos(phi)*sin(theta)*cos(psi) , -sin(phi)*cos(psi)+cos(phi)*sin(theta)*sin(psi), cos(phi)*cos(theta)];
end