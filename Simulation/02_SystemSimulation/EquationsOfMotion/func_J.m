function T = J(phi_f)
  phi = phi_f(1);
  theta = phi_f(2);
  psi = phi_f(3);
  
  T = 1/cos(theta) * [cos(theta), sin(phi)*sin(theta), cos(phi)*sin(theta) ;
                      0         , cos(phi)*cos(theta), -sin(phi)*cos(theta);
                      0         , sin(phi)           , cos(phi)];

end