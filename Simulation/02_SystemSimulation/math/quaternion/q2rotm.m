function rotm = q2rotm(q)
  q1  = q(1);
  q2  = q(2);
  q3  = q(3);
  q4  = q(4);
  q12 = q1*q1;
  q22 = q2*q2;
  q32 = q3*q3;
  q42 = q4*q4;

  rotm =  [ q12-q22-q32+q42,  2*(q1*q2+q3*q4),  2*(q1*q3-q2*q4); ...
            2*(q1*q2-q3*q4),  -q12+q22-q32+q42, 2*(q2*q3+q1*q4); ...
            2*(q1*q3+q2*q4),  2*(q2*q3-q1*q4),  -q12-q22+q32+q42 ];
end