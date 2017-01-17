classdef class_BusEoM < Bus
  properties
    omega_f;
    omegadot_f;
    q;            %rotation Quaternion 
    qdot;
    v_f;
    vdot_f;
    r_e;
    v_e;
    a;
    b;
    R_Pf;
    Q_Pf;
    R_Af;
    Q_Af;
  end
end