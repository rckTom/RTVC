function T = func_T_fp(kappa,iota)
    T = [ cos(kappa)*cos(iota), sin(iota), sin(kappa)*cos(iota);
         -cos(kappa)*sin(iota), cos(iota), -sin(kappa)*sin(iota);
         -sin(kappa)          , 0        , cos(kappa)];
end