classdef class_atmosphere
  properties

  end
  methods
    function obj = class_BusAtmos()
      
    end

    function BusAtmos = update(obj,BusEoM)
          h = -BusEoM.r_e(3);
          BusAtmos.rho = 1.25;
          if ((h>=0)&&(h<=11*10^3))
              BusAtmos.T = 288.15-6.5*10^-3*h;
          elseif ((h>11*10^3)&&(h<=20*10^3))
              BusAtmos.T = 216.65;
          elseif ((h>20*10^3)&&(h<=32*10^3))
              BusAtmos.T = 216.65+1*10^-3*(h-20*10^3);
          elseif ((h>32*10^3)&&(h<=47*10^3))
              BusAtmos.T = 228.65+2.8*10^-3*(h-32*10^3);
          elseif ((h>47*10^3)&&(h<=52*10^3))
              BusAtmos.T = 270.65;
          elseif ((h>52*10^3)&&(h<=61*10^3))
              BusAtmos.T = 270.65-2*10^-3*(h-52*10^3);
          elseif ((h>61*10^3)&&(h<=79*10^3))
              BusAtmos.T = 252.65 - 4*10^-3*(h-61*10^3);
          elseif ((h>79*10^3)&&(h<=88*10^3))
              BusAtmos.T = 180.65;
          else
              warning(['Altitude of' num2str(h) ' meters is outside of DIN ISO 2535 Standard Atmosphere Model specifications.Using sea level values instead' ]);
               T = 288.15;
          end
      end
  end
end