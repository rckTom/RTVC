function result = rotx( angle )
    result = [1 0 0;
              0 cos(angle) -sin(angle);
              0 sin(angle) cos(angle)];
end

