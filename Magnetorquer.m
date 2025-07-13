%%% Monitors the current being supplied to the magnetorquers and
%%% limits it to a specified maximum

function current = Magnetorquer(angvel, magfieldbody)
    
    k = 1e5;

    moment = k*(cross(angvel, magfieldbody));
    
    MagnetorquerParams

current = moment/(n*A);
