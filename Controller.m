function torque = Controller(magfieldbody, angvel)

    %40000 is for step size = 1
    %k=4e4;

    % for tstep = 0.1
    k = 1e5;
    
    % for step size = 0.01
    % k=3e6;

   
    %torque_intermed = k*cross((cross(angvel, magfieldbody)), magfieldbody);

    %current = MagneticField()
    
    angvel_noised = GryoSensorModule(angvel);

    moment = k*cross(angvel_noised, magfieldbody);

torque = cross(moment, magfieldbody);

