%%% System controller that produces a torque with magnetorquers to
%%% detumble a cubesat using the equation:

%%% torque = moment x magnetic_field(body frame)
%%% where: moment = -k*(ang_velocity x magnetic_field(body frame))
%%% therefore:
%%% torque = (k*(ang_velocity x magnetic_field(body frame))) x magnetic_field(body frame)

function torque = Controller(magfieldbody, angvel)

    % for step size = 1 second
    %k=4e4;

    % for tstep = 0.1 seconds
    k = 1e5;
    
    % for step size = 0.01 seconds
    % k=3e6;

    moment = k*cross(angvel, magfieldbody);

torque = cross(moment, magfieldbody);

