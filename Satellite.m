% This function computes the derivative of the position, velocity,
% quarternions and angular velocity for numerical integration

function dstatedt = Satellite(t, state)
% format: position xyz (1:3), velocity in xyz directions (4:6), 
%         quarternion orientation (7:10), angular velocity (11:13)
% init_state = [x0; y0; z0; xdot0; ydot0; zdot0; quart0; p0; q0; r0];

quart = state(7:10); % [q1, q2, q3, q4]
angvel = state(11:13); % [p, q, r]
p = angvel(1);
q = angvel(2);
r = angvel(3);

%%% get inertial parameters %%%
InertialParams


%%% Gravity Model %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Get earth params
Planet 

dist_vect = state(1:3); %[x, y, z]
dist = norm(dist_vect);
distvect_hat = dist_vect/dist;

F_gravity = ((-G*M*mass)/(dist^2))*distvect_hat;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Translation Kinematics %%%%%%%
vel = state(4:6); % [vx, vy, vz]
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Translational Dynamics %%%%%%%
force = F_gravity;
accel = force/mass;
p = angvel(1);
q = angvel(2);
r = angvel(3);
rpqmat = [ 0, -r,  q;
           r,  0, -p;
          -q,  p,  0];

%accel = (1/mass)*state(1:3) - (rpqmat*vel);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Rotational Kinematics %%%%%%%%%%%%%%%%%%%
angvel_mat = [ 0  -p  -q  -r ;
               p   0   r  -q ;
               q  -r   0   p ;
               r   q  -p   0 ];

quartder = (1/2).*(angvel_mat*quart);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Rotational Dynamics %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
invInertia = inv(Inertia);
H = Inertia*angvel;

x = state(1);
y = state(2);
z = state(3);
magfieldinertial = MagneticField(x, y, z)';
magfieldbody = TBIquat(quart)*magfieldinertial;

MagnetorquerParams
mag_torque = Controller(magfieldbody, angvel);

angvelder = invInertia*(-cross(angvel, H) + mag_torque);  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


dstatedt = [vel; accel; quartder; angvelder];




