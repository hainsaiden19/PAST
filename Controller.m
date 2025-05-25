function torque = Controller(magfieldbody, angvel)

%4000 is for step size = 0.1 and 1
%k=4e4;

% for step size = 0.01
k=3e6;


torque = k*cross((cross(angvel, magfieldbody)), magfieldbody);

