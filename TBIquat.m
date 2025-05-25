% rotation matrix from inertial frame to body frame using quartenions
function rotationmatrix = TBIquat(quarternion)

q0 = quarternion(1);
q1 = quarternion(2);
q2 = quarternion(3);
q3 = quarternion(4);

q0s = q0^2;
q1s = q1^2;
q2s = q2^2;
q3s = q3^2;



rt = [ q0s + q1s - q2s - q3s,  2*(q1*q2 + q0*q3),      2*(q1*q3 - q0*q2);
       2*(q1*q2 - q0*q3),      q0s - q1s + q2s - q3s,  2*(q0*q1 + q2*q3);
       2*(q0*q2 + q1*q3),      2*(q2*q3 - q0*q1),      q0s - q1s - q2s + q3s];

rotationmatrix = rt;