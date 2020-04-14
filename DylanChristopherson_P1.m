%% Problem 1

% At an instant, a UAV's roll, pitch, and yaw angles in global frame are
% 10, 15, and 70 degrees, respectively. Total thrust F by motors is 20 N.
% A) What is the rotation matrix from body to global frame?
% B) What are the components of F in global X, Y, and Z?

% Make sure to convert angle to radians
phi=10*pi/180
theta=15*pi/180
psi=70*pi/180

% RM : Rotation Matrix
% Rotation matrix from body frame to global inertial frame
RM = [cos(psi)*cos(theta) cos(psi)*sin(theta)*sin(phi)-sin(psi)*cos(phi)  cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi);
     sin(psi)*cos(theta)  sin(psi)*sin(theta)*sin(phi)+cos(psi)*cos(phi)  sin(psi)*sin(theta)*cos(phi)-cos(psi)*sin(phi);
    -sin(theta)           cos(theta)*sin(phi)                             cos(theta)*cos(phi)]

% Bf : Body frame
% Thrust is 20 Netwons in the z direction on the body frame
Bf = [0 0 20] 

% Gf : Glboal frame
% Global frame = Rotation matrix * Body frame
Gf = RM * Bf' 
