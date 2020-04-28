close all; clear all; clc;

%% Paramteres of UAV
m=2; % mass (kg)
Ixx=0.5; % moment of intertia about body x-axis (Kg-m^2)
Iyy=0.5; % moment of inertia about boyd y-axis (Kg-m^2)
Izz=1; % moment of inertia about body z-axis (Kg-mY2)
IUAV=[Ixx 0 0 ; 0 Iyy 0 ; 0 0 Izz]; % Inertia matrix
k=0.0001; % thrust coefficient of each motor
b=0.0001; % antitorque coefficient of each motor
IM=0.001; % moment of inertia of motor about its central axis (Kg/m^3)
Lx=0.2; % distance of motors fromx-axis (m)
Ly=0.2; % distance of motors from y-axis (m)
g=9.8; % gravity (m/s^2)

%% UAV state at t=0
x=0;y=0;z=0; % (m): x, y, and z in global frame
vx=0;vy=0;vz=0; % (m/s): first time-derivatives of x, y, and z in global frame
phi=0;theta=0;psi=0; % (rad): roll, pitch, and yaw angles in global frame
phid=0;thetad=0;psid=0; % (rad/2): first time-derivatives of roll, pitch, and yaw angles in global frame
wx=0;wy=0;wz=0; % (rad/s): roll rate (p), pitch rate (q), and yaw rate (r) in body frame

%% Simulation parameters
dt=0.005; % time-step (s)
tf=4; % plot from t=0 to t=tf (s)

%% Define trajectories (position, orientation, etc.)
Nt=tf/dt + 1; % number of time-steps
timtraj=zeros(Nt,1); % time
postraj=zeros(Nt,3); % position (x,y,z) in global frame 
atdtraj=zeros(Nt,3); % orientation/attitude (phi theta psi) in global frame
postraj(1,1)=x;postraj(1,2)=y;postraj(1,3)=z; % position at t-0 (time-step # 1)
atdtraj(1,1)=phi;atdtraj(1,2)=theta;atdtraj(1,3)=psi; % orientation at t=0 (time-step #1)

%% Control Inputs
wm=sqrt(m*g/(4*k)); % value of motor speeds to hover (total thrust F = m*g)
WM=[wm wm wm wm]; % RPMs of motors 1,2,3,4 to hover
WMtraj=zeros(Nt,4); % motors RPMs time-trajectory
WMtraj(1,1)=WM(1); WMtraj(1,2)=WM(2);WMtraj(1,3)=WM(3);WMtraj(1,4)=WM(4); % motors RPMs at t=0 (time-step # 1)

%% Quadcopter dynamics simulation (compute UAV state at t+dt using the UAV state and controls at t)
nt=1; % time-step 1 (t=0)
while nt<Nt
    %% Matrices at t
    %% Rotation matrix 3x3
    RM=[cos(psi)*cos(theta)  (cos(psi)*sin(theta)*sin(phi) - sin(psi)*cos(phi))  (cos(psi)*sin(theta)*cos(phi) + sin(psi)*sin(phi));
        sin(psi)*cos(theta)  (sin(psi)*sin(theta)*sin(phi) + cos(psi)*cos(phi))  (sin(psi)*sin(theta)*cos(phi) - cos(psi)*sin(phi));
       -sin(theta)            cos(theta)*sin(phi)                                 cos(theta)*cos(phi)]

    %% Transformation Matrix 3x3
    TM = [1  0          -sin(theta)          ;
          0  cos(phi)   cos(theta)*sin(phi)  ;
          0  -sin(phi)  cos(theta)*cos(phi) ]
      
    %% Inverse Transform Matrix 3x3
    INVTM = [1  sin(phi)*tan(theta)  cos(phi)*tan(theta) ;
             0  cos(phi)             -sin(phi)           ;
             0  sin(phi)/cos(theta)   cos(phi)/cos(theta)];

     %% First time-Derivative of Inverse Transform Matri 3x3
     DINVTM=[0  ((phid*cos(phi)*tan(theta)) + (thetad*sin(phi)/(cos(theta)^2)))       ((-phid*sin(phi)*cos(theta)) + (thetad*cos(phi)/(cos(theta)^2)))        ;
             0  (-phid*sin(phi))                                                      (-phid*cos(phi))                                                        ;
             0  ((phid*cos(phi)/cos(theta)) + (phid*sin(phi)*tan(theta)/cos(theta)))  ((-phid*sin(phi)/cos(theta)) + (thetad*cos(phi)*tan(theta)/cos(theta)))];
    
    %% Motor thrusts and torques at t
    F=k*(WM(1)^2 + WM(2)^2 + WM(3)^2 + WM(4)^2); %(N): total thrust force in body zb direction
    Tphi=k*Lx*(WM(3)^2 + WM(4)^2 - WM(1)^2 - WM(2)^2); % (N-m): Total torque about body xb axis
    Ttheta=k*Ly*(WM(2)^2 + WM(3)^2 - WM(1)^2 - WM(4)^2); % (n-m): Total torque about body zb axis [Ignore motor's own inertia]
    Tpsi=b*(WM(2)^2 + WM(4)^2 - WM(1)^2 - WM(3)^2); % (N-m): Total torque about body zb axis [Ignore motor's own inertia]
    
    %% STEP 1: %%%%%%%%%%%%%%%%%%%%%%%%%%
    % Newton's equations for translation: gives x y z acceleration in
    % global frame at t
    ax=(F/m)*(cos(psi)*sin(theta)*cos(phi) + sin(psi)*sin(phi)); % (m/s^2): x acceleration in global x direction at t
    ay=(F/m)*(sin(psi)*sin(theta)*cos(phi) - cos(psi)*sin(phi)); % (m/s^2): y acceleration in global y direction at t
    az=(F/m)*cos(theta)*cos(phi) - g; % (m/s^2): z acceleration in global z direction at t
    
    %% STEP 2: %%%%%%%%%%%%%%%%%%%%%%%%%%
    % Calculate velocity in global frame at t+dt
    vx1=vx + ax*dt; % (m/s): x velocity in global x direction at t+dt
    vy1=vy + ay*dt; % (m/s): y velocity in global y direction at t+dt
    vz1=vz + az*dt; % (m/s): z velocity in global z direction at t+dt
    
    % Calculate position in global frame at t+dt
    x1=x + ((vx1+vx)/2)*dt; % (m): x in global frame at t+dt
    y1=y + ((vy1+vy)/2)*dt; % (m): y in global frame at t+dt
    z1=z + ((vz1+vz)/2)*dt; % (m): z in global frame at t+dt

    %% STEP 3: %%%%%%%%%%%%%%%%%%%%%%%%%%
    % Netwon's equations for rotation: gives x y z angular acceleration in
    % body frame at t
    wxdot=(Iyy-Izz)*wy*wz/Ixx + Tphi/Ixx; % pdot (rad/s^2): x angular acceleration in body frame
    wydot=(Izz-Ixx)*wz*wx/Iyy + Ttheta/Iyy; % qdot (rad/s^2): y angular acceleration in body frame
    wzdot=(Ixx-Iyy)*wx*wy/Izz + Tpsi/Izz; % rdot (rad/s^2): z angular acceleration in body frame
    
    %% STEP 4: %%%%%%%%%%%%%%%%%%%%%%%%%%
    % Calculate angular acceleration in global frame at t+dt
    AAG=DINVTM*[wx wy wz]' + INVTM*[wxdot wydot wzdot]'; % (rad/s^2): angular acceleration in global frame at t
    phidd=AAG(1); % rad/2^2): x angular acceleration in global framte at t
    thetadd=AAG(2); % (rad/s^2): y angular acceleration in global frame at t
    psidd=AAG(3); % (rad/s^2): z angular acceleration in global frame at t
    
    %% STEP 5: %%%%%%%%%%%%%%%%%%%%%%%%%%
    % Calculate angular velocity in global frame at t+dt
    phid1=phid + phidd*dt; % (rad/s): x angular velocity in global frame at t+dt
    thetad1=thetad + thetadd*dt; % (rad/s): y angular velocity in global frame at t+dt
    psid1=psid + psidd*dt; % (rad/s): z angular velocity in global frame at t+dt
    
    % Calculate angular position in global frame at t+dt
    phi1=phi + ((phid1+phid)/2)*dt; % (rad): roll angle in global frame at t+dt
    theta1=theta+ ((thetad1+thetad)/2)*dt; % (rad): pitch angle in global frame at t+dt
    psi1=psi + ((psid1+psid)/2)*dt; % (rad): roll angle in global frame at t+dt
    
    %% STEP 6: %%%%%%%%%%%%%%%%%%%%%%%%%%
    % Calculate angular velocity in body frame at t+dt
    TM1=[1  0           -sin(theta1)          ; % Transformation matrix at t+dt
         0  cos(phi1)   cos(theta1)*sin(phi1) ;
         0  -sin(phi1)  cos(theta1)*cos(phi1)];
     AVB=TM1*[phid1 thetad1 psid1]'; % (rad/2): angular velocity in body frame at t+dt
     wx1=AVB(1); % p (rad/s): x angular velocity in body frame at t+dt
     wy1=AVB(2); % q (rad/s): y angular velocity in body frame at t+dt
     wz1=AVB(3); % r (rad/s): z angular velocity in body frame at t+dt
     
     %% Store trajectory at t+dt
     nt=nt+1; % t -> t+dt
     timtraj(nt)=(nt-1)*dt; % (s): time at t+dt
     postraj(nt,1)=x1;postraj(nt,2)=y1;postraj(nt,3)=z1; % (m): x, y, and z in global frame at t+dt
     atdtraj(nt,1)=phi1; atdtraj(nt,2)=theta1;atdtraj(nt,3)=psi1; % (rad): roll, pitch, and yaw angles in global frame at t+dt
     
     %% Update UAV state: t -> t+dt (we need these values in STEPS 1 to 6)
     x=x1;y=y1;z=z1;
     vx=vx1;vy=vy1;vz=vz1;
     phi=phi1;theta=theta1;psi=psi1;
     phid=phid1;thetad=thetad1;psid=psid1;
     wx=wx1;wy=wy1;wz=wz1;
     
     %% user input: Update controls: t-> t+dt
     
    % WM(1)=wm;WM(2)=wm;WM(3)=wm;WM(4)=wm;
    % WMtraj(nt,1)=WM(1);WMtraj(nt,2)=WM(2);WMtraj(nt,3)=WM(3);WMtraj(nt,4)=WM(4);
     
     if timtraj(nt)>=0.5 && timtraj(nt)<=0.6
         WM(2)=wm+20;WM(3)=wm+20; % pitching forward about yb-axis
     elseif timtraj(nt)>=0.7 && timtraj(nt)<=0.8
         WM(1)=wm+20;WM(4)=wm+20; % pitching backward about yb-axis
     else
         WM(1)=wm;WM(2)=wm;WM(3)=wm;WM(4)=wm;
     end
     WMtraj(nt,1)=WM(1);WMtraj(nt,2)=WM(2);WMtraj(nt,3)=WM(3);WMtraj(nt,4)=WM(4);
end
     
%% plot 3-D trajectory x,y,z
figure(1);hold on;grid on;view(3)
plot3(postraj(:,1),postraj(:,2),postraj(:,3));
xlabel('x (m)');ylabel('y (m)');zlabel('z (m)'); % axis equal

%% plot attitude/orientation angles (roll(phi), pitch(theta), yaw(psi)) in global frame
figure(2);hold on;
subplot(3,1,1);
plot(timtraj,atdtraj(:,1));xlabel('time(s)');ylabel('Roll \phi (rad)');
subplot(3,1,2);
plot(timtraj,atdtraj(:,2));xlabel('time(s)');ylabel('Pitch \theta(rad)');
subplot(3,1,3);
plot(timtraj,atdtraj(:,3));xlabel('time (s)');ylabel('Yaw \psi (rad)');
    
%% Plot motors (1,2,3,4) speeds
figure(3);hold on;
subplot(2,2,1);
plot(timtraj,WMtraj(:,4));xlabel('time (s)');ylabel('Motor 4 RPM');
subplot(2,2,2);
plot(timtraj,WMtraj(:,1));xlabel('time (s)');ylabel('Motor 1 RPM');
subplot(2,2,3);
plot(timtraj,WMtraj(:,3));xlabel('time (s)');ylabel('Motor 3 RPM');
subplot(2,2,4);
plot(timtraj,WMtraj(:,2));xlabel('time (s)');ylabel('Motor 2 RPM');
    
    
         
         
         