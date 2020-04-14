clear all; close all; clc;

%% System parameters
% length is left to right
% width is top to bottom
L=50; W=40; % L x W field

%% xs=1; ys=1; % Start location {xs,ys}
xs=20; ys=5; % Start location {xs,ys}
HeadingS = pi/2; % Bot heading at start location
xg=10; yg=35; % Goal location {xg,yg}

L1=20; W1=4; x1=10; y1=25; % Obstacle 1
L2=5; W2=18; x2=10; y2=5; % Obstacle 2
L3=20; W3=3; x3=20; y3=33; % Obstacle 3
L4=5; W4=25; x4=36; y4=10; % Obstacle 4

%% Create obstacle map
dx=0.5; dy=0.5; % Space resolution
Nx=L/dx; Ny=W/dy; % Nx and Ny are number of cells in x and y directions
X=zeros(Nx,Ny); Y=zeros(Nx,Ny); % X and Y maps
Z=zeros(Nx,Ny); % Z map

for i=1:Nx
    for j=1:Ny
        X(i,j)=i*dx;
        Y(i,j)=j*dy;
    end
end
        
% i1=x1/dx; j1=y1/dy;
for i=1:Nx
    for j=1:Ny
        x=X(i,j);
        y=Y(i,j);
        if x >= x1 && x <= x1+L1 && y>=y1 && y <= y1+W1
            Z(i,j)=1;
        end
        if x >= x2 && x <= x2+L2 && y>=y2 && y <= y2+W2
            Z(i,j)=1;
        end
        if x >= x3 && x <= x3+L3 && y>=y3 && y <= y3+W3
            Z(i,j)=1;
        end
        if x >= x4 && x <= x4+L4 && y>=y4 && y <= y4+W4
            Z(i,j)=1;
        end
    end
end


%% Display
%figure(4);hold on;grid on;
%surf(X,Y,Z); % for terrain
%xlabel('x (m)');ylabel('y (m)');zlabel('z (m)');

%% Node numbers
NodeID=zeros(Nx,Ny);  % Node ID of any node (i,j)
Nodei=zeros(Nx*Ny,1); Nodej=zeros(Nx*Ny,1); % i and j of a node number
for i=1:Nx
   for j=1:Ny
       NodeID(i,j)= (i-1)*Ny + j;
       n=NodeID(i,j);
       Nodei(n)=i;
       Nodej(n)=j;
   end
end

%% Computer optimal cost to go to any node (i,j) from the start node
% Using Dijkstra's algorithm

CTG=Inf(Nx,Ny);
VISIT=zeros(Nx,Ny);
PARENT=zeros(Nx,Ny);
is=xs/dx; js=ys/dy % (i,j) of start location
CTG(is,js)=0;
LN=[NodeID(is,js)]; % list of leaf nodes

%% Define motion primitives (units)
% BOT_STEPS1: If heading is in +i direction (3:29)
BOT_STEPS1=[1 1;2 2;1 0]; % 1 1 = turn 90 deg left                         
                          % 2 2 = large turn 90 deg left
                          % 1 0 = straight
% BOT_STEPS2: If heading is in +j direction (3:29)            
BOT_STEPS2=[-1 1;-2 2;0 1]; % -1 1 = turn 90 deg left                           
                            % -2 2 = large turn 90 deg left
                            % 0 1 = straight
% BOT_STEPS3: If heading is in -i direction (3:29)
BOT_STEPS3=[-1 -1;-2 -2;-1 0]; % -1 -1 = turn 90 deg left                        
                               % -1 1 = large turn 90 deg left
                               % -1 0 = straight
% BOT_STEPS4: If heading is in -j direction (3:29)
BOT_STEPS4=[1 -1;2 -2;0 -1]; % 1 -1 = turn 90 deg left                        
                             % 2 -2 = large turn 90 deg left
                             % 0 -1 = straight
     
                             
% To calculate cost of big circle - C=2pi*r
% radius is 2dx. Distrance traveled is a quarter of a circle so divide by 4
% C=2*pi*2*dx/4 for total distance traveled
% C=pi*dx
COST_STEPS=[pi*dx/2 pi*dx dx]; % length traveled in each step. 90 degree turn is quarter circle
TURNANGLE=[pi/2 pi/2 0]; % Heading angle turned (radians) in each step
Nsteps=3; % number of steps (rows) in BOT_STEPS

Heading=NaN(Nx,Ny); % need to store heading angle at each cell
                    % It can be 0, pi/2, pi, or 3*pi/2
                    
Heading(is,js)=HeadingS; % give robot heading at start location                

while ~isempty(LN)
    LLN=length(LN); % number of entries in LN array
    CLN=zeros(1,LLN); % CTG from nodes in LN
    for k=1:LLN
       i=Nodei(LN(k));
       j=Nodej(LN(k));
       CLN(k)=CTG(i,j);
    end
    
    %% Find minimum cost entry
    [Min_val Min_index]=sort(CLN);
    nex=LN(Min_index(1)); % node to expand
    ctgnex=Min_val(1);
    
    % Sort the LN and CLN arrays based on cost values in CLN & remove 1st
    % entry (nex)
    LN=LN(Min_index); LN=LN(2:end);
    CLN=CLN(Min_index); CLN=CLN(2:end);
    
    %% Expand the minimum cost node
    inex=Nodei(nex); jnex=Nodej(nex); % (i,j) of the node to be expanded
    VISIT(inex,jnex)=1; % node nex is nvisited/sxpanded now [NEW]
   
    CurrentHeading=Heading(inex,jnex);
    if CurrentHeading==0
        BOT_STEPS=BOT_STEPS1;
    elseif CurrentHeading==pi/2
        BOT_STEPS=BOT_STEPS2;
    elseif CurrentHeading==pi      
        BOT_STEPS=BOT_STEPS3;       
    elseif CurrentHeading==3*pi/2        
        BOT_STEPS=BOT_STEPS4;
    else 
        display("Error CurrentHead: " + CurrentHeading);
        while 1
        end
    end         
    
    %for i1=inex-1:inex+1
        %for j1=jnex-1:jnex+1
    for n=1:Nsteps
        i1=inex+BOT_STEPS(n,1); % i of next cell (after taking nth step from BOT_STEPS)
        j1=jnex+BOT_STEPS(n,2); % j of next cell (after taking nth step from BOT_STEPS)
            if i1>=1 && i1 <=Nx && j1>=1 && j1<=Ny % check if node (i1,j1) is in domain 1 to Nx
            if ~(i1==inex && j1==jnex) % not the nex node
                    if Z(i1,j1)==0 && VISIT(i1,j1)==0 % node is free
                       %ctgnew=ctgnex+sqrt(((i1-inex)*dx)^2+((j1-jnex)*dy)^2); % cost from start to nex and nex to (i1,j1)
                       ctgnew=ctgnex + COST_STEPS(n); % node is free and not expanded yet
                       HeadingNew = CurrentHeading + TURNANGLE(n);
                       HeadingNew = wrapTo2Pi(HeadingNew);
                       if HeadingNew == 2*pi
                           HeadingNew = 0;
                       end
                       
                       if ismember(NodeID(i1,j1),LN) % if cell (i1,j1) is in LN
                           if ctgnew < CTG(i1,j1) % if cost start to nex + nex to (i1,j1) is better
                               CTG(i1,j1)=ctgnew; % update ctg at node (i1,j1)
                               Heading(i1,j1)=HeadingNew; % update heading
                               PARENT(i1,j1)=nex; % update parent of node (i1,j1)
                           end
                       else % if cell (i1,j1) is NOT in LN
                           LN=[LN NodeID(i1,j1)]; % insert node into the leaf node list
                           PARENT(i1,j1)=nex; % update parent of node (i1,j1)
                           CTG(i1,j1)=ctgnew; % assign cost to node (i1,j1)
                           Heading(i1,j1)=HeadingNew; % update heading
                       end
                    end
            end
            end
    end
end


figure(1);hold on;grid on;
surf(X,Y,CTG, 'Edgecolor','None'); colorbar; colormap(jet); % for terrain
xlabel('x (m)');ylabel('y (m)');zlabel('z (m)');
title('CTG (Cost-to-Go) Map');
plot(xs,ys,'mo','LineWidth',3);text(xs,ys,'Start','color','r');

%% Extract trajectory from Start (xs, ys) to 
% xg=3; yg=10; % Goal location (xg,yg)
ig=xg/dx; jg=yg/dy; % (i,j) of (xg,yg)
np = PARENT(ig,jg); % parent node of (ig,jg);
ns = NodeID(is,js); % node id of (is,js) or Start location
Traj = [xg yg];
while np~=ns % backtrack all the way to start node
    ip=Nodei(np);jp=Nodej(np); % (i,j) of node np
    x=X(ip,jp); y=Y(ip,jp); % (x,y) of node np
    Traj=[x y; Traj]; % add (x,y) to trajectory
    np=PARENT(ip,jp); % parent node of (ip,jp)
end

figure(2); hold on; % continue figure 1
surf(X,Y,Z); % for terrain
plot(Traj(:,1),Traj(:,2),'g','LineWidth',2);
plot(xg,yg,'mo','LineWidth',3);text(xg,yg,'Goal','color','m');
plot(xs,ys,'mo','LineWidth',3);text(xg,yg,'Start','color','r');

%% Plot veloctiy vector field (VVF) map
Vx=NaN(Nx,Ny); % x velocity at each cell
Vy=NaN(Nx,Ny); % y velocity at each cell
V=1; % Bot speed (assume its constant)
for i=1:Nx
    for j=1:Ny
        Angle=Heading(i,j);
        if ~isnan(Angle)
            Vx(i,j)=V*cos(Angle);
            Vy(i,j)=V*sin(Angle);
        end
    end
end

figure(3);hold on;grid on;
quiver(X,Y,Vx,Vy,0.5);
xlabel('x (m)');ylabel('y (m)');
title('VVF (Velocity Vector Field) Map');
xlim([0 L]);ylim([0 W]);

%figure(1); hold on; % continue figure 1
%plot(Traj(:,1),Traj(:,2),'g','LineWidth',2);
%plot(xg,yg,'mo','LineWidth',3);text(xg,yg,'Goal','color','m');
%plot(xs,ys,'mo','LineWidth',3);text(xs,ys,'Start','color','r');

