% This piece of code represents a bot that is traveling from a start
% location to a goal location using Dijkstra's Algorithm. The bot can head
% up, down, left, right
clear all; close all; clc;

%% System parameters
L=20; W=15; % L x W field

%% xs=1; ys=1; % Start location {xs,ys}
xs=11; ys=5; % Start location {xs,ys}
xg=10; yg=13; % Goal location {xg,yg}

L1=3; W1=7; x1=4; y1=4; % Obstacle 1
L2=5; W2=3; x2=9; y2=8; % Obstacle 2

%% Create obstacle map
dx=0.1; dy=0.1; % Space resolution
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
    end
end


%% Display
figure(1);hold on;grid on;
surf(X,Y,Z); % for terrain
xlabel('x (m)');ylabel('y (m)');zlabel('z (m)');

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
BOT_STEPS=[-1 0; 1 0;0 -1; 0 1]; % -1 0 = one step in -i direction,
                                   % +1 0 = one step in +i direction,
                                   % 0 -1 = one step in -j direction,
                                   % 0 +1 = one step in +j direction,
Nsteps=4; % number of steps (rows) in BOT_STEPS

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
    %for i1=inex-1:inex+1
        %for j1=jnex-1:jnex+1
    for n=1:Nsteps
        i1=inex+BOT_STEPS(n,1); % i of next cell (after taking nth step from BOT_STEPS)
        j1=jnex+BOT_STEPS(n,2); % j of next cell (after taking nth step from BOT_STEPS)
            if i1>=1 && i1 <=Nx && j1>=1 && j1<=Ny % check if node (i1,j1) is in domain 1 to Nx
            if ~(i1==inex && j1==jnex) % not the nex node
                    if Z(i1,j1)==0 && VISIT(i1,j1)==0 % node is free
                       ctgnew=ctgnex+sqrt(((i1-inex)*dx)^2+((j1-jnex)*dy)^2); % cost from start to nex and nex to (i1,j1)
                       if ismember(NodeID(i1,j1),LN) % if cell (i1,j1) is in LN
                           if ctgnew < CTG(i1,j1) % if cost start to nex + nex to (i1,j1) is better
                               CTG(i1,j1)=ctgnew; % update ctg at node (i1,j1)
                               PARENT(i1,j1)=nex; % update parent of node (i1,j1)
                           end
                       else % if cell (i1,j1) is NOT in LN
                           LN=[LN NodeID(i1,j1)]; % insert node into the leaf node list
                           PARENT(i1,j1)=nex; % update parent of node (i1,j1)
                           CTG(i1,j1)=ctgnew; % assign cost to node (i1,j1)
                       end
                    end
            end
            end
    end
end

%% Extract trajectory from Start (xs, ys) to 
xg=3; yg=10; % Goal location (xg,yg)
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

figure(1); hold on; % continue figure 1
plot(Traj(:,1),Traj(:,2),'g','LineWidth',2);
plot(xg,yg,'mo','LineWidth',3);text(xg,yg,'Goal','color','m');
plot(xs,ys,'mo','LineWidth',3);text(xs,ys,'Start','color','r');

%% Computer optimal cost to go to any node (i,j) from the start node
% Dijkstra's algorithm
%%CTG=Inf(Nx,Ny)' % Cost-to-go
%%VISIT=zeroes(Nx,Ny); % is a cell (i,j) visited (expanded) or not
%%PARENT=zeros(Nx,Ny); % id of parent node of node (i,j)
%%is=xs/dx; js=ys/dy; % (i,j) of start location
%%CTG(is,js)=0; 
%%LN=[NodeID(is,js)]; % list of leaf nodes

%% Display
%%figure(1);hold on;grid on;
%%surf(X,Y,Z); % for terrain
%%xlabel('x (m)');ylabel('y (m)');zlabel('z (m)');

figure(2);hold on;grid on;
surf(X,Y,CTG, 'Edgecolor','None'); colorbar; colormap(jet); % for terrain
xlabel('x (m)');ylabel('y (m)');zlabel('z (m)');