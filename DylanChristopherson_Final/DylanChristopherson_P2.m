clear all; close all; clc;

%% Vehicle dynamics constraints
vmax = 10; % (m/s): maximum speed
anmax = 20; % (m/s^2): maximum normal acceleration
atmax = 10; % (m/s^2): maximum tangential acceleration
Rmin = (vmax^2)/anmax; % (M): minimum radius turn at maximum speed and maximum normal acceleration

%% Obstacle field
L=100; W=100; % (m): L x W field
L1=30; W1=4; x1=10; y1=30; % (m): Obstacle 1
L2=8; W2=60; x2=50; y2=10; % (m): Obstacle 2
L3=50; W3=4; x3=30; y3=50; % (m): Obstacle 2

%% Start position
xs=30; ys=15; % (M): Start location (xs,ys)
HeadingS=pi/2; % (radians): Bot heading at start location: it can be 0, pi/2, pi, and 3*pi/2
SpeedLS=2; % (no unit): Bot speed level at start

%% Discretization
nv = 6; % number of speed levels
dxy = Rmin/(nv-1); % (m): space resolution
VL = zeros(1,6); % 1 to nv speed level array

for i=1:nv
    VL(i)=sqrt((i-1)*dxy*anmax); % (m/s): speed level i
end

%% Create obstacle map
dx=dxy; dy=dxy; % Space resolution
Nx=L/dx; Ny=W/dy; % Nx and Ny are number of cells in x and y directions
X=zeros(Nx,Ny); Y=zeros(Nx,Ny); % X and Y maps
Z=zeros(Nx,Ny); % Z map

for i=1:Nx
    for j=1:Ny
        X(i,j)=i*dx;
        Y(i,j)=j*dy;
    end
end

for i=1:Nx
    for j=1:Ny
        x=X(i,j);
        y=Y(i,j);
        if x>=x1 && x<=x1+L1 && y>=y1 && y<=y1+W1
            Z(i,j)=1;
        end
        if x>=x2 && x<=x2+L2 && y>=y2 && y<=y2+W2
            Z(i,j)=1;
        end
        if x>=x3 && x<=x3+L3 && y>=y3 && y<=y3+W3
            Z(i,j)=1;
        end
    end
end

%% Plot obstacles map Z
figure(1);hold on; grid on;
subplot(1,2,1);hold on;
surf(X,Y,Z);colormap(jet);colorbar;
plot(xs,ys,'ro','LineWidth',3);text(xs,ys,'Start','color','r');
xlabel('x (m)');ylabel('y (m)');zlabel('z (m)');
axis equal;xlim([0 L]);ylim([0 W]);

%% Maximum allowable speed at cells
dstopmin = (vmax^2)/(2*atmax); 
nsafe = ceil(dstopmin/dxy);
VAL = zeros(Nx,Ny);

for i=1:Nx
    for j=1:Ny
        if Z(i,j)==0
            x=X(i,j);y=Y(i,j);
            dobsclosest=Inf;
            
            for i1=i-nsafe:i+nsafe
                for j1=j-nsafe:j+nsafe
                    if i1>=1 && i1<=Nx && j1>=1 && j1<=Ny
                        if Z(i1,j1)>0
                            x1=X(i1,j1);y1=Y(i1,j1);
                            d=sqrt((x-x1)^2 + (y-y1)^2);
                            if d<dobsclosest
                                dobsclosest=d;
                            end
                        end
                    end
                end
            end
            
            if dobsclosest < Inf
                v=sqrt(2*atmax*dobsclosest);
                v1=1;
                for k=1:nv
                    if v>VL(k)
                        v1=k;
                    end
                end
                VAL(i,j)=v1;
            else
                VAL(i,j)=nv;
            end
            
        end
    end
end

%% Plot maximum allowed speed level map VAL
figure(1);hold on;grid on;
subplot(1,2,2);hold on;
surf(X,Y,VAL);colormap(jet);colorbar;
xlabel('x (m)');ylabel('y (m)');zlabel('Velocity level');
axis equal;xlim([0 L]);ylim([0 W]);

%% Node numbers
NodeID=zeros(Nx,Ny);  % Node ID of any node (i,j)
Nodei=zeros(Nx*Ny,1); Nodej=zeros(Nx*Ny,1); % i and j of a node number

for i=1:Nx
   for j=1:Ny
       NodeID(i,j)=(i-1)*Ny + j;
       n=NodeID(i,j);
       Nodei(n)=i;
       Nodej(n)=j;
   end
end

%% Initialization
CTG=Inf(Nx,Ny);
VISIT=zeros(Nx,Ny);
PARENT=zeros(Nx,Ny);
SPEEDL=ones(Nx,Ny);
SPEED=zeros(Nx,Ny);
Heading=NaN(Nx,Ny);

is=xs/dx;js=ys/dy;

%% Initial conditions at start
CTG(is,js)=0;
Heading(is,js)=HeadingS;
SPEEDL(is,js)=SpeedLS;
SPEED(is,js)=VL(SpeedLS);

%% Define motjion primitives (units)
%% Generate for heading 0 (+x direction)
for k=1:nv 
    for c=1:6
        % case 1: straight dxy with constant speed
        % case 2: turn pi/2 left with constant speed
        % case 3: straight dxy and increase/decrease speed
        % case 4: turn pi/2 left and increase/decrease speed
        % case 5 (mirror of case 2): turn pi/2 right with constant speed
        % case 6 (mirror of case 4): turn pi/2 right and increase/decrease speed
        v=VL(k);
        
        if c==1
            MPs(k,c).IJ1=[1 0]; % heading 0: go to (i+1,j)
            MPs(k,c).IJ2=[0 1]; % heading pi/2: go to (i,j+1)
            MPs(k,c).IJ3=[-1 0]; % heading pi: go to (i-1,j)
            MPs(k,c).IJ4=[0 -1]; % heading 3*pi/2: go to (i,j-1)
            MPs(k,c).v1=k; % speed level at next cell
            MPs(k,c).turn=0; % no turning
        elseif c==2            
            MPs(k,c).IJ1=[k-1 k-1]; % heading 0: go to (i+k-1,j+k-1)
            MPs(k,c).IJ2=[-k+1 k-1]; % heading pi/2: go to (i-k+1,j+k-1)
            MPs(k,c).IJ3=[-k+1 -k+1]; % heading pi: go to (i-k+1,j-k+1)
            MPs(k,c).IJ4=[k-1 -k+1]; % heading 3*pi/2: go to (i+k-1,j-k+1)
            MPs(k,c).v1=k; % speed level at next cell
            MPs(k,c).turn=pi/2; % turning left by pi/2
         elseif c==5    
            MPs(k,c).IJ1=[k-1 -k+1]; % heading 0: go to (i+k-1,j+k-1)
            MPs(k,c).IJ2=[k-1 k-1]; % heading pi/2: go to (i-k+1,j+k-1)
            MPs(k,c).IJ3=[-k+1 k-1]; % heading pi: go to (i-k+1,j-k+1)
            MPs(k,c).IJ4=[-k+1 -k+1]; % heading 3*pi/2: go to (i+k-1,j-k+1)
            MPs(k,c).v1=k; % speed level at next cell
            MPs(k,c).turn=-pi/2; % turning left by pi/2
        elseif c==3    
            MPs(k,c).IJ1=[1 0]; % heading 0: go to (i+k-1,j+k-1)
            MPs(k,c).IJ2=[0 1]; % heading pi/2: go to (i-k+1,j+k-1)
            MPs(k,c).IJ3=[-1 0]; % heading pi: go to (i-k+1,j-k+1)
            MPs(k,c).IJ4=[0 -1]; % heading 3*pi/2: go to (i+k-1,j-k+1)
            vv=sqrt(v^2 + 2*atmax*dxy);
            vv1=1; % speed level 1 (zero speed)
            for kk=1:nv
                if vv>VL(kk)
                    vv1=kk;
                end
            end
            vv=VL(vv1);
            MPs(k,c).v1=vv1; % speed level at next cell
            MPs(k,c).turn=0; % turning left by pi/2   
        elseif c==4  
            MPs(k,c).IJ1=[k-1 k-1]; % heading 0: go to (i+k-1,j+k-1)
            MPs(k,c).IJ2=[-k+1 k-1]; % heading pi/2: go to (i-k+1,j+k-1)
            MPs(k,c).IJ3=[-k+1 -k+1]; % heading pi: go to (i-k+1,j-k+1)
            MPs(k,c).IJ4=[k-1 -k+1]; % heading 3*pi/2: go to (i+k-1,j-k+1)
            MPs(k,c).v1=k; % speed level at next cell
            MPs(k,c).turn=pi/2; % turning left by pi/2
        elseif c==6    
            MPs(k,c).IJ1=[k-1 -k+1]; % heading 0: go to (i+k-1,j+k-1)
            MPs(k,c).IJ2=[k-1 k-1]; % heading pi/2: go to (i-k+1,j+k-1)
            MPs(k,c).IJ3=[-k+1 k-1]; % heading pi: go to (i-k+1,j-k+1)
            MPs(k,c).IJ4=[-k+1 -k+1]; % heading 3*pi/2: go to (i+k-1,j-k+1)
            MPs(k,c).v1=k; % speed level at next cell
            MPs(k,c).turn=-pi/2; % turning left by pi/2
        end
    end
end

%% Computer optimal cost to go to any node/cell (i,j) from the start node/cell using Dijkstra's algorithm
LN=[NodeID(is,js)];
while ~isempty(LN)
    LLN=length(LN);
    CLN=zeros(1,LLN);
    for k=1:LLN
        i=Nodei(LN(k));
        j=Nodej(LN(k));
        CLN(k)=CTG(i,j);
    end

    %% Find minimum cost entry 
    [Min_val Min_index]=sort(CLN);
    nex=LN(Min_index(1));
    ctgnex=Min_val(1);

    %% Sort the LN and CLN arrays based on cost values in CLN & remove 1st entry (nex)
    LN=LN(Min_index);LN=LN(2:end);
    CLN=CLN(Min_index);CLN=CLN(2:end);

    %% Expand the minimum cost node
    inex=Nodei(nex);jnex=Nodej(nex);
    VISIT(inex,jnex)=1;
    CurrentHeading=Heading(inex,jnex);
    CurrentSpeedL=SPEEDL(inex,jnex);
    CurrentSpeed=VL(CurrentSpeedL);

    for c=1:6
        % Heading
        if CurrentHeading==0
            IJNext=MPs(CurrentSpeedL,c).IJ1;
        elseif CurrentHeading==pi/2
            IJNext=MPs(CurrentSpeedL,c).IJ2;
        elseif CurrentHeading==pi
            IJNext=MPs(CurrentSpeedL,c).IJ3;
        elseif CurrentHeading==3*pi/2
            IJNext=MPs(CurrentSpeedL,c).IJ4;
        else
            display("Error CurrentHead");
            while 1
            end
        end

        i1=inex + IJNext(1); % i of next cell
        j1=jnex + IJNext(2); % j of next cell
        if i1>=1 && i1<=Nx && j1>=1 && j1<=Ny % Check if node (i1,j1) is in domain 1 to Nx and 1 to Ny
            if ~(i1==inex && j1==jnex)
                if Z(i1,j1)==0 && VISIT(i1,j1)==0
                  
                    %% Check maximum allowed speed at (i1,j1)
                    vln=MPs(CurrentSpeedL,c).v1;
                    if vln > VAL(i1,j1)
                        vln=VAL(i1,j1);
                    end
                    NextSpeed=VL(vln);
                   
                    %% Compute cost to go to (i1,j1) from (inex,jnex)
                    vavg=(CurrentSpeed+NextSpeed)/2; % (m/s): average speed
                    if c==1 || c==3
                        cost=dxy/vavg; % (s): cost (time0
                    else
                        dis=pi*(CurrentSpeedL-1)*dxy/2; % (M): quarter circle with radius (CurrentSpeedL-1)*dxy
                        cost=dis/vavg;
                    end
                    ctgnew = ctgnex + cost;
                    HeadingNew=CurrentHeading + MPs(CurrentSpeedL,c).turn;
                    HeadingNew = wrapTo2Pi(HeadingNew);
                    if HeadingNew == 2*pi
                        HeadingNew=0;
                    end
                    if ismember(NodeID(i1,j1),LN)
                        if ctgnew < CTG(i1,j1)
                            CTG(i1,j1)=ctgnew;
                            Heading(i1,j1)=HeadingNew;
                            SPEEDL(i1,j1)=vln;
                            SPEED(i1,j1)=VL(vln)
                            PARENT(i1,j1)=nex;
                        end
                    else
                        LN=[LN NodeID(i1,j1)]; % insert node into the leaf node list
                        CTG(i1,j1)=ctgnew;
                        Heading(i1,j1)=HeadingNew;
                        SPEEDL(i1,j1)=vln;
                        SPEED(i1,j1)=VL(vln);
                        PARENT(i1,j1)=nex;
                    end
                end
            end
        end
    end
    
end
                        
%% Plot CTG and Speed map
figure(2);hold on;grid on;
subplot(1,2,1);hold on;
surf(X,Y,CTG,'Edgecolor','None'); colorbar; colormap(jet);
xlabel('x (m)');ylabel('y (m)');zlabel('CTG (s)');
axis equal;xlim([0 L]);ylim([0 W]);
title('CTG (Cost-to-go) Map');

subplot(1,2,2);hold on;
surf(X,Y,SPEED,'Edgecolor','None'); colorbar; colormap(jet);
xlabel('x (m)');ylabel('y (m)');zlabel('Speed (m/s)');
axis equal;xlim([0 L]);ylim([0 W]);
title('Speed Map')

%% Extract trajectory from Start (xs,ys) to any point (xg,yg)
xg=65; yg=60; % Goal location (xg,yg)   % 30, 45
ig=xg/dx; jg=yg/dy;
np=PARENT(ig,jg);
ns=NodeID(is,js);
Traj=[xg yg];
while np~=ns
    ip=Nodei(np);jp=Nodej(np);
    x=X(ip,jp);y=Y(ip,jp);
    Traj=[x y; Traj];
    np=PARENT(ip,jp);
end

%% Plot trajectory from start to goal
figure(3); hold on;
surf(X,Y,Z);colormap(jet);
plot(Traj(:,1),Traj(:,2),'g','LineWidth',2);
% plot(xg,yg,'mo','LineWidth',3);text(xg,yg,'Goal','color','m');
axis equal;xlim([0 L]);ylim([0 W]);

%% Plot velocity vector field (VVF) map
Vx=NaN(Nx,Ny);
Vy=NaN(Nx,Ny);
for i=1:Nx
    for j=1:Ny
        if Z(i,j)==0 && ~isnan(CTG(i,j))
            Angle=Heading(i,j);
            v1=SPEEDL(i,j);
            v=VL(v1);
            Vx(i,j)=v*cos(Angle);
            Vy(i,j)=v*sin(Angle);
        end
    end
end

%% Plot VVF map
figure(4);hold on; grid on;
quiver(X,Y,Vx,Vy,0.5);
xlabel('x (m)');ylabel('y (m)');
title('VVF (Velocity Vector Field) Map');
axis equal;xlim([0 L]);ylim([0 W]);

