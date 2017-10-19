%function [J,U]=DP_valueiteration

% DEFINE GLOBAL PARAMETERS
global resR resThetaPhi rdim thetadim phidim nr ntheta nphi dr dtheta dphi ...
    perr vGoal vObst vMove vInitial gamma
% - minimum grid resolution for XY
resR = 1.0;
resThetaPhi = 10.0;
% - dimensions of the world
rdim = [0.0 10.0];
thetadim = [0.0 360.0];
phidim = [0.0 360.0];
% - vectors of each variable
rVec=rdim(1):resR:rdim(2);
thetaVec=thetadim(1):resThetaPhi:thetadim(2);
phiVec=phidim(1):resThetaPhi:phidim(2);
% - number of grid cells
nr=length(rVec);
ntheta=length(thetaVec);
nphi=length(phiVec);
% - probability of going the wrong way
perr = 0;
% - attenuation rate
gamma = 1;
% - value of goal, collision, movement
vGoal = 100;
vObst = -100;
vMove = -1;
% - initial guess at all values
vInitial = 0;

% DEFINE OBSTACLE AND GOAL LOCATIONS
isobst=zeros(nr,ntheta,nphi);
isobst(nr,:,:)=1;
isgoal=zeros(nr,ntheta,nphi);
for itheta=1:ntheta
    for iphi=1:nphi
        if (itheta==iphi)
        isgoal(1,itheta,iphi)=1;
        end
    end
end

J = zeros(nr,ntheta,nphi);
U = zeros(nr,ntheta,nphi);

% DEFINE INITIAL GUESS AT VALUE AND POLICY
for i=1:nr
    for j=1:ntheta
        for k=1:nphi
            if (isobst(i,j,k))
                J(i,j,k) = vObst;
                U(i,j,k) = 0;
            elseif (isgoal(i,j,k))
                J(i,j,k) = vGoal;
                U(i,j,k) = 0;
            else
                J(i,j,k) = vInitial;
                U(i,j,k) = ceil(4*rand);
            end
        end
    end
end

% DO VALUE ITERATION
T=100;
for t=2:T
    % Uncomment one of these things to slow the display down.
    % pause;
    % pause(0.1);
    % drawnow;
    
    % Iterate over all states.
    Jprev = J;
    Uprev = U;
    for i=1:nr
        for j=1:ntheta
            for k=1:nphi
                % Update the value/policy for every square that is not an
                % obstacle and not a goal.
                if ((~isgoal(i,j,k)) && (~isobst(i,j,k)))
                    cStay=Jprev(i,j,k);
                    if (i==1)
                        cRPos=Jprev(i+1,j,k);
                        cRNeg=Jprev(i,j,k);
                        if (j==1)
                            if (k==1)
                                cThetaPos=Jprev(i,j+1,k);
                                cThetaNeg=Jprev(i,ntheta,k);
                                cPhiPos=Jprev(i,j,k+1);
                                cPhiNeg=Jprev(i,j,nphi);
                            elseif (k==nphi)
                                cThetaPos=Jprev(i,j+1,k);
                                cThetaNeg=Jprev(i,ntheta,k);
                                cPhiPos=Jprev(i,j,1);
                                cPhiNeg=Jprev(i,j,k-1);
                            else
                                cThetaPos=Jprev(i,j+1,k);
                                cThetaNeg=Jprev(i,ntheta,k);
                                cPhiPos=Jprev(i,j,k+1);
                                cPhiNeg=Jprev(i,j,k-1);
                            end
                        elseif (j==ntheta)
                            if (k==1)
                                cThetaPos=Jprev(i,1,k);
                                cThetaNeg=Jprev(i,j-1,k);
                                cPhiPos=Jprev(i,j,k+1);
                                cPhiNeg=Jprev(i,j,nphi);
                            elseif (k==nphi)
                                cThetaPos=Jprev(i,1,k);
                                cThetaNeg=Jprev(i,j-1,k);
                                cPhiPos=Jprev(i,j,1);
                                cPhiNeg=Jprev(i,j,k-1);
                            else
                                cThetaPos=Jprev(i,1,k);
                                cThetaNeg=Jprev(i,j-1,k);
                                cPhiPos=Jprev(i,j,k+1);
                                cPhiNeg=Jprev(i,j,k-1);
                            end
                        else
                            if (k==1)
                                cThetaPos=Jprev(i,j+1,k);
                                cThetaNeg=Jprev(i,j-1,k);
                                cPhiPos=Jprev(i,j,k+1);
                                cPhiNeg=Jprev(i,j,nphi);
                            elseif (k==nphi)
                                cThetaPos=Jprev(i,j+1,k);
                                cThetaNeg=Jprev(i,j-1,k);
                                cPhiPos=Jprev(i,j,1);
                                cPhiNeg=Jprev(i,j,k-1);
                            else
                                cThetaPos=Jprev(i,j+1,k);
                                cThetaNeg=Jprev(i,j-1,k);
                                cPhiPos=Jprev(i,j,k+1);
                                cPhiNeg=Jprev(i,j,k-1);
                            end
                        end
                    else
                        cRPos=Jprev(i+1,j,k);
                        cRNeg=Jprev(i-1,j,k);
                        if (j==1)
                            if (k==1)
                                cThetaPos=Jprev(i,j+1,k);
                                cThetaNeg=Jprev(i,ntheta,k);
                                cPhiPos=Jprev(i,j,k+1);
                                cPhiNeg=Jprev(i,j,nphi);
                            elseif (k==nphi)
                                cThetaPos=Jprev(i,j+1,k);
                                cThetaNeg=Jprev(i,ntheta,k);
                                cPhiPos=Jprev(i,j,1);
                                cPhiNeg=Jprev(i,j,k-1);
                            else
                                cThetaPos=Jprev(i,j+1,k);
                                cThetaNeg=Jprev(i,ntheta,k);
                                cPhiPos=Jprev(i,j,k+1);
                                cPhiNeg=Jprev(i,j,k-1);
                            end
                        elseif (j==ntheta)
                            if (k==1)
                                cThetaPos=Jprev(i,1,k);
                                cThetaNeg=Jprev(i,j-1,k);
                                cPhiPos=Jprev(i,j,k+1);
                                cPhiNeg=Jprev(i,j,nphi);
                            elseif (k==nphi)
                                cThetaPos=Jprev(i,1,k);
                                cThetaNeg=Jprev(i,j-1,k);
                                cPhiPos=Jprev(i,j,1);
                                cPhiNeg=Jprev(i,j,k-1);
                            else
                                cThetaPos=Jprev(i,1,k);
                                cThetaNeg=Jprev(i,j-1,k);
                                cPhiPos=Jprev(i,j,k+1);
                                cPhiNeg=Jprev(i,j,k-1);
                            end
                        else
                            if (k==1)
                                cThetaPos=Jprev(i,j+1,k);
                                cThetaNeg=Jprev(i,j-1,k);
                                cPhiPos=Jprev(i,j,k+1);
                                cPhiNeg=Jprev(i,j,nphi);
                            elseif (k==nphi)
                                cThetaPos=Jprev(i,j+1,k);
                                cThetaNeg=Jprev(i,j-1,k);
                                cPhiPos=Jprev(i,j,1);
                                cPhiNeg=Jprev(i,j,k-1);
                            else
                                cThetaPos=Jprev(i,j+1,k);
                                cThetaNeg=Jprev(i,j-1,k);
                                cPhiPos=Jprev(i,j,k+1);
                                cPhiNeg=Jprev(i,j,k-1);
                            end
                        end
                    end
                    

                    % Now compute the total expected cost for each of the seven
                    % possible actions (stay, positive/negative in r direction
                    % positive/negative in theta direction, positive/negative in
                    % phi direction).
                    JStay = vMove+gamma*((1-perr)*cStay + (perr/6)*cRPos + (perr/6)*cRNeg...
                        + (perr/6)*cThetaPos + (perr/6)*cThetaNeg...
                        + (perr/6)*cPhiPos + (perr/6)*cPhiNeg);
                    JRPos = vMove+gamma*((1-perr)*cRPos + (perr/6)*cStay + (perr/6)*cRNeg...
                        + (perr/6)*cThetaPos + (perr/6)*cThetaNeg...
                        + (perr/6)*cPhiPos + (perr/6)*cPhiNeg);
                    JRNeg = vMove+gamma*((1-perr)*cRNeg + (perr/6)*cRPos + (perr/6)*cStay...
                        + (perr/6)*cThetaPos + (perr/6)*cThetaNeg...
                        + (perr/6)*cPhiPos + (perr/6)*cPhiNeg);
                    JThetaPos = vMove+gamma*((1-perr)*cThetaPos + (perr/6)*cRPos + (perr/6)*cRNeg...
                        + (perr/6)*cStay + (perr/6)*cThetaNeg...
                        + (perr/6)*cPhiPos + (perr/6)*cPhiNeg);
                    JThetaNeg = vMove+gamma*((1-perr)*cThetaNeg + (perr/6)*cRPos + (perr/6)*cRNeg...
                        + (perr/6)*cThetaPos + (perr/6)*cStay...
                        + (perr/6)*cPhiPos + (perr/6)*cPhiNeg);
                    JPhiPos = vMove+gamma*((1-perr)*cPhiPos + (perr/6)*cRPos + (perr/6)*cRNeg...
                        + (perr/6)*cThetaPos + (perr/6)*cThetaNeg...
                        + (perr/6)*cStay + (perr/6)*cPhiNeg);
                    JPhiNeg = vMove+gamma*((1-perr)*cPhiNeg + (perr/6)*cRPos + (perr/6)*cRNeg...
                        + (perr/6)*cThetaPos + (perr/6)*cThetaNeg...
                        + (perr/6)*cPhiPos + (perr/6)*cStay);
                    

                    % Finally compute the new expected cost-to-go, by taking
                    % the maximum over possible actions. Note when I say
                    % "cost-to-go" here, I really mean "payoff-to-go". Also
                    % note that actions are denoted 1=north, 2=south, 3=east,
                    % and 4=west.
                    [J(i,j,k),U(i,j,k)] = max([JStay JRPos JRNeg...
                        JThetaPos JThetaNeg JPhiPos JPhiNeg]);

                    
                end
            end
        end
    end
    
    J(:,:,1);
    U(:,:,1)
    
end
            
Ucur = U;
% INITIAL STATE
i = 10;
j = 1;
k = 5;

arrow=1;

while(Ucur(i,j,k) ~= 0)
    
    U(i,j,k)
    i
    j
    k
    
    % SETUP THE FIGURE
    figure(1);
    clf;
    axis equal;
    axis([-rdim(2) rdim(2) -rdim(2) rdim(2)]);
    box on;
    hold on

    % DRAW HORIZONTAL AND VERTICAL GRID LINES
    for x=-(nr-1):nr-1
        h=line(x*[1,1],[-rdim(2),rdim(2)]);
        set(h,'color',0.8*ones(1,3));
    end
    for y=-(nr-1):nr-1
        h=line([-rdim(2),rdim(2)],y*[1,1]);
        set(h,'color',0.8*ones(1,3));
    end

    % DRAW ROBOT (always at the center)
    plot(0,0,'o', 'MarkerSize',10,'MarkerFaceColor','k')
    
    % DRAW GOAL
    plot(rVec(i),0,'o', 'MarkerSize',10,'MarkerFaceColor','b');
    % DRAW DIRECTION OF ROBOT
    plot([0,arrow*cosd(thetaVec(j))],[0,arrow*sind(thetaVec(j))],'-',...
        'LineWidth',2);
    % DRAW DIRECTION OF GOAL
    plot([rVec(i),rVec(i)+arrow*cosd(phiVec(k))],[0,arrow*sind(phiVec(k))],'-',...
        'LineWidth',2);
    pause(1)
        
    if(Ucur(i,j,k) == 2) 
        i=i+1; 
    elseif(Ucur(i,j,k) == 3)
        i=i-1;
    elseif(Ucur(i,j,k) == 4)
        j=j+1;
        if(j>ntheta)
            j=1;
        end
    elseif(Ucur(i,j,k) == 5)
        j=j-1;
        if(j<1)
            j=ntheta;
        end
    elseif(Ucur(i,j,k) == 6)
        k=k+1;
        if(k>phi)
            k=1;
        end
    elseif(Ucur(i,j,k) == 7)
        k=k-1;
        if(k<1)
            j=nphi;
        end
    end
    
    U(i,j,k)
    i
    j
    k
           
end

U(i,j,k)
i
j
k

% SETUP THE FIGURE
figure(1);
clf;
axis equal;
axis([-rdim(2) rdim(2) -rdim(2) rdim(2)]);
box on;
hold on

% DRAW HORIZONTAL AND VERTICAL GRID LINES
for x=-(nr-1):nr-1
    h=line(x*[1,1],[-rdim(2),rdim(2)]);
    set(h,'color',0.8*ones(1,3));
end
for y=-(nr-1):nr-1
    h=line([-rdim(2),rdim(2)],y*[1,1]);
    set(h,'color',0.8*ones(1,3));
end

% DRAW ROBOT'S PLACE (0,0)
plot(0,0,'o', 'MarkerSize',10,'MarkerFaceColor','k')

plot(rVec(i),0,'o', 'MarkerSize',10,'MarkerFaceColor','b')
plot([0,arrow*cosd(thetaVec(j))],[0,arrow*sind(thetaVec(j))],'-',...
    'LineWidth',2);
plot([rVec(i),rVec(i)+arrow*cosd(phiVec(k))],[0,arrow*sind(phiVec(k))],'-',...
    'LineWidth',2);
