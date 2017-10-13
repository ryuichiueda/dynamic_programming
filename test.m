%function [J,U]=DP_valueiteration

% DEFINE GLOBAL PARAMETERS
global resR resThetaPhi rdim thetadim phidim nr ntheta nphi dr dtheta dphi ...
    perr vGoal vObst vMove vInitial
% - minimum grid resolution for XY
resR = 1.0;
resThetaPhi = 90.0;
% - dimensions of the world
rdim = [0.0 10.0];
thetadim = [-180.0 180.0];
phidim = [-180.0 180.0];
% - number of grid cells
nr=ceil((rdim(2)-rdim(1))/resR);
ntheta=ceil((thetadim(2)-thetadim(1))/resThetaPhi);
nphi=ceil((phidim(2)-phidim(1))/resThetaPhi);
% - size of grid cells
dr = (rdim(2)-rdim(1))/nr;
dtheta = (thetadim(2)-thetadim(1))/ntheta;
dphi = (phidim(2)-phidim(1))/nphi;
% - probability of going the wrong way
perr = 0.1;
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
T=10;
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
                    

                    % Now compute the total expected cost for each of the four
                    % possible actions (north, south, east, west).
                    JStay = -1+((1-perr)*cStay + (perr/6)*cRPos + (perr/6)*cRNeg...
                        + (perr/6)*cThetaPos + (perr/6)*cThetaNeg...
                        + (perr/6)*cPhiPos + (perr/6)*cPhiNeg);
                    JRPos = -1+((1-perr)*cRPos + (perr/6)*cStay + (perr/6)*cRNeg...
                        + (perr/6)*cThetaPos + (perr/6)*cThetaNeg...
                        + (perr/6)*cPhiPos + (perr/6)*cPhiNeg);
                    JRNeg = -1+((1-perr)*cRNeg + (perr/6)*cRPos + (perr/6)*cStay...
                        + (perr/6)*cThetaPos + (perr/6)*cThetaNeg...
                        + (perr/6)*cPhiPos + (perr/6)*cPhiNeg);
                    JThetaPos = -1+((1-perr)*cThetaPos + (perr/6)*cRPos + (perr/6)*cRNeg...
                        + (perr/6)*cStay + (perr/6)*cThetaNeg...
                        + (perr/6)*cPhiPos + (perr/6)*cPhiNeg);
                    JThetaNeg = -1+((1-perr)*cThetaNeg + (perr/6)*cRPos + (perr/6)*cRNeg...
                        + (perr/6)*cThetaPos + (perr/6)*cStay...
                        + (perr/6)*cPhiPos + (perr/6)*cPhiNeg);
                    JPhiPos = -1+((1-perr)*cPhiPos + (perr/6)*cRPos + (perr/6)*cRNeg...
                        + (perr/6)*cThetaPos + (perr/6)*cThetaNeg...
                        + (perr/6)*cStay + (perr/6)*cPhiNeg);
                    JPhiNeg = -1+((1-perr)*cPhiNeg + (perr/6)*cRPos + (perr/6)*cRNeg...
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
    
end
            
    









