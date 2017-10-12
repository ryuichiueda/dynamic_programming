
% STATES VARIABLES
xRobot = [0:10];
yRobot = [0:10];
theta = [0,90,180,270];
xGoal = length(xRobot)-1;
yGoal = length(yRobot)-1;
r = zeros(1,length(xRobot)*length(yRobot));
for ix=1:length(xRobot)
    for iy=1:length(yRobot)
        r((ix-1)*length(yRobot)+iy) = sqrt((xGoal-xRobot(ix))^2+(yGoal-yRobot(iy))^2);
    end
end
phi = [0,90,180,270];

length(xRobot)*length(yRobot)*length(theta)*length(xGoal)*length(yGoal)*length(r)*length(phi)
J = zeros(length(xRobot),length(yRobot),length(theta),length(r),length(phi));
U = J;

% FINAL STATES
vGoal = 100;
for ixRobot=1:length(xRobot)
    for iyRobot=1:length(yRobot)
        for itheta=1:length(theta)
            for ir=1:length(r)
                for iphi=1:length(phi)
                    % if robot reaches the goal
                    if (r(ir) == 0)
                        J(ixRobot,iyRobot,itheta,ir,iphi) = vGoal;
                        U(ixRobot,iyRobot,itheta,ir,iphi) = 0;
                    else
                        U(ixRobot,iyRobot,itheta,ir,iphi) = ceil(rand*3);
                    end
                end
            end
        end
    end
end

% VALUE ITERATION
% - probability of going the wrong way
perr = 0.1;
uchange(1)=0;
T = 100;
for t=1:T
    % Iterate over all states.
    Jprev = J;
    Uprev = U;
    uchange(t)=0;
    for ixRobot=1:length(xRobot)
        for iyRobot=1:length(yRobot)
            for itheta=1:length(theta)
                for ir=1:length(r)
                    for iphi=1:length(phi)
                        % Update the value/policy for every state that is not a goal.
                        if (r(ir) ~= 0)
                            % Set 3 actions.
                            if (iyRobot ~= length(yRobot)) && (itheta ~= length(theta))
                                stay = Jprev(ixRobot,iyRobot,itheta,ir,iphi);
                                fwrd = Jprev(ixRobot,iyRobot+1,itheta,ir,iphi);
                                left = Jprev(ixRobot,iyRobot,itheta+1,ir,iphi);
                            elseif (iyRobot == length(yRobot)) && (itheta ~= length(theta))
                                stay = Jprev(ixRobot,iyRobot,itheta,ir,iphi);
                                fwrd = Jprev(ixRobot,iyRobot,itheta,ir,iphi);
                                left = Jprev(ixRobot,iyRobot,itheta+1,ir,iphi);
                            elseif (iyRobot ~= length(yRobot)) && (itheta == length(theta))
                                stay = Jprev(ixRobot,iyRobot,itheta,ir,iphi);
                                fwrd = Jprev(ixRobot,iyRobot+1,itheta,ir,iphi);
                                left = Jprev(ixRobot,iyRobot,itheta-3,ir,iphi);
                            else
                                stay = Jprev(ixRobot,iyRobot,itheta,ir,iphi);
                                fwrd = Jprev(ixRobot,iyRobot,itheta,ir,iphi);
                                left = Jprev(ixRobot,iyRobot,itheta-3,ir,iphi);
                            end
                            
                            % Now compute the total expected cost for each
                            % of the three possible actions.
                            Jstay = -1+((1-perr)*stay + (perr/2)*fwrd + (perr/2)*left);
                            Jfwrd = -1+((1-perr)*fwrd + (perr/2)*stay + (perr/2)*left);
                            Jleft = -1+((1-perr)*left + (perr/2)*stay + (perr/2)*fwrd);
                            
                            % Finally compute the new expected cost-to-go, by taking
                            % the maximum over possible actions.
                            [J(ixRobot,iyRobot,itheta,ir,iphi),...
                                U(ixRobot,iyRobot,itheta,ir,iphi)] = max([Jstay Jfwrd Jleft]);
                            
                            % See if the policy changed (for display purposes only).
                            if (U(ixRobot,iyRobot,itheta,ir,iphi)~=...
                                    Uprev(ixRobot,iyRobot,itheta,ir,iphi))
                                uchange(t) = uchange(t)+1
                            end
                        end
                    end
                end
            end
        end
    end
end

