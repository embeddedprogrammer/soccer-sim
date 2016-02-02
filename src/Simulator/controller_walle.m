% 
% Wall-E
% Simulator Competition
% Jacob White, Jordan Hofhiens, Chris Knell, Greg Stewart

function v_c=controller_walle(uu,P)
    persistent prevUU
    persistent debugArgs
    if isempty(prevUU)
        prevUU = zeros(size(uu));
        debugArgs = zeros([length(uu), 2]);
    end
    diffUU = uu - prevUU;
    if length(uu) == 1
        uu = debugArgs(:, 1);
        diffUU = debugArgs(:, 2);
        size(uu)
        if length(uu) == 1
            disp('Error: Stored debug arguments lost when file was changed. Run again.');
            return;
        end
    else
        debugArgs = [uu, diffUU];
    end
    
    % process inputs to function
    % robots - own team
    robot = zeros(3, P.num_robots);
    for i=1:P.num_robots,
        robot(:,i)   = uu(1+3*(i-1):3+3*(i-1));
    end
    NN = 3*P.num_robots;
    % robots - opponent
    opponent = zeros(3, P.num_robots);
    for i=1:P.num_robots,
        opponent(:,i) = uu(1+3*(i-1)+NN:3+3*(i-1)+NN);
    end
    NN = NN + 3*P.num_robots;
    % ball
    ball = [uu(1+NN); uu(2+NN); diffUU(1+NN)/P.control_sample_rate; diffUU(2+NN)/P.control_sample_rate];
    NN = NN + 2;
    % score: own team is score(1), opponent is score(2)
    score = [uu(1+NN); uu(2+NN)];
    NN = NN + 2;
    % current time
    t      = uu(1+NN);
    
    v_c = strategy(robot, opponent, ball, P);
    
    % Keep last state for calculations
    prevUU = uu;
end

% main control code, without velocity calculations.
function v_c=strategy(robot, opponent, ball, P)
    halfLength = P.field_length/2;
    v1 = offense(robot(:,1), [opponent robot(:,2)], ball, P.goal, P, -halfLength*.3, halfLength*1.1);
    %v2 = offense(robot(:,2), [opponent robot(:,1)], ball, pos, P, 2, -halfLength*1.1, 0);
 
    v2 = goalie(robot(:,2), [opponent robot(:,1)], ball, -P.field_length/2+P.robot_radius, P);

    % output velocity commands to robots
    v1 = saturateVelocity(v1,P);
    v2 = saturateVelocity(v2,P);
    v_c = [v1; v2];
end

function v = offense(robot, allOtherRobots, ball, pos, P, minX, maxX)
    n = utility_unit_vector(pos-ball(1:2));
    angleToBall = getAngle(robot(1:2), ball(1:2));
    if minDistToRobotsWithAngle(robot(1:2), angleToBall, allOtherRobots, P) < P.robot_radius * 1.1
        %collision
        if dist(robot(1:2), ball(1:2)) < P.robot_radius
            closestRobot = findClosestRobot(robot(1:2), angleToBall, allOtherRobots, P);
            v = runAtAngle(getAngle(robot(1:2), ball(1:2)), P);
            rotateAngle = pi/2*sign(robot(2) - closestRobot(2));
        else
            v = skillFetchBall(robot, allOtherRobots, ball, P);
            %slow ball -> opportunity to kick. (this should be here)
            
            rotateAngle = getBounceAngle(ball(3:4), v(1:2), getAngle1(-ball(3:4)), getAngle(ball(1:2), pos));
            ballAngle = getAngle(robot, ball(1:2));
            maxAngle = pi/2-getAngle1([P.robot_radius; P.ball_radius]);
            if abs(angleMod(ballAngle - rotateAngle)) > maxAngle
                rotateAngle = ballAngle + maxAngle*sign(rotateAngle - ballAngle);
            end            
        end
    else
        if dist(robot(1:2), ball(1:2)-P.ball_radius*n) < P.robot_radius * 4 && ...
            findDistanceToLine(robot(1:2), ball(1:2), n) < P.robot_radius * .8
            v = runAtAngle(angleToBall, P);
            rotateAngle = getBounceAngle(ball(3:4), v(1:2), getAngle1(-ball(3:4)), getAngle(ball(1:2), pos));
        else %fetch
            v = skillFetchBall(robot, allOtherRobots, ball, P);
            rotateAngle = getBounceAngle(ball(3:4), v(1:2), getAngle1(-ball(3:4)), getAngle(ball(1:2), pos));
        end
        ballAngle = getAngle(robot, ball(1:2));
        maxAngle = pi/2-getAngle1([P.robot_radius; P.ball_radius]);
        if abs(angleMod(ballAngle - rotateAngle)) > maxAngle
            rotateAngle = ballAngle + maxAngle*sign(rotateAngle - ballAngle);
        end
    end
    if(robot(1) < minX && v(1) < 0)
        v(1) = 0;
    end
    if(robot(1) > maxX && v(1) > 0)
        v(1) = 0;
    end
    v(3) = rotateToAngle(robot, rotateAngle, P);
    %persistent labelHandle;
    %labelHandle = drawLabel(robot(1:2),labelHandle, action);
end

function v=goalie(robot, allOtherRobots, ball, x_pos, P)
    if(utility_dist(ball(1:2), -P.goal) < P.goal_width) %More aggressive
        if(utility_dist(robot(1:2), ball(1:2)) < P.robot_radius/2 && minDistToRobots(robot(1:2), allOtherRobots, P) > P.robot_radius*1.5)
            v = runAtAngle(getAngle(robot(1:2), ball(1:2)), P);
        else
            v = skillFetchBallGoalie(robot, allOtherRobots, ball, P);
        end
            
        if abs(ball(3)) > abs(ball(4))
            desiredAngle = pi/4*sign(ball(4));
        else
            desiredAngle = 0;
        end
    else
        % control x position to stay on line
        x_desired = x_pos;
        y_desired = utility_find_intersection(x_desired, ball);

        % Stay within goal posts!
        y_desired = min(y_desired, P.goal(2) + P.goal_width / 2);
        y_desired = max(y_desired, P.goal(2) - P.goal_width / 2);
        v = skillGoToPoint(robot, [x_desired; y_desired], P);
        
        desiredAngle = getAngle(robot(1:2), ball(1:2));
    end
    
    ballAngle = getAngle(robot(1:2), ball(1:2));
    maxAngle = pi/2-getAngle1([P.robot_radius; P.ball_radius]);
    if abs(angleMod(ballAngle - desiredAngle)) > maxAngle
        desiredAngle = ballAngle + maxAngle*sign(desiredAngle - ballAngle);
    end        
    v(3) = rotateToAngle(robot, desiredAngle, P);
end
function futureBall = predictBall(ball, t)
    futureBall = ball(1:2) + ball(3:4) * t;
end
function v_c = skillFetchBall(robot, allOtherRobots, ball, P, i)
    tx = timeToFetch1dim(robot(1), ball(1), ball(3), P.robot_max_vx, 0, P);
    ty = timeToFetch1dim(robot(2), ball(2), ball(4), P.robot_max_vy, 0, P);
    [ballBounce, tBounce] = predictBallBounce(allOtherRobots, ball, P);
    timeToFetch = max(tx, ty);
    tThreshold = P.robot_radius / norm(ball(3:4));
    if timeToFetch > tBounce - tThreshold
        action = 'b1';
        %fast ball -> predict bounce.
        ball = ballBounce;
        tx = timeToFetch1dim(robot(1), ball(1), ball(3), P.robot_max_vx, tBounce, P);
        ty = timeToFetch1dim(robot(2), ball(2), ball(4), P.robot_max_vy, tBounce, P);
        [ballBounce, tBounce] = predictBallBounce(allOtherRobots, ball, P);
        timeToFetch = max(tx, ty);
    end
    position = predictBall(ball, timeToFetch);
    
    %If ball is behind robot, go around it, not through it!
    persistent needToGoBehind;
    if isempty(needToGoBehind), needToGoBehind = 0; end
    ballSpacing = P.robot_radius + P.ball_radius*2;
    if position(1) < (robot(1) - P.robot_radius/2) && abs(robot(2)-position(2)) < ballSpacing
        needToGoBehind = true;
    elseif robot(1) < position(1) - P.ball_radius*2
        needToGoBehind = false;
    end
    if needToGoBehind
        position(1) = position(1) - P.ball_radius*4;
        position(2) = position(2) + ballSpacing * sign(robot(2) - position(2));
        if dist(robot(1:2), ball(1:2)) < ballSpacing
            v_c = runAtAngle(getAngle(ball(1:2), robot(1:2)), P);
        else
            v_c = skillGoToPoint(robot, position, P);
        end
    else
        position(1) = position(1) - P.ball_radius;
        v_c = skillGoToPoint(robot, position, P);
    end
end
function v_c = skillFetchBallGoalie(robot, allOtherRobots, ball, P)
    tx = timeToFetch1dim(robot(1), ball(1), ball(3), P.robot_max_vx, 0, P);
    ty = timeToFetch1dim(robot(2), ball(2), ball(4), P.robot_max_vy, 0, P);
    [ballBounce, tBounce] = predictBallBounce(allOtherRobots, ball, P);
    timeToFetch = max(tx, ty);
    tThreshold = P.robot_radius / norm(ball(3:4));
    if timeToFetch > tBounce - tThreshold
        ball = ballBounce;
        tx = timeToFetch1dim(robot(1), ball(1), ball(3), P.robot_max_vx, tBounce, P);
        ty = timeToFetch1dim(robot(2), ball(2), ball(4), P.robot_max_vy, tBounce, P);
        timeToFetch = max(tx, ty);
    end
    position = predictBall(ball, timeToFetch);
    v_c = skillGoToPoint(robot, position, P);
end

function minDist = minDistToRobots(p1, allOtherRobots, P)
    minDist = 100;
    for i = 1:size(allOtherRobots, 2)
        d = dist(p1, allOtherRobots(1:2, i)) - P.robot_radius;
        if d < minDist
            minDist = d;
        end
    end
    if P.field_width - abs(p1(2)) < minDist
        minDist = P.field_width - abs(p1(2));
    end
    if P.field_length - abs(p1(1)) < minDist
        minDist = P.field_length - abs(p1(1));
    end
end
function minDist = minDistToRobotsWithAngle(p1, angle, allOtherRobots, P)
    minDist = 100;
    for i = 1:size(allOtherRobots, 2)
        robotLocation = allOtherRobots(1:2, i);
        d = dist(p1, robotLocation) - P.robot_radius;
        a = angleMod(angle - getAngle(p1, robotLocation));
        if d < minDist && abs(a) < pi/2-.4 %.4 radians is 17 degrees
            minDist = d;
        end
    end
    if P.field_width - abs(p1(2)) < minDist
        minDist = P.field_width - abs(p1(2));
    end
    if P.field_length - abs(p1(1)) < minDist
        minDist = P.field_length - abs(p1(1));
    end
end
function robot = findClosestRobot(p1, angle, allOtherRobots, P)
    minDist = 100;
    for i = 1:size(allOtherRobots, 2)
        robotLocation = allOtherRobots(1:2, i);
        d = dist(p1, robotLocation) - P.robot_radius;
        a = angleMod(angle - getAngle(p1, robotLocation));
        if d < minDist && abs(a) < pi/2-.4 %.4 radians is 17 degrees
            minDist = d;
            robot = allOtherRobots(1:3, i);
        end
    end
end

%Note - all params in this function are 1-dimensional
function t = timeToFetch1dim(robot, ball, ballVel, max_v, startTime, P)
    % if ball too fast for us to get?
    if (robot < ball && ballVel >= max_v) || (robot > ball && ballVel <= -max_v)
        t = 100; %100 means it's going too fast.
    else
        k = (robot<ball)*2-1;
        t = max(0, (ball - robot - k*max_v*startTime - P.robot_radius/2) / (max_v - ballVel*k)*k);
	end
end
function handle = drawLabel(p1,handle,string)
    if isempty(handle) || ~ishghandle(handle) %~isvalid(handle),
        handle = text(p1(1), p1(2),string);
    else
        set(handle,'position', p1);
        set(handle,'string',string);
        drawnow
    end
end

function [newBall, timeTillBounce] = predictBallBounce(robots, ball, P)
    result = findIntersectionOfBallPathWithRobots(robots, ball, P);
    if length(result) == 1
        result = findIntersectionOfBallPathWithBox(ball(1:2), ball(3:4), P.field_length/2 - P.ball_radius, P.field_width/2 - P.ball_radius);
    end
    if length(result) == 1
        newBall = ball; % This is likely to happen if ball goes out of bounds on us...
        timeTillBounce = 0;
        return;
    end
    oldAngle = getAngle1(-ball(3:4));
    newAngle = angleMod(2*result(3) - oldAngle);
    timeTillBounce = dist(ball(1:2), result(1:2)) / norm(ball(3:4));
    newBall = [result(1:2); unitVectorAtAngle(newAngle)*norm(ball(3:4))];
end
function pointAndAngle = findIntersectionOfBallPathWithBox(p1, v1, halfLength, halfWidth)
    threshold = .001;
    if abs(p1(1)) > halfLength
        p1(1) = sign(p1(1)) * halfLength;
    end
    if abs(p1(2)) > halfWidth
        p1(2) = sign(p1(2)) * halfWidth;
    end
    result = findIntersectionWithNormalLine(-halfLength, p1(1:2), v1(1:2)); %Left
    if abs(result) <= halfWidth && abs(result-p1(2)) > threshold
        pointAndAngle = [-halfLength; result; 0];
        return;
    end
    result = findIntersectionWithNormalLine(halfLength, p1(1:2), v1(1:2)); %Right
    if abs(result) <= halfWidth && abs(result-p1(2)) > threshold
        pointAndAngle = [halfLength; result; pi];
        return;
    end
    result = findIntersectionWithNormalLine(-halfWidth, p1(2:-1:1), v1(2:-1:1)); %Bottom
    if abs(result) <= halfLength && abs(result-p1(1)) > threshold
        pointAndAngle = [result; -halfWidth; pi/2];
        return;
    end
    result = findIntersectionWithNormalLine(halfWidth, p1(2:-1:1), v1(2:-1:1)); %Top
    if abs(result) <= halfLength && abs(result-p1(1)) > threshold
        pointAndAngle = [result; halfWidth; -pi/2];
        return;
    end
    pointAndAngle = 0;
end
function pointAndAngle = findIntersectionOfBallPathWithRobots(rob, ball, P)
    threshold = .001;
    minDist = 100;
    minResult = 0;
    for i = 1:size(rob, 2)
        result = findIntersectionOfBallPathWithRobot(rob(:, i), ball, P);
        if length(result) > 1
            d = dist(ball(1:2), result(1:2));
            if d < minDist && d > threshold;
                minDist = d;
                minResult = result;
            end
        end
    end
    pointAndAngle = minResult;
end
function pointAndAngle = findIntersectionOfBallPathWithRobot(rob, ball, P)
    distToRobot = findDistanceToLine(rob(1:2), ball(1:2), ball(3:4));
    radius = P.robot_radius + P.ball_radius;
    if distToRobot > radius
        pointAndAngle = 0;
        return;
    end
    p1 = findIntersectionWithCircle(ball(1:2), ball(3:4), rob(1:2), radius);
    if abs(angleMod(getAngle(rob(1:2), p1) - rob(3))) > pi/2-getAngle1([radius; P.ball_radius]) ...
            && dot(p1 - ball(1:2), ball(3:4)) > 0
        pointAndAngle = [p1; getAngle(rob(1:2), p1)];
        return;
    end
    paddleCenter = rob(1:2) + unitVectorAtAngle(rob(3))*P.ball_radius;
    paddleVector = unitVectorAtAngle(rob(3) + pi/2);
    paddleWidth = sqrt(radius^2 - P.ball_radius^2);
    if getAngle1(ball(1:2)) == getAngle1(paddleVector)
        pointAndAngle = 0;
        return;
    end
    p1 = findIntersectionWithLine(ball(1:2), ball(3:4), paddleCenter, paddleVector);
    if dist(paddleCenter, p1) < paddleWidth && dot(p1 - ball(1:2), ball(3:4)) > 0
        pointAndAngle = [p1; rob(3)];
        return;
    end
    pointAndAngle = 0;
end
function p1 = findIntersectionWithCircle(p1, v1, p2, r2) %, closestPoint, distToClosestPoint)
    closestPoint = findClosestPointOnLine(p2, p1, v1);
    distToClosestPoint = dist(p2, closestPoint);
    parallelDistToEdgeOfCircle = sqrt(r2^2 - distToClosestPoint^2);
    parallelVectorToEdgeOfCircle = utility_unit_vector(v1)*parallelDistToEdgeOfCircle;
    p1 = closestPoint - parallelVectorToEdgeOfCircle;
end
function p12 = findIntersectionWithLine(p1, v1, p2, v2)
    closestPointOnLine2 = findClosestPointOnLine(p1, p2, v2);
    vectorToClosestPoint = closestPointOnLine2 - p1;
    thetaBetweenClosestPointAndV1 = acos(dot(v1, vectorToClosestPoint)/norm(v1)/norm(vectorToClosestPoint));
    distFromP1ToIntersection = norm(vectorToClosestPoint) / cos(thetaBetweenClosestPointAndV1);
    p12 = p1 + utility_unit_vector(v1)*distFromP1ToIntersection;
end
function yPosition = findIntersectionWithNormalLine(xPosition, p1, v)
    if(v(1) == 0 || v(1)*(xPosition - p1(1)) < 0)
        yPosition = 100;
        return;
    end
    lineSlope = v(2) / v(1);
    lineYIntercept = p1(2) - lineSlope * p1(1);
    yPosition = lineSlope * xPosition + lineYIntercept;
end


% Goes at maximum velocity towards point but always in a straight line.
function v = skillGoToPoint(robot, point, P)
    v = [maximizeVelocity(point - robot(1:2), P); 0];
end

%*************************** UTILITY FUNCTIONS ***************************
function theta = getAngle(p1, p2)
    theta = atan2(p2(2)-p1(2), p2(1)-p1(1));
end
function theta = getAngle1(d)
    theta = atan2(d(2), d(1));
end
function d = dist(p1, p2)
    d = norm(p1 - p2);
end

function d = utility_dist(p1, p2)
    d = norm(p1 - p2);
end

function y = utility_find_intersection(x_pos, ball)
    % control y position to match intersection of ball line and x-pos
    if(ball(3) ~= 0)
        line_slope = ball(4) / ball(3);
        line_y_intercept = ball(2) - line_slope * ball(1);
        y = line_slope * x_pos + line_y_intercept;
    else
        % this doesn't really work if dividing by zero, so we have to revert to
        % a simpler algorithm.
        y =  ball(2);
    end
end

% ball before - angle the ball is coming from
% Find angle required to bounce ball to desired angle.
function theta_required = utility_bounce_angle(theta_ball_before, theta_ball_after)
    theta_required = (theta_ball_after + theta_ball_before) / 2;
    if(abs(angleMod(theta_required - theta_ball_after)) > pi/2)
        theta_required = theta_required + pi;
    end
end

% ball before - angle the ball is coming from
% Find angle required to bounce ball to desired angle.
function theta_required = getBounceAngle(ballVelocity, runVelocity, theta_ball_before, theta_ball_after)
    compensation = getAngle1(norm(ballVelocity)*unitVectorAtAngle(theta_ball_after) + runVelocity) - theta_ball_after;
    theta_ball_after = theta_ball_after - compensation;
    theta_required = (theta_ball_after + theta_ball_before) / 2;
    if(abs(angleMod(theta_required - theta_ball_after)) > pi/2)
        theta_required = theta_required + pi;
    end
end
function v = unitVectorAtAngle(angle)
    v = [cos(angle); sin(angle)];
end

% Mods an angle so that it always is between -pi and +pi.
function diff = angleMod(angle)
    diff = mod(angle + pi, 2*pi) - pi;
end

% ball before - angle the ball is coming from
% Find angle required to bounce ball to desired angle.
function closest_point = findClosestPointOnLine(nearby_point, line_point, line_velocity)
    vec_v = nearby_point - line_point;
    closest_point = line_point + utility_proj(line_velocity, vec_v);
end
% ball before - angle the ball is coming from
% Find angle required to bounce ball to desired angle.
function d = findDistanceToLine(nearbyPoint, linePoint, lineVelocity)
    vec_v = nearbyPoint - linePoint;
    parallelComponent = dot(lineVelocity, vec_v) / norm(lineVelocity);
    d = sqrt(norm(vec_v)^2 - parallelComponent^2);
end

% Returns the normal vector in the direction of u
function u = utility_unit_vector(u)
u = u/norm(u);
end

% Returns the projection of v onto u.
function proj = utility_proj(u, v)
proj = u/(norm(u)^2)*dot(u, v);
end
function v = runAtAngle(angle, P)
    v = maximizeVelocity([unitVectorAtAngle(angle); 0], P);
end
function v = maximizeVelocity(v, P)
    if v(1) == 0 && v(2) == 0
        return;
    end
    if abs(v(1)) / P.robot_max_vx > abs(v(2)) / P.robot_max_vy
        v = v / abs(v(1)) * P.robot_max_vx;
    else
        v = v / abs(v(2)) * P.robot_max_vy;
    end
end
function v = saturateVelocity(v,P)
    if abs(v(1)) > P.robot_max_vx
        v(1:2) = v(1:2) / abs(v(1)) * P.robot_max_vx;
    end
    if abs(v(2)) > P.robot_max_vy
        v(1:2) = v(1:2) / abs(v(2)) * P.robot_max_vy;
    end
    if abs(v(3)) > P.robot_max_omega
        v(3) = v(3) / abs(v(3)) * P.robot_max_omega;
    end
end
function omega = rotateToAngle(robot, desiredAngle, P)
    omega = P.robot_max_omega * sign(angleMod(desiredAngle - robot(3)));
end

