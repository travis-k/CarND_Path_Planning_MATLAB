function [obj] = fcnPATH_PLANNING(obj)
% fcnPATH_PLANNING takes in the status of the car from the simulator
% and returns the path that the car should take (usually up to 30m ahead).
% Inputs:
% obj - object from the classCARND_SIM class
%       obj.from_sim -> values from the simulator JSON message
%                       including x,y,s,d,yaw,speed,previous_path and end_path
%       obj.to_sim -> What is sent TO the simulator after this function, which
%                       should be 2 30x1 vectors of x,y points for path
%       obj.ref_vel -> Reference speed of the car
%       obj.lane -> Current lane the car is marked in
%       obj.map_waypoints -> all of the information from the history.csv with
%                               track information
%
% T.D.K 2017-12-13 Sao Paulo, Brasil

% Renaming values for simplicity
car_x = obj.from_sim.x;
car_y = obj.from_sim.y;
car_s = obj.from_sim.s;
car_yaw = deg2rad(obj.from_sim.yaw);
car_speed = obj.from_sim.speed;
safe_dist = (10 + 1.4.*(car_speed./2.24));

%% Gathering sensor fusion data
% Format of sensor_fusion matrix:
% car ID, car x, car y, car dx, car dy, car s, car d
sensor_fusion = obj.from_sim.sensor_fusion;
sensor_fusion(:,1) = sensor_fusion(:,1) + 1; % Adding one, MATLAB indices start at 1
num_cars = max(sensor_fusion(:,1));

% Cars in this lane
idx_inlane = (sensor_fusion(:,7) <= 4.*obj.lane) & (sensor_fusion(:,7) >= ((4.*obj.lane) - 4));
% Cars up the road from our car (all lanes)
idx_upstream = sensor_fusion(:,6) >= car_s;
% Cars behind our car (all lanes)
idx_downstream = sensor_fusion(:,6) <= car_s;

% This is a logical matrix, where the rows are the sensor fusion cars 
% and the columns are lanes 1,2,3. A "1" indicates the corresponding car (row)
% is in that lane (column)
idx_lane = false(num_cars,3);
idx_lane(:,1) = 0 < sensor_fusion(:,7) & sensor_fusion(:,7) < 4;
idx_lane(:,2) = 4 < sensor_fusion(:,7) & sensor_fusion(:,7) < 8;
idx_lane(:,3) = 8 < sensor_fusion(:,7) & sensor_fusion(:,7) < 12;

% Finding cars in front of us in our own lane
idx_infront = find(idx_inlane & idx_upstream);
% Identifying which car we are behind
[~, idx] = min(sensor_fusion(idx_infront,6));
idx_follow = idx_infront(idx);

% Default booleans for adjustments to lane or speed
obj.too_close = false;
obj.change_left = true;
obj.change_right = true;

%% Identifying if we are too close to the car in front of us
if ~isempty(idx_follow)
    obj.following.id = idx_follow;
    obj.following.dist = sensor_fusion(idx_follow, 6) - car_s;
    obj.following.speed = norm(sensor_fusion(idx_follow,4:5));
    
    if obj.following.dist <= safe_dist
        % If the car is closer than the safe following distance
        obj.too_close = true;
    end
else
    % In this case, there is no car in front of us
    obj.following.id = [];
    obj.following.dist = [];
    obj.following.speed = [];
end

%% Lane Change
% We only look to lane change when we are stuck behind another car
% This is a crude implementation
if obj.too_close
    % Lane changes are assumed safe by default
    % Below is where we look to see if a car is present in the next lane
    % and adjust our decision based on that
    
    % Look left to see if the left lane has a car
    if obj.lane > 1
        idx_ahead = idx_upstream & idx_lane(:,obj.lane - 1);
        idx_behind = idx_downstream & idx_lane(:,obj.lane - 1);
        
        if min(sensor_fusion(idx_ahead,6)) - car_s < 30
            obj.change_left = false;
        elseif (car_s - max(sensor_fusion(idx_behind,6)) <= 15)
            obj.change_left = false;
        end
        
    end
    
    % Look right to see if that lane has a car
    if obj.lane < 3
        idx_ahead = idx_upstream & idx_lane(:,obj.lane + 1);
        idx_behind = idx_downstream & idx_lane(:,obj.lane + 1);
        
        if min(sensor_fusion(idx_ahead,6)) - car_s < 30
            obj.change_right = false;
        elseif (car_s - max(sensor_fusion(idx_behind,6)) <= 15)
            obj.change_right = false;
        end
        
    end
    
    % Making the lane change decision, defaulting with a lane change
    % to the left if both are safe
    if obj.change_left && obj.lane > 1
        obj.lane = obj.lane - 1;
    elseif obj.change_right && obj.lane < 3
        obj.lane = obj.lane + 1;
    end
    
end


%% Speed adjustments (this method is crude)
if obj.too_close && abs(obj.following.speed - obj.ref_vel) > 0.224
    obj.ref_vel = obj.ref_vel - 0.224;
elseif obj.too_close && abs(obj.following.speed - obj.ref_vel) <= 0.224
    obj.ref_vel = obj.following.speed;
elseif ~obj.too_close && obj.ref_vel < 49.5
    obj.ref_vel = obj.ref_vel + 0.224;
end

%% Generating Path
% The amount of previous path points still in the simulator
prev_len = length(obj.from_sim.previous_path_x);

% "pts" is a 5x2 matrix of 5 (x,y) pairs for the spline
% The first 2 (x,y) points are the start of the spline
pts = nan(5,2);
if prev_len < 2
    % If no previous points are in the simulator, the path starts just behind the car
    ref_x = car_x;
    ref_y = car_y;
    ref_yaw = car_yaw;
    
    pts(1,:) = [car_x - cos(car_yaw), car_y - sin(car_yaw)]; % Previous location
    pts(2,:) = [car_x, car_y]; % Current location
    
else
    % If the simulator has points in it, the path starts just before the end of those points
    ref_x = obj.from_sim.previous_path_x(end);
    ref_y = obj.from_sim.previous_path_y(end);
    
    pts(1:2,1) = obj.from_sim.previous_path_x(end-1:end);
    pts(1:2,2) = obj.from_sim.previous_path_y(end-1:end);
    
    ref_yaw = atan2(pts(2,2) - pts(1,2), pts(2,1) - pts(1,1));
end

% Extend the path up the road a bunch
pts(3,:) = getXY(car_s + 50, ((4*obj.lane) - 2), obj.map_waypoints_s, obj.map_waypoints_x, obj.map_waypoints_y);
pts(4,:) = getXY(car_s + 70, ((4*obj.lane) - 2), obj.map_waypoints_s, obj.map_waypoints_x, obj.map_waypoints_y);
pts(5,:) = getXY(car_s + 90, ((4*obj.lane) - 2), obj.map_waypoints_s, obj.map_waypoints_x, obj.map_waypoints_y);

% Shifting the (x,y) points to the car reference frame (car is (0,0))
shift_x = pts(:,1) - ref_x;
shift_y = pts(:,2) - ref_y;
pts(:,1) = shift_x.*cos(0 - ref_yaw) - shift_y.*sin(0 - ref_yaw);
pts(:,2) = shift_x.*sin(0 - ref_yaw) + shift_y.*cos(0 - ref_yaw);

% Our target location (using the spline to find the "y")
target_x = 30;
target_y = spline(pts(:,1), pts(:,2), target_x);
% And a rough estimation of that distance (should be ~30)
target_dist = distance(0,0, target_x, target_y);

N = target_x./(target_dist./(0.02.*obj.ref_vel./2.24));

% Taking the cumulative sum of the intervals to get the x points in the car reference frame
x_points = cumsum(repmat(N,30 - prev_len,1));
y_points = spline(pts(:,1), pts(:,2), x_points);

% Adjusting these points to the global (x,y)
x_add_on = (x_points.*cos(ref_yaw) - y_points.*sin(ref_yaw)) + ref_x;
y_add_on = (x_points.*sin(ref_yaw) + y_points.*cos(ref_yaw)) + ref_y;

% Appending the new points to the old ones
% For some reason the simulator runs way better if they
% are appended together instead of only sending in
% the new points
next_x = [obj.from_sim.previous_path_x; x_add_on];
next_y = [obj.from_sim.previous_path_y; y_add_on];


obj.to_sim = struct('next_x',next_x,'next_y',next_y);

end

%% HELPER FUNCTIONS
% These are copies of the ones from the Udacity project main.cpp

function closestWaypoint = NextWaypoint(x, y, theta, maps_x, maps_y)

closestWaypoint = dsearchn([x y], [maps_x maps_y]);

map_x = closestWaypoint(:,1);
map_y = closestWaypoint(:,2);

heading = atan2((map_y - y), (map_x - x));

angle = abs(theta - heading);

closestWaypoint(angle > pi/4) = closestWaypoint(angle > pi/4) + 1;
closestWaypoint(closestWaypoint >= length(maps_x)) = closestWaypoint(closestWaypoint >= length(maps_x)).*0;

end

function d = distance(x1, y1, x2, y2)

d = sqrt((x2 - x1).^2 + (y2 - y1).^2);

end

% I NEVER VECTORIZED THIS ONE PROPERLY B/C I'M LAZY
% IT WOULD WORK, BUT FOR 1 "x,y" pair at a time!
% function [frenet_s, frenet_d] = getFrenet(x, y, theta, maps_x, maps_y)
% 
% next_wp = NextWaypoint(x, y, theta, maps_x, maps_y);
% 
% prev_wp = next_wp - 1;
% prev_wp(prev_wp < 1) = length(maps_x) - 1;
% 
% n_x = maps_x(next_wp) - maps_x(prev_wp);
% n_y = maps_y(next_wp) - maps_y(prev_wp);
% 
% x_x = x - maps_x(prev_wp);
% x_y = y - maps_y(prev_wp);
% 
% proj_norm = (x_x.*n_x + x_y.*n_y)./(n_x.*n_x + n_y.*n_y);
% proj_x = proj_norm.*n_x;
% proj_y = proj_norm.*n_y;
% 
% frenet_d = distance(x_x, x_y, proj_x, proj_y);
% 
% center_x = 1000 - maps_x(prev_wp);
% center_y = 2000 - maps_y(prev_wp);
% 
% centerToPos = distance(center_x, center_y, x_x, x_y);
% centerToRef = distance(center_x, center_y, proj_x, proj_y);
% 
% frenet_d(centerToPos <= centerToRef) = frenet_d(centerToPos <= centerToRef).*-1;
% 
% % NOT VECTORIZED PROPERLY
% temp = distance(maps_x(2:prev_wp), maps_y(2:prev_wp), maps_x(1:prev_wp - 1), maps_y(1:prev_wp - 1));
% frenet_s = sum(temp) + distance(0,0,proj_x, proj_y);
% 
% end

function out = getXY(s, d, maps_s, maps_x, maps_y)

if s > maps_s
    s = s - maps_s(end);
end

next_wp = find(s(1) < maps_s, 1, 'first');
prev_wp = next_wp - 1;

prev_wp(prev_wp < 1) = length(maps_x) - 1;

% wp2 = mod(prev_wp + 1, length(maps_x));
wp2 = next_wp;
heading = atan2((maps_y(wp2) - maps_y(prev_wp)), (maps_x(wp2) - maps_x(prev_wp)));
seg_s = s - maps_s(prev_wp);

seg_x = maps_x(prev_wp) + seg_s.*cos(heading);
seg_y = maps_y(prev_wp) + seg_s.*sin(heading);

perp_heading = heading - pi./2;

x = seg_x + d.*cos(perp_heading);
y = seg_y + d.*sin(perp_heading);

out = [x, y];

end






