classdef classCARND_SIM < WebSocketServer
% I have never worked with object-oriented programming before, all of my work
% is functional programming. I don't know wtf a "class" is, I am a vector & matrix 
% kind of man myself. I apologize if this is a crappy implementation.
% T.D.K 2017-12-13 Sao Paulo, Brasil
    
% obj - object from the classCARND_SIM class
%       obj.from_sim -> values from the simulator JSON message
%                       including x,y,s,d,yaw,speed,previous_path and end_path
%       obj.to_sim -> What is sent TO the simulator after this function, which
%                       should be 2 30x1 vectors of x,y points for path
%       obj.ref_vel -> Speed of the car (mph?)
%       obj.lane -> Current lane the car is marked in
%       obj.map_waypoints -> all of the information from the history.csv with
%                               track information
    
    properties
        from_sim;
        to_sim;
        lane;
        ref_vel;
        map_waypoints_x;
        map_waypoints_y;
        map_waypoints_s;
        map_waypoints_dx;
        map_waypoints_dy;
        following; 
        change_left;
        change_right;
        too_close;
    end
    
    methods
        function obj = classCARND_SIM(varargin)
            %Constructor
            obj@WebSocketServer(varargin{:});
            
            % Reading in the map waypoints from the.cvs (in local directory)
            A = dlmread('data/highway_map.csv');
            obj.map_waypoints_x = A(:,1);
            obj.map_waypoints_y = A(:,2);
            obj.map_waypoints_s = A(:,3);
            obj.map_waypoints_dx = A(:,4);
            obj.map_waypoints_dy = A(:,5);
            
            obj.lane = 2; % Start in middle lane
            obj.ref_vel = 0; % Reference speed (mph?)
            
            % Info on the car we are following
            obj.following.id = [];
            obj.following.dist = [];
            obj.following.speed = [];
            
            % Are left and right lane changes safe?
            obj.change_left = true;
            obj.change_right = true;
            obj.too_close = false;
            
        end
    end
    
    methods (Access = protected)
        function onOpen(obj,conn,message)
            disp('Connected to Udacity Simulator.');            
        end
        
        function onTextMessage(obj,conn,message)
            
            % Checking to see if the message is valid with telemetry
            if length(message) > 3 && strcmp('42',message(1:2))
                
                % Decoding the json data and adding it to our car class "obj"
                telem = jsondecode(message(3:end));
                obj.from_sim = telem{2,1};

                % If the telemetry is not null, we do our computations to update the path
                if ~isempty(telem{2,1})
                    
                    obj = fcnPATH_PLANNING(obj);
                    
                    % Preparing the data in JSON and sending it back to the sim
                    json_string = ['42',jsonencode([{'control'},obj.to_sim])];
                    conn.send(json_string);
                   
                % If the telemetry is null for some reason, we refeed the previous points
                else
                    json_string = ['42',jsonencode([{'control'},obj.to_sim])];
                    conn.send(json_string);                    
                end 
            end
        end
        
        function onBinaryMessage(obj,conn,message)
            
        end
        
        function onError(obj,conn,message)
            disp('Error - Disconnected from Udacity Simulator.');
        end
        
        function onClose(obj,conn,message)
            disp('Connection closed with Udacity Simulator.');
        end
    end
end