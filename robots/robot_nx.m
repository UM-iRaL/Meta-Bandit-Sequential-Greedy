classdef robot_nx < handle
    properties (Constant = true)
        % Vehicle
        %stdev_v = 0.1;     % motion noise
        %stdev_w = deg2rad(5);
        
        stdev_v = 0;     % no motion noise
        stdev_w = 0;
        % Detection model
        p0 = 0.90*ones(8,1);
        v0 = 20.5*ones(8,1);
        m0 = 3*ones(8,1);
        
        % Bearing Model
        b_sigma = deg2rad(1);
        % To stop bearing noise let b_sigma = 0;
        
        % Range Model
        r_sigma = 1;
        % To stop range noise let r_sigma = 0;        
        
        
        % DATA ASSOCIATION:
        da_known = true;
        % MISSED DETECTIONS
        miss_det = false;
        % CLUTTER
        lambda = 0;     % To enable clutter lambda > 0
        
        
        % Class Model
        % Confusion matrix ( hypothesized x true )
        num_class = 8;%2;
        num_score = 11;%28;
        
        meters2pix = 0.003;     % meters per pixel
        cx = 360; cy = 180;     % center of image in pixels
        
        Cmat = eye(8);
        % To stop class confusion let Cmat = eye(5);
        
    end

    properties (SetAccess = protected, GetAccess = protected)
        x = [0;0;0];    % robot pose

    end
    properties (SetAccess = protected, GetAccess = public)
        params;     
        % Sensor
        fov = deg2rad(94); %deg2rad(94);     % deg2rad(124);
        r_sense = 45;          % meter
        T = 1;            % sampling period
      
    end
    
    methods
        function obj = robot_nx(x0, r_sense, fov, dT)
            if(nargin < 1)
                x0 = [0;0;0];
            end
            obj.set_x(x0(:));
            
            % params
            obj.params.T = obj.T;
            obj.params.stdev_v = obj.stdev_v;
            obj.params.stdev_w = obj.stdev_w;
            if (nargin < 3)
                obj.params.r_sense = obj.r_sense;
                obj.params.fov = obj.fov;
            else
                obj.params.r_sense = r_sense;
                obj.params.fov = fov;
                obj.r_sense = r_sense;
                obj.fov = fov;
                obj.T = dT;
                obj.params.T = dT;
            end
            obj.params.b_sigma = obj.b_sigma;
            obj.params.r_sigma = obj.r_sigma;
            
            obj.params.p0 = obj.p0;
            obj.params.v0 = obj.v0;
            obj.params.m0 = obj.m0;
            obj.params.fpd = @(d,c) obj.p0(c) .* exp(-abs(obj.m0(c)-d)./obj.v0(c));
            
            obj.params.da_known = obj.da_known;
            obj.params.miss_det = obj.miss_det;
            obj.params.clutter_rate = obj.lambda;
            
            
            obj.params.num_class = obj.num_class;
            obj.params.num_score = obj.num_score;
            obj.params.Cmat = obj.Cmat;
            
        end
        
        function set_x(obj,x0)
            if(nargin < 2)
                x0 = [0;0;0];
            end
            obj.x = x0(:);
        end
        
        function x = get_x(obj)
            x = obj.x;
        end
        
        function move(obj,u,smm)            
            if( nargin < 3)
                %obj.x = dd_motion_model(obj.x,u,obj.T);
                obj.x = point_mass_motion_model(obj.x, u, obj.T);
            else
                obj.x = smm(obj.x,u,obj.params);
            end
        end
        
        function z = sense(obj,map,som)
            % map = num_obj x 3 = (x,y,class)
            % som = @generate_msrmnt
            if( nargin < 3)
                z = generate_msrmnt(map,obj.x.',obj.params);
            else
                z = som(map,obj.x.',obj.params);
            end
        end
        
    end
end