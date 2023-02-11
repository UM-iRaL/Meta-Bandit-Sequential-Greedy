function laserScan = laser_sim_nx(x,MAP,r_min,r_max,fov,dfov)
%  laserScan = laser_sim_nx(x,MAP,r_min,r_max,fov,dfov)
%
% @input:
%   x = 3 x 1 = robot pose
%   MAP = occupancy grid representation of the environment
%   r_min = min range
%   r_max = max range
%   fov = field of view [rad]
%   dfov = radian step for laser beams
%
% @return:
%  laserScan.minRange
%  laserScan.maxRange
%  laserScan.angles
%  laserScan.ranges
%  laserScan.valid
%  laserScan.P_earth = points in global coord frame
%
%  laserScan.frontierCoords
%  laserScan.frontierCells
%  laserScan.frontierLinIdx
%
% @author:
%   Nikolay Atanasov
%   Grasp Lab, UPenn
%

persistent P_laser R minRange maxRange angles

if nargin < 6
    dfov = 0.01;
end

if nargin < 5
    fov = deg2rad(94);
end

if nargin < 4
    r_max = 10;
end

if nargin < 3
    r_min = 0;
end

if isempty(P_laser)
    minRange = r_min;
    maxRange = r_max;
    angles = -(fov/2):dfov:(fov/2);
    laserPts = minRange:MAP.res:maxRange;
        
    [R,A] = ndgrid(laserPts,angles); %num_pts x num_angles
    
    P_laser = [reshape(R.*cos(A),1,[]);reshape(R.*sin(A),1,[])];
end

laserScan.minRange = minRange;
laserScan.maxRange = maxRange;
laserScan.angles = angles;

x = x(:);
P_earth = bsxfun(@plus,rot2d(x(3))*P_laser,x(1:2)); % 2 x num_pts

invalid = (P_earth(1,:) < MAP.min(1))|(P_earth(1,:) > MAP.max(1))|...
    (P_earth(2,:) < MAP.min(2))|(P_earth(2,:) > MAP.max(2));


P_earth(1:2,invalid) = 0;

P_earth_cells = meters2cells(P_earth(1:2,:),MAP.min,MAP.res);

linidx = sub2ind_nx(MAP.size.',P_earth_cells(1,:),P_earth_cells(2,:));
isobst = MAP.map(linidx);

good = (~isobst)&(~invalid);
Ranges = R;
Ranges(good) = maxRange;
laserScan.ranges = min(Ranges,[],1);
laserScan.valid = (laserScan.ranges >= laserScan.minRange)&(laserScan.ranges < laserScan.maxRange);

laserScan.P_earth = P_earth;
tmp = reshape(P_earth,2,size(R,1),[]);
tmp = tmp(:,:,~laserScan.valid);
laserScan.frontierCoords = squeeze(tmp(:,end,:));

tmp = reshape(P_earth_cells,2,size(R,1),[]);
tmp = tmp(:,:,~laserScan.valid);
laserScan.frontierCells = squeeze(tmp(:,end,:));

tmp = reshape(linidx,size(R));
laserScan.frontierLinIdx = tmp(end,~laserScan.valid);
end
