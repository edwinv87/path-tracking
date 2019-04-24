function [curvature, yaw_err, lateral_err] = GetLineParameters(line_pixel, KK, Rc, Tc)
% Function to do whatever the 
% Inputs:
%
% Outputs:
%
%
Plotting = 0; % Dont allow plotting at this stage

s_pixel = size(line_pixel);
pc = [line_pixel; ones(1, s_pixel(2))];
cam = [1 1 640 640 1;1 480 480 1 1; 1 1 1 1 1];
s_cam = size(cam);
s_pc = size(pc);
pv = zeros(s_pc(1), s_pc(2));
pv_cam = zeros(s_cam(1), s_cam(2));

H = KK*[Rc(:, 1), Rc(:, 2), Tc];

for n = 1:s_pc(2)
    pv(:, n) = H\pc(:, n); 
    pv(:, n) = (1/pv(3, n))*pv(:, n);
end
 
for n = 1:s_cam(2)
    pv_cam(:, n) = H\cam(:, n);
    pv_cam(:, n) = (1/pv_cam(3, n))*pv_cam(:, n);
end

pts_u = pv(1, :)/1000;
pts_v = pv(2, :)/1000;

if (Plotting == 1)
    plot(pv(1, :), pv(2, :), '.r');
    hold on;
    plot(pv_cam(1, :), pv_cam(2, :), '-g', 'LineWidth', 2);
    axis([-200 1000 -600 600]);
    grid on;
end

p = polyfit(pts_u, pts_v, 3);

if (Plotting == 1) 
    new_u = -1000:1:1000;
    new_v = polyval(p, new_u);
    hold on;
    plot(new_u, new_v);
end

curvature = abs((p(2))*2);        % Curvature (1/m) 
lateral_err = -(p(4)); % Lateral Error
yaw_err = -atan(p(3));          % Yaw Angle Error
end % End of function