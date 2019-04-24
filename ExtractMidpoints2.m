function [Points] = ExtractMidpoints2(Image)
% Function to Extract the Midpoints of a line in a binary image.
% Input: Image - matrix of the binary image with a line
% Output: Points - matrix of the extracted points
%
%

s = size(Image);
width = s(2);
height = s(1);
HPoints = zeros(1, height);
WPoints = zeros(1, width);
st = [];
p = 1; % Inner increment variable
for m = height:-1:1
    xa = [];
    xb = [];
    for n = 1:1:width
        if(Image(m, n) == 1)
            xa = n;
            break;
        end
    end
   
    for n = width:-1:1
        if(Image(m, n) == 1)
            xb = n;
            break;
        end
    end
    
    if ~(isempty(xa) || isempty(xb))
        if ~(xa == 1 || xb == width)
            if(~((xb - xa) > 80))
                if(~((xb - xa) < 40))
                    st = m;
                    break;
                end
            end
        end
    end
end

% Entry Point xa and xb
midpoint = xa + round((xb - xa)/2);
% disp(midpoint)
if (~isempty(st))
    for m = st:-10:1
        ya = [];
        yb = [];

        for n = midpoint:1:width
            if (Image(m,n) == 0)
                yb = n;
                break;
            end
        end

        for n = midpoint:-1:1
            if (Image(m,n) == 0)
               ya = n;
               break;
            end
        end

        if ~(isempty(ya) || isempty(yb))
            if ~(ya == 1 || yb == width)
                if(~((yb - ya) > 80))
                    if(~((yb - ya) < 20))
                        midpoint = ya + round((yb - ya)/2);
                        % disp(midpoint);
                        HPoints(p) = m;
                        WPoints(p) = midpoint;
                        p = p + 1;
                    end
                end
            end
        end
    end
end
Iw = WPoints > 0;
Ih = HPoints > 0;
Points = [WPoints(Iw); HPoints(Ih)];
%image(Image);
%hold on;
%plot(WPoints, HPoints, '.r');
end