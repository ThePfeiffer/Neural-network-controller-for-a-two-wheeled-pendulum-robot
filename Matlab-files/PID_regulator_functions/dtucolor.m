% DTU color RGB vector
% [rgb] = dtucolor(dtuidx)
%      dtuidx is color 1..7 1=red, 2=blue, 3=dark, 4=yellow,
%                           5=green, 6=purpel, 7=orange
function [rgb] = dtucolor(dtuidx)
    %hold on
switch dtuidx
   case 1
      rgb=[153,0,0]/256;
   case 2
      rgb = [47,62,234]/256;
   case 3
      rgb = [3,15,79]/256;
   case 4
      rgb=[246,208,77]/256;
   case 5
      rgb=[0,136,53]/256;
    case 6
      rgb=[121,35,142]/256;
    case 7
      rgb = [252,118,52]/256;
   otherwise
      rgb = [0,0,0];
end
end
