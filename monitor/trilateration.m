function triPt = trilateration( P1, DistA, P2, DistB, P3, DistC )
%trilateration : retourne les coordonnées d'un point par trilatération sur la distance à 3 points de référence
%   Fournir les coordonnées [degrés] des points de référence,
%   et les distances [km] aux points de référence,
%   pour obtenir les coordonnées sur le globe terrestre modélisé par une sphère authalique.
%
%   Example :
%    format long
%    [lat, lon]  = trilateration(37.418436, -121.963477, 0.265710701754, 37.417243, -121.961889, 0.234592423446, 37.418692, -121.960194, 0.0548954278262)
%    lat =
%     37.419102373825389
%    lon =
%     -1.219605792083924e+02
%
%   Code d'origine =
%   http://gis.stackexchange.com/questions/66/trilateration-using-3-latitude-and-longitude-points-and-3-distances,
%   cmeeren - 28/07/2010
%   Conversion MatLab = L. Tailhardat - 06/04/2016

%% ECEF des points de référence
% using authalic sphere
% if using an ellipsoid this step is slightly different
% Convert geodetic Lat/Long to ECEF xyz
%   1. Convert Lat/Long to radians
%   2. Convert Lat/Long(radians) to ECEF

% P1 = [xA; yA; zA];
% P2 = [xB; yB; zB];
% P3 = [xC; yC; zC];

%% Transformation
% from wikipedia https://en.wikipedia.org/wiki/Trilateration
% transform to get circle 1 at origin
% transform to get circle 2 on x axis
ex = (P2 - P1) / (norm(P2 - P1));
i  = dot(ex, (P3 - P1));
ey = (P3 - P1 - i*ex) / (norm(P3 - P1 - i*ex));
ez = cross(ex, ey);
d  = norm(P2 - P1);
j  = dot(ey, (P3 - P1));

%% Estimation
% from wikipedia https://en.wikipedia.org/wiki/Trilateration
% plug and chug using above values
x = ((DistA^2) - (DistB^2) + (d^2))/(2*d);
y = (((DistA^2) - (DistC^2) + (i^2) + (j^2))/(2*j)) - ((i/j)*x);

% only one case shown here
z = -sqrt((DistA^2) - (x^2) - (y^2));

% triPt is an array with ECEF x,y,z of trilateration point
triPt = P1 + x*ex + y*ey + z*ez;

%% Conversion finale
% convert back to lat/long from ECEF
% convert to degrees
% lat = rad2deg(asin(triPt(3) / earthR));
% lon = rad2deg(atan2(triPt(2),triPt(1)));

end