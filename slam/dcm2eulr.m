function eul_vect = dcm2eulr(DCMbn)
%DCM2EULR       Direction cosine matrix to Euler angle
%               vector conversion.
%       
%	eul_vect = dcm2eulr(DCMbn)
%
%   INPUTS
%       DCMbn = 3x3 direction cosine matrix providing the
%             transformation from the body frame
%             to the navigation frame
%
%   OUTPUTS
%       eul_vect(1) = roll angle in radians 
%
%       eul_vect(2) = pitch angle in radians 
%
%       eul_vect(3) = yaw angle in radians 
%
%   NOTE
%       If the pitch angle is vanishingly close to +/- pi/2,
%       the elements of EUL_VECT will be filled with NaN.

%   REFERENCE:  Titterton, D. and J. Weston, STRAPDOWN
%               INERTIAL NAVIGATION TECHNOLOGY, Peter
%               Peregrinus Ltd. on behalf of the Institution
%               of Electrical Engineers, London, 1997.
%
%	M. & S. Braasch 12-97
%	Copyright (c) 1997 by GPSoft
%	All Rights Reserved.
% EMF changed phi to use atan2 instead of atan on 3/28/2000

  if nargin<1,error('insufficient number of input arguments'),end

  phi = atan2(DCMbn(3,2),DCMbn(3,3));
  theta = asin(-DCMbn(3,1));
  psi = atan2(DCMbn(2,1),DCMbn(1,1));

  eul_vect = [phi theta psi]';
