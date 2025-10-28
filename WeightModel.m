function [weight, inertia] = WeightModel(RN2B)
mass = 20500 * 0.45359237; % lbf to kg
weight = RN2B*[0;0;mass*9.81];
Ixx = 9496 * 1.35581795; % slugft2 to kgm2
Iyy = 55814 * 1.35581795;
Izz = 63100 * 1.35581795;
Ixy = 0 * 1.35581795;
Ixz = 982 * 1.35581795;
Iyz = 0 * 1.35581795;
inertia = [Ixx Ixy Ixz;Ixy Iyy Iyz;Ixz Iyz Izz];
end