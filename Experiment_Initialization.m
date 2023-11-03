clc
clear 
close all






% load("trace1_data.mat");
load("trace2_data.mat");
iter = length(camera_theta);
hov=pi/3;  %horizontal fieled of view in radians
vov=50/180*pi;  %vertical field of view in radians
focall = 0.2; 
focalh = 6.5; 
Rad = 10;
focalm = [(focall+focalh)/2 ;0 ; 0 ; 1];
innerR = (focalh-focall)/2;


color_uncovered_landmark_edge = [102, 148, 196];
color_uncovered_landmark_face = [186, 217, 249];
color_covered_landmark_edge = [150, 200, 90];
color_covered_landmark_face = [182, 222, 133];
color_estimated_covered_landmark_edge = [0, 0, 0];
color_estimated_covered_landmark_face = [255, 0, 0];
color_obstacle = [190, 76, 236];
color_covered_obstacle = [255, 255, 0];


for i= 1:length(landmark)
    y(:,i) = [landmark(1,i) landmark(2,i) landmark(3,i) 1 landmark_id(i)].';
end;
y(:,length(landmark)+1) = [-30 -10 -10 1 668].';
y(:,length(landmark)+2) = [-30 -20 -20 1 190].';
y(:,length(landmark)+3) = [-35 -20 -10 1 502].';
y(:,length(landmark)+4) = [-31 -10 -20 1 519].';
y(:,length(landmark)+5) = [-35 -11 -30 1 638].';
y(:,length(landmark)+6) = [-37 -15 -10 1 378].';
y(:,length(landmark)+7) = [-32 -20 -10 1 396].';




for i = 1:iter
    SE3_t(:,:,i) = [rotation(camera_theta(i),0,0) [camera_x(i) camera_y(i) camera_z(i)].'; 0 0 0 1];
%     U_real(:,:,i) = [hatoperate([-U_vtheta{i},0,0].') [U_vz{i} 0 0].'; 0 0 0 0];
    U_real(:,:,i) = [zeros(3,3) [U_vz{i} 0 0].'; 0 0 0 0];
end;    
SE3_s(:,:,1) = [rotation(camera_theta(1),0,0) [3 1 0.25].'; 0 0 0 1];
%  SE3_s(:,:,1) = [rotation(camera_theta(1),0,0) [camera_x(1) camera_y(2) camera_z(3)].'; 0 0 0 1];