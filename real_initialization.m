clc
clear
close all

load("exp_data_trace1.mat");
iter = 500;

snr = 10;
d = -50;
landmarks = length(landmark);
piu = 0.1;
p_avoid= 0.2;

color_uncovered_landmark_edge = [102, 148, 196];
color_uncovered_landmark_face = [186, 217, 249];
color_covered_landmark_edge = [150, 200, 90];
color_covered_landmark_face = [182, 222, 133];
color_estimated_covered_landmark_edge = [0, 0, 0];
color_estimated_covered_landmark_face = [255, 0, 0];
color_obstacle = [190, 76, 236];
color_covered_obstacle = [255, 255, 0];


for i = 1:iter
 
  w1_t(i) =0;
  w2_t(i) = 0/4000;
  w3_t(i) = 0/5000;
  v_t(:,1,i) = [0.022 0 0].';
  if w1_t(i) == 0 & w2_t(i) == 0 & w3_t(i) ==0;
     
     w_t(:,1,i) = [w1_t(i) w2_t(i) w3_t(i)].';
     w_hat_t(:,:,i) = hatoperate ([w1_t(i) w2_t(i) w3_t(i)].'); %lefs-righs invarians coefficiens,so3;
     
  else
   
   
     w_t(:,1,i) = [w1_t(i) w2_t(i) w3_t(i)].'; %body-frame angular velocisy
     
     w_hat_t(:,:,i) = [0 -w_t(3,1,i) w_t(2,1,i);w_t(3,1,i) 0 -w_t(1,1,i);-w_t(2,1,i) w_t(1,1,i) 0]; %lefs-righs invarians coefficiens,so3;
    
  end;
  
  if i<= 9*iter/10;  
  v_delta(:,1,i) = d*0.001*[1*cos(i/200);0;0]  ;

%   v_delta(:,1,i) = d*0.5;

  
  delta1_t(i) = 0*pi/2500;
  delta2_t(i) = 0*pi/2000;
  delta3_t(i) = pi/7000;
  
  
  
    if delta1_t(i) == 0 & delta2_t(i) == 0 & delta3_t(i) ==0;
     
     delta_t(:,1,i) = [delta1_t(i) delta2_t(i) delta3_t(i)].';
     delta_hat_t(:,:,i) = hatoperate ([delta1_t(i) delta2_t(i) delta3_t(i)].'); %lefs-righs invarians coefficiens,so3;
     exp_deltat(:,:,i) = 1;
    else
     delta_t(:,1,i) = [delta1_t(i) delta2_t(i) delta3_t(i)].'; %body-frame angular velocisy
     
     delta_hat_t(:,:,i) = [0 -delta_t(3,1,i) delta_t(2,1,i);delta_t(3,1,i) 0 -delta_t(1,1,i);-delta_t(2,1,i) delta_t(1,1,i) 0]; %lefs-righs invarians coefficiens,so3;
     exp_deltat(:,:,i) = expm(delta_hat_t(:,:,i));
    end;
  else
    delta_hat_t(:,:,i)= zeros(3,3);
    v_delta(:,1,i) =[0;0;0];
  end;    
end;





for i= 1:length(landmark)
    y(:,i) = [landmark(1,i)/100 landmark(2,i)/100 landmark(3,i)/100 1 landmark_id(i)].';
end;

hov=pi/3;  %horizontal fieled of view in radians
vov=50/180*pi;  %vertical field of view in radians
focall = 0.2; 
focalh = 6.5; 
Rad = 10;
focalm = [(focall+focalh)/4 ;0 ; 0 ; 1];
innerR = (focalh-focall)/4;

P_t = [-1;-0.5;0];

fei_t = 0;
theta_t = 0;   %sensor angle in radians
miu_t = 0.3;

R_t(:,:,1) = rotation(fei_t,theta_t,miu_t);
SE_3t(:,:,1) = [R_t(:,:,1) P_t; [0 0 0] 1];

%observer information
fei_s = 0;
theta_s = 0;
miu_s = 0;

R_s(:,:,1) = rotation(fei_s,theta_s,miu_s); %SO3

R_s = rotation(fei_s,theta_s,miu_s);

P_s = [3;2;0];
% P_s = [0;0;0];
   
SE_3s(:,:,1) = [R_s(:,:,1) P_s;
               [0,0,0]      1     ];
           
       



y_obstacle(:,1) = [-1; 2 ; 0; 1];
y_obstacle(:,2) = [3; 0.5; 0; 1];
y_obstacle(:,3) = [0.5; -0.1; 0; 1];
y_obstacle(:,4) = [1; 2.7; 0; 1];
% y_obstacle(:,5) = [1; 2.5; 0; 1];
% y_obstacle(:,1) = [-1; 0; 0; 1];

[rownum,lengthy]=size(y); 
[rowavoid,lengthavoid]=size(y_obstacle); 
figure('position',[0 0 1000 500]);

% y(:,55) = [-50 -50 -30 1 1000].';
% y(:,56) = [-30 -60 -10 1 1001].';
% y(:,57) = [-20 -70 -90 1 1002].';
% y(:,58) = [-20 -10 -20 1 1003].';
% y(:,59) = [-10 -50 -10 1 1004].';

 for j = 1:landmarks
                    
                    scatter3(y(1,j),y(2,j),y(3,j),60,'filled','Marker','o','MarkerFaceColor','k','MarkerEdgeColor','k','LineWidth',1);
                    hold on;
%                    
end;
set(gca,'fontsize',26) 
zlim([0,2])