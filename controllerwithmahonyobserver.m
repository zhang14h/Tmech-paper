
%% 

%%%%%% initial setup
clc
clear
% close all
a0 =1;
iter = 2000;

%%
%%%%%%%%%%%%%  Modeling Uncertianty Genertation
for i = 1:iter
%     if rem(i,100) <=40;
       q(i) = 0.00*pi/2500;
%     else
%        q(i) = 0;
%     end;
%    w1_t(i) =0;
%   w2_t(i) = pi/500;
%   w3_t(i) = pi/500;
%   v_delta(:,1,i) = 1.5*[-2*2*pi*cos(i/200) 2*2*pi*sin(i/300+pi/6)  1*2*pi/iter*sin(2*i*pi/iter)].';
  v_delta(:,1,i) = 0*0.00*[1*cos(i/200),1*sin(i/300+pi/6),0.5*cos(i/100)*sin(i/700+pi/7)] ;

  
  delta1_t(i) = 0*1*q(i);
  delta2_t(i) = 0*0.02*pi/2000;
  delta3_t(i) = 0*0.04*pi/1500;
  
  
%   if w1_t(i) == 0 & w2_t(i) == 0 & w3_t(i) ==0;
%      
%      w_t(:,1,i) = [w1_t(i) w2_t(i) w3_t(i)].';
%      w_hat_t(:,:,i) = hatoperate ([w1_t(i) w2_t(i) w3_t(i)].'); %lefs-righs invarians coefficiens,so3;
%      exp_t(:,:,i) = 1;
%   else
%    
%    
%      w_t(:,1,i) = [w1_t(i) w2_t(i) w3_t(i)].'; %body-frame angular velocisy
%      
%      w_hat_t(:,:,i) = [0 -w_t(3,1,i) w_t(2,1,i);w_t(3,1,i) 0 -w_t(1,1,i);-w_t(2,1,i) w_t(1,1,i) 0]; %lefs-righs invarians coefficiens,so3;
%      exp_t(:,:,i) = expm(w_hat_t(:,:,i));
%   end;
  if delta1_t(i) == 0 & delta2_t(i) == 0 & delta3_t(i) ==0;
     
     delta_t(:,1,i) = [delta1_t(i) delta2_t(i) delta3_t(i)].';
     delta_hat_t(:,:,i) = hatoperate ([delta1_t(i) delta2_t(i) delta3_t(i)].'); %lefs-righs invarians coefficiens,so3;
%      exp_deltat(:,:,i) = 1;
  else
   
   
     delta_t(:,1,i) = [delta1_t(i) delta2_t(i) delta3_t(i)].'; %body-frame angular velocisy
     
     delta_hat_t(:,:,i) = [0 -delta_t(3,1,i) delta_t(2,1,i);delta_t(3,1,i) 0 -delta_t(1,1,i);-delta_t(2,1,i) delta_t(1,1,i) 0]; %lefs-righs invarians coefficiens,so3;
%      exp_deltat(:,:,i) = expm(delta_hat_t(:,:,i));
  end;
end;


%%%%%%%%%%%%%  Robot Inital Position
fei_t= 0.0;
theta_t= 0.0;
miu_t= 0.0;
% fei_t= 0;
% theta_t= 0.98*pi;
% miu_t= 0.98*pi;
rpy = [fei_t;
       theta_t;
       miu_t];

R_t(:,:,1) = rotation(fei_t,theta_t,miu_t); %SO3 


px_t = 10;
py_t = 10;
pz_t = 10;
P_t= [px_t;
      py_t;
      pz_t];
   

         
SE3_t(:,:,1) = [R_t(:,:,1) P_t;
                [0,0,0]            1     ];

%%%%%%%%%%%%%  Observer Initial Position

fei_s = 0;
theta_s = 0;
miu_s = 0;

R_s(:,:,1) = rotation(fei_s,theta_s,miu_s); %SO3

R_s = rotation(fei_s,theta_s,miu_s);
px_s = 7;
py_s = 11;
pz_s = 10;
P_s= [px_s;
       py_s;
       pz_s];

v_s(:,1) = [0,0,0].';
   
SE3_s(:,:,1) = [R_s(:,:,1) P_s;
               [0,0,0]      1     ];
         

%%%%%%%%%%%%%  Target Point

R = rotation( 0.0 , 0.0, 0.0 );
P = [0 0 0];
T = [R P.'; [ 0 0 0] 1];

%%%%%%%%%%%%%  Landmark Distribution
y(:,1) = [2;2;2;1]; y(:,2) = [4; 1; 1; 1]; y(:,3) = [1; 2;3;1]; y(:,4) = [3;2;1;1]; y(:,5) = [-1;3;1;1]; y(:,6) = [2;2;3;1]; 
y(:,7) = [2; 1; 2; 1]; y(:,8) = [3;4;5;1] ; y(:,9) = [ 5;3;1;1] ; y(:,10) = [7;8;9;1]; y(:,11) = [2;5;7;1]; y(:,12) = [3;6;8;1];
y(:,13) = [1;5;9;1]; y(:,14) = [3;7;8;1];

% y(:,1) = [2;2;2;1]; y(:,2) = [4;4;4;1]; y(:,3) = [1; 2;3;1]; y(:,4) = [3;2;1;1]; y(:,5) = [-1;3;1;1]; y(:,6) = [2;2;3;1]; 
% y(:,7) = [2; 1; 2; 1]; y(:,8) = [3;4;5;1] ; y(:,9) = [ 5;3;1;1] ; y(:,10) = [7;8;9;1]; y(:,11) = [2;5;7;1]; y(:,12) = [3;6;8;1];
% y(:,13) = [1;5;9;1]; y(:,14) = [3;7;8;1];

% y(:,1) = [1;2;3;1]; y(:,2) = [1.2;2.4;3.6;1]; y(:,3) = [1.3;2.6;3.9;1]; y(:,4) = [2.6;5.2;7.8;1]; y(:,5) = [1.7;3.4;5.1;1]; y(:,6) = [1.9;3.8;5.7;1]; 
% y(:,7) = [2.2;4.4;6.6;1]; y(:,8) = [3;5;4;1] ;  y(:,9) = [ 5;3;1;1] ; y(:,10) = [7;8;9;1]; y(:,11) = [2;5;7;1]; y(:,12) = [3;6;8;1];
% y(:,13) = [1;5;9;1]; y(:,14) = [3;7;8;1];

%  y(:,1) = [1;2;3;1]; y(:,2) = [1.2;2.4;3.6;1]; y(:,3) = [1.3;2.6;3.9;1]; y(:,4) = [2.6;5.2;7.8;1]; y(:,5) = [1.7;3.4;5.1;1]; y(:,6) = [1.9;3.8;5.7;1]; y(:,7) = [2.2;4.4;6.6;1]; 
% y(:,8) = [3;5;4;1] ;  y(:,9) = [3.6;6;4.8;1] ; y(:,10) = [3.9;6.5;5.2;1]; y(:,11) = [7.2;13;10.4;1]; y(:,12) = [5.1;8.5;6.8;1];
% y(:,13) = [5.7;9.5;7.2;1]; y(:,14) = [3;7;8;1];
% 
% y(:,1) = [1;2;3;1]; y(:,2) = [1.2;2.4;3.6;1]; y(:,3) = [1.3;2.6;3.9;1]; y(:,4) = [2.6;5.2;7.8;1]; y(:,5) = [1.7;3.4;5.1;1]; y(:,6) = [3.6;6;4.8;1];  y(:,7) = [3;5;4;1]; 

% y(:,1) = [1;2;3;1]; y(:,2) = [1.2;2.4;3.6;1]; y(:,3) = [1.5;3.5;5;1]; y(:,4) = [3;7;8;1]; y(:,5) = [3;5;4;1];  y(:,6) = [3.6;6;4.8;1] ;y(:,7) = [7.2;13;10.4;1];

% y(:,1) = [1;2;3;1]; y(:,2) = [2;7;1;1]; y(:,3) = [1.5;3.5;5;1]; y(:,4) = [3;7;8;1]; y(:,5) = [3;5;4;1];  y(:,6) = [5;4;1;1] ;y(:,7) = [7.2;13;10.4;1];

% y(:,1) = [1;2;3;1]; y(:,2) = [2;7;1;1]; y(:,3) = [3;7;8;1]; y(:,4) = [3;5;4;1];
% y(:,1) = [1;2;3;1]; y(:,2) = [1.2;2.4;3.6;1]; y(:,3) = [1.3;2.6;3.9;1]; y(:,4) = [2.6;5.2;7.8;1];
% y(:,1) = [1;2;3;1];  y(:,2) = [2;7;1;1]; y(:,3) = [1.3;2.6;3.9;1]; y(:,4) = [2.6;5.2;7.8;1];


%%%%%%%%%%%%%   Obstacle Distribution
y_obstacle(:,1) = [7.1; 7.2; 7.3; 1];
y_obstacle(:,2) = [7; 4.9; 2.6; 1];
y_obstacle(:,3) = [1; 1; 5.0; 1];

%%
%%%%%%%%%%%%%   Dimension and Initial Setup for U,  Ut = Ui + modeling uncertianty 

[rownum,lengthy]=size(y); 
[rowavoid,lengthavoid]=size(y_obstacle); 

Ut(:,:,1) = [ 0 0 0 0; 0 0 0 0 ; 0 0 0 0; 0 0 0 0 ];
Ui(:,:,1) = [ 0 0 0 0; 0 0 0 0 ; 0 0 0 0; 0 0 0 0 ];


%%
for i = 1:iter
%%   
%%%%%%%%%%%%%   Calculating Avoidance Algorithm
    U_avoid(:,:,i) = zeros(4,4);
    for j = 1:lengthavoid
        p_obstacle(:,j) = inv(SE3_t(:,:,i))*y_obstacle(:,j); 
        omega_avoid(:,:,j) =  p_obstacle(:,j)*(p_obstacle(:,j)-[0;0;0;1]).';
        a_avoid = 1*exp(0.1 - 1*norm(p_obstacle(:,j)-[0;0;0;1],'fro'));
%         a_avoid = 0;     
        U_avoid(:,:,i) = U_avoid(:,:,i) + a_avoid*projection(1/4*((omega_avoid(:,:,j) - omega_avoid(:,:,j).'))); 
    end;
%%    
%%%%%%%%%%%%%   Error Calculation
    TRt(:,:,i) = [SE3_t(1,1,i) SE3_t(1,2,i) SE3_t(1,3,i);
                 SE3_t(2,1,i) SE3_t(2,2,i) SE3_t(2,3,i);
                 SE3_t(3,1,i) SE3_t(3,2,i) SE3_t(3,3,i)];
             
    TPt(:,i) = [SE3_t(1,4,i) SE3_t(2,4,i) SE3_t(3,4,i)]; 
   
    TRs(:,:,i) = [SE3_s(1,1,i) SE3_s(1,2,i) SE3_s(1,3,i);
                 SE3_s(2,1,i) SE3_s(2,2,i) SE3_s(2,3,i);
                 SE3_s(3,1,i) SE3_s(3,2,i) SE3_s(3,3,i)];
    TPs(:,i) = [SE3_s(1,4,i) SE3_s(2,4,i) SE3_s(3,4,i)];
    
    
    error4(:,:,i) =  SE3_s(:,:,i)*inv(T);
    error6(:,:,i) = SE3_t(:,:,i)*inv(T);
    error7(:,:,i) =  SE3_t(:,:,i)*inv(SE3_s(:,:,i));
    omega1(:,:,i)=zeros(4,4);
    omega2(:,:,i)=zeros(4,4);
    omega3(:,:,i)=zeros(4,4);
    
    omega(:,:,i) =   1*projection(1/4*((error4(:,:,i) - error4(:,:,i).')));
    omega4(:,:,i) =  0*inv(T)*projection(1/4*((error4(:,:,i) - eye(4))-(error4(:,:,i) - eye(4)).'))*T;

%%   
%%%%%%%%%%%%%   Measurement Noise
    deltay(:,:,i) = zeros(4,14);  
%% 
%%%%%%%%%%%%%   Ui = U_trajecotry + U_avoid
    Ui(:,:,i) = -0.05*inv(T)*(omega(:,:,i))*(T)+U_avoid(:,:,i);
%%%%%%%%%%%%%   Ut = Ui + modling uncertianty      
    Ut(:,:,i) =  Ui(:,:,i)+[delta_hat_t(:,:,i) v_delta(:,1,i);[0,0,0]    0 ];
%%   
%%%%%%%%%%%%%   System Trajectory
    SE3_t(:,:,i+1) = SE3_t(:,:,i)*expm(Ut(:,:,i));
%%   
%%%%%%%%%%%%%   Observer Convergence Trajectory
    error5(:,:,i) =  SE3_t(:,:,i)*inv(SE3_s(:,:,i));
    for p = 1:lengthy
       deltay(:,p,i) = 0*[0.3*cos(i*p/2) 0.2*sin(i*p/5) 0.1*sin(i*p/4) 0].';
         omega2(:,:,i) = omega2(:,:,i)+0.005*(-error5(:,:,i)*y(:,p)-SE3_s(:,:,i)*deltay(:,p,i)+y(:,p))*transpose(y(:,p));
%        omega2(:,:,i) = omega2(:,:,i)+0.01*inv(SE3_s(:,:,i)).'*(inv(SE3_s(:,:,i))*y(:,p)-inv(SE3_t(:,:,i))*y(:,p))*transpose(y(:,p));
    end;
    omega_t(:,:,i) =   projection(1/4*((omega2(:,:,i) - omega2(:,:,i).')));
    se3_s = Ui(:,:,i)-inv(SE3_s(:,:,i))*omega_t(:,:,i)*SE3_s(:,:,i); 
  
    SE3_s(:,:,i+1) = SE3_s(:,:,i)*expm(se3_s);
%%    
%%%%%%%%%%%%%   Two Measures    
%     dv1(i) = real(visiondistance(R,TRs(:,:,i),P.',TPs(:,i)));
%     dv2(i) = real(visiondistance(R,TRt(:,:,i),P.',TPt(:,i)));
%     dv3(i) = real(visiondistance(TRs(:,:,i),TRt(:,:,i),TPs(:,i),TPt(:,i)));
      dv1(i) = norm(error4(:,:,i)-eye(4),'fro');  %T - hatX
      dv2(i) = norm(error6(:,:,i)-eye(4),'fro');  %T - X
      dv3(i) = norm(error7(:,:,i)-eye(4),'fro');  %X - hatX 
%%   
%%%%%%%%%%%%%   Real Time Plot
%     plot3([TPs(1,i),TPs(1,i)+5*TRs(1,1,i)],[TPs(2,i),TPs(2,i)+5*TRs(1,2,i)],[TPs(3,i),TPs(3,i)+5*TRs(1,3,i)],'--r');
%     hold on 
%     xlim([-20 20]);
%     ylim([-20,20]);
%     zlim([-20,20]);
%     plot3([TPs(1,i),TPs(1,i)+5*TRs(2,1,i)],[TPs(2,i),TPs(2,i)+5*TRs(2,2,i)],[TPs(3,i),TPs(3,i)+5*TRs(2,3,i)],'--g');
%     plot3([TPs(1,i),TPs(1,i)+5*TRs(3,1,i)],[TPs(2,i),TPs(2,i)+5*TRs(3,2,i)],[TPs(3,i),TPs(3,i)+5*TRs(3,3,i)],'--b');
%     plot3([TPt(1,i),TPt(1,i)+5*TRt(1,1,i)],[TPt(2,i),TPt(2,i)+5*TRt(1,2,i)],[TPt(3,i),TPt(3,i)+5*TRt(1,3,i)],'color',[10 10 10]/255);
%     plot3([TPt(1,i),TPt(1,i)+5*TRt(2,1,i)],[TPt(2,i),TPt(2,i)+5*TRt(2,2,i)],[TPt(3,i),TPt(3,i)+5*TRt(2,3,i)],'color',[100 100 100]/255);
%     plot3([TPt(1,i),TPt(1,i)+5*TRt(3,1,i)],[TPt(2,i),TPt(2,i)+5*TRt(3,2,i)],[TPt(3,i),TPt(3,i)+5*TRt(3,3,i)],'color',[200 200 200]/255);
%     plot3(y(1,:),y(2,:),y(3,:),'o','Color','k','MarkerSize',5)
%     hold off  
%     pause(0.01)
%%
end;
k1 = 0;
k2 = 0;
k3 = 0;
for i=1:iter
k1 = k1+(dv1(i));
k2 = k2+(dv2(i));
k3 = k3+(dv3(i));
end;
% for i= 1:iter
% 
% trn3(i) = trace(projection(inv(error5(:,:,i))-inv(error5(:,:,i)).')*projection(inv(error5(:,:,i))-inv(error5(:,:,i)).').');
% trn4(i) = trace(inv(error5(:,:,i)));
% trn5(i)= trace(projection((inv(error5(:,:,i))*SE3_s(:,:,i)*inv(T))-(inv(error5(:,:,i))*SE3_s(:,:,i)*inv(T)).')*projection((SE3_s(:,:,i)*inv(T)-(SE3_s(:,:,i)*inv(T)).')).');
% 
% end

%%
%%%%%%%%%%%%%   Trajectory and Obstacle Plot
figure();
for i = 1:iter
    px(i) = SE3_t(1,4,i);
    py(i) = SE3_t(2,4,i);
    pz(i) = SE3_t(3,4,i);
    hpx(i) = SE3_s(1,4,i);
    hpy(i) = SE3_s(2,4,i);
    hpz(i) = SE3_s(3,4,i);
end;    
 plot3(px,py,pz,'b');  
 hold on
 for j = 1:lengthavoid
      plot3(y_obstacle(1,j),y_obstacle(2,j),y_obstacle(3,j),'o','Color','r','MarkerSize',5);
 end;     
 plot3(hpx,hpy,hpz,'-.k');
 
 
%% 
% display (k1);
% figure()
%  
%   plot (dv1);
%   title('observer and proposed controlled output');
%   grid
% figure(1)  
%   p1 = plot (dv2,'-r','linewidth',1); m1 = "Ec with Kc = 0";
% %   title('attitude and position control performance with Kc = 0');
% %  
% %   txt = {'dV = ' ,k2};
% %   text(iter/2,10,txt);
%   grid
%   hold on
%   p2 = plot (dv3,'-.g','linewidth',1); m2 = "Eo with Ko = 0";
%   title('Closed-loop Performance ');
%   xlabel('Time');
%   ylabel('Error');
%   legend([p1,p2],[m1,m2]);
% 
% figure(2)  
%   p2 = plot (dv3,'-.g','linewidth',1);
%   title('observer in [24] with Ko=0');
%   xlabel('Time');
%   ylabel('Estimation error');
% %   txt = {' = ' ,k3};
% %   text(iter/2,3,txt);
%   grid
%   hold on