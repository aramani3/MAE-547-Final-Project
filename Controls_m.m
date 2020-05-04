%matlab script test for Motion Control
clc;clear all;close all;

%sample Robotic Serial Link setup
Links{1}=Revolute('a',1,'alpha',0,'d',0,'Jm',0,'G',111,'I',eye(3),'m',1,'r',[-0.5,0,0]);
Links{2}=Revolute('a',1,'alpha',0,'d',0,'Jm',0,'G',111,'I',eye(3),'m',1,'r',[-0.5,0,0]);
for k=1:2
    x(k)=Links{k};
end
Robot_R=SerialLink(x);
% time=1;%sample run time
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %run when inverse dynamics button pushed:
% dt=2e-2;
% T=floor(time/dt);
% n=Robot_R.n;
% init_angle_vals=(pi/180)*str2num('[30 0]');
% init_derivative_vals=(pi/180)*str2num('[0 0]');
% xp=str2num('[3 2 1]');xp=xp';%sample position
% xv=str2num('[2 1 0]');xv=xv';%sample velocity
% xa=str2num('[1 0 0]');xa=xa';%sample acceleration
% q(1,:)=init_angle_vals;qd(1,:)=init_derivative_vals;
% Kd=10*diag(ones(1,1));Kp=10*diag(ones(1,1));
% for i=1:T
%     J=Robot_R.jacob0(q(i,:));
% 
%     Jd=Robot_R.jacob_dot(q(i,:),qd(i,:));
% 
%     xe=transl(Robot_R.fkine(q(i,:)));xek(i,:)=xe';
%     J_qd=J*qd(i,:)';
%     xt=xp-xe;xtd=xv-J_qd(1:3);
%     s=Kp*xt+Kd*xtd+xa-Jd(1:3);
%     Jinv=J\eye(max(size(J)));%calculate inverse (pseudo-inverse automatically calcuated if rank-deficient).
%     y=Jinv(:,1:3)*s;
% 
%     Bq=Robot_R.inertia(q(i,:));
%     n_q_qd=Robot_R.coriolis(q(i,:),qd(i,:))*qd(i,:)'+Robot_R.gravjac(q(i,:))';
%     u=Bq*y+n_q_qd;
% 
%     qdd=inv(Bq)*(u-n_q_qd);
%     q(i+1,:)=q(i,:)+qd(i,:)*dt;
%     qd(i+1,:)=qd(i,:)+qdd'*dt;
% end
% t_plot1=0:dt:(time-dt);t_plot2=0:dt:(time);
% figure; hold on;
% plot(t_plot1, xek(:,1));plot(t_plot1, xek(:,2));plot(t_plot1, xek(:,3));
% title('X, Y, and Z-axis plot vs. Time - Inverse');legend('X','Y','Z');
% hold off;
% figure; hold on;
% for j=1:Robot_R.n
%     plot(t_plot2, q(:,j));
% end
% title('Q vs. Time - Inverse');hold off;
% figure; hold on;
% for j=1:Robot_R.n
%     plot(t_plot2, qd(:,j));
% end
% title('Qd vs. Time - Inverse');hold off;
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %run when PD/gravity compensation is pushed:
% dt=2e-2;
% T=floor(time/dt);
% n=Robot_R.n;
% init_angle_vals=(pi/180)*str2num('[30 0]');
% init_derivative_vals=(pi/180)*str2num('[0 0]');
% xp=str2num('[3 2 1]');xp=xp';%sample position
% xv=str2num('[2 1 0]');xv=xv';%sample velocity
% xa=str2num('[1 0 0]');xa=xa';%sample acceleration
% q(1,:)=init_angle_vals;qd(1,:)=init_derivative_vals;
% Kd=10*diag(ones(1,1));Kp=10*diag(ones(1,1));
% for i=1:T
%     J=Robot_R.jacob0(q(i,:));
%     xe=transl(Robot_R.fkine(q(i,:)));xek(i,:)=xe';
%     xt=xp-xe;
%     J_qd=J*qd(i,:)';
%     s=Kp*xt-Kd*J_qd(1:3);
% 
%     J=J(1:3,:);
%     u=J'*s+Robot_R.gravjac(q(i,:))';
% 
%     Bq=Robot_R.inertia(q(i,:));
%     n_q_qd=Robot_R.coriolis(q(i,:),qd(i,:))*qd(i,:)'+Robot_R.gravjac(q(i,:))';
% 
%     qdd=inv(Bq)*(u-n_q_qd);
%     q(i+1,:)=q(i,:)+qd(i,:)*dt;
%     qd(i+1,:)=qd(i,:)+qdd'*dt;
% end
% t_plot1=0:dt:(time-dt);t_plot2=0:dt:(time);
% figure; hold on;
% plot(t_plot1, xek(:,1));plot(t_plot1, xek(:,2));plot(t_plot1, xek(:,3));
% title('X, Y, and Z-axis plot vs. Time - PD/Gravity');legend('X','Y','Z');
% hold off;
% figure; hold on;
% for j=1:Robot_R.n
%     plot(t_plot2, q(:,j));
% end
% title('Q vs. Time - PD/Gravity');hold off;
% figure; hold on;
% for j=1:Robot_R.n
%     plot(t_plot2, qd(:,j));
% end
% title('Qd vs. Time - PD/Gravity');hold off;