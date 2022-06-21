% plot(out.STATES(:,2), out.STATES(:,3), 'r');
% hold on
% plot(out.STATES(:,7), out.STATES(:,8), 'g');
% axis equal;

t = out.STATES(:,13); % t de lie shu
n = length(out.CONTROLS(:,1));

%% plot error state
%  n = length(out.CONTROLS(:,1));
%  ex = out.STATES(:,13);
%  ey = out.STATES(:,14);
%  
%  subplot(1,2,1);
%  plot(t,ex,'r');
%  ylabel('error state of position x');
%  xlabel('Time(s)');
%  hold on;
%  legend('ex');
% 
%  subplot(1,2,2);
%  plot(t,ey,'b');
%  ylabel('error state of position y');
%  xlabel('Time(s)');
%  hold on;
%  legend('ey');

% plot control input
% n = length(out.CONTROLS(:,1));
% u_a_all = zeros(n);
% u_delta_all = zeros(n);
% u_a_all = out.CONTROLS(:,2);
% u_delta_all = out.CONTROLS(:,3);
% 
% figure(1);
% 
% 
% subplot(1,2,1);
% plot(t,u_a_all,'r');
% legend('u a');
%  ylabel('control input u a');
%  xlabel('Time(s)');
% 
% subplot(1,2,2);
% plot(t,u_delta_all,'b');
% hold on;
% legend('u delta');
%  ylabel('control input u delta');
%  xlabel('Time(s)');
% 
% % calculate cost function
% J = 0;
% for i = 1:n
%     J = J+u_a_all(i)*0.5*u_a_all(i)+u_delta_all(i)*0.5*u_delta_all(i)
% end

%% plot px py video
% figure(2);
axis ([-15,0,-4,8]);
axis equal;
px0 = -13;
py0 = 0;
plot(px0,py0,'x');
par = [0:0.1:2*pi+0.1];
hold on;

stellar_x = -5+.45*cos(par);
stellar_y = -.5+.45*sin(par);
plot(stellar_x,stellar_y,'k');

for i = 1:1:size(t,1)
    t_cur = t(i,1);
    obx1 = -5+cos(0.5*t_cur*pi+pi/3)+.1*cos(par);
    oby1 = -0.5+sin(0.5*t_cur*pi+pi/3)+.1*sin(par);
    h = plot(obx1,oby1,'b');
    hold on;
    obx2 = -5+8*cos(0.02*t_cur*pi+2*pi/3)+0.8*cos(par);
    oby2 = -0.5+8*sin(0.02*t_cur*pi+2*pi/3)+0.8*sin(par);
    l = plot(obx2,oby2,'g');
    obx3 = -5+5*cos(0.04*t_cur*pi+pi/2)+0.8*cos(par);
    oby3 = -0.5+5*sin(0.04*t_cur*pi+pi/2)+0.8*sin(par);
    m = plot(obx3,oby3,'r');
    
    j = plot(out.STATES(i,2),out.STATES(i,3),'r*');
    pause(15/150);
    if i ~=  size(t,1)
        set(h,'Visible','off');
        set(j,'Visible','off');
        set(l,'Visible','off');
        set(m,'Visible','off');
    end
end