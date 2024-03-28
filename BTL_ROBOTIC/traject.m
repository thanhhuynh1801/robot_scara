% 
% 
currentX = 800;
currentY = 0;
currentZ = 10;
targetx=0;
targety=-800;
targetz=20;
yaw_0= 0;
yaw_1= 1;
a1 = 450; a2= 400;
desiredVector = [targetx - currentX, targety - currentY, targetz - currentZ];
normDesiredVector = norm(desiredVector);

qMax = normDesiredVector;  
formula = @(stdVec) stdVec * desiredVector + [currentX, currentY, currentZ];

aMax = 100;
vMax = sqrt(qMax*aMax);

t1      = vMax/aMax;
tm      = (qMax - aMax*t1^2)/vMax;
tmax    = 2*t1 + tm;
t2      = tmax - t1;
disp(tm);
disp(t1);
t       = 0:0.05:tmax;
lengthT = length(t);
a       = zeros(lengthT,1);
v       = zeros(lengthT,1);
q       = zeros(lengthT,1);
x_pre=currentX;
y_pre=currentY;
z_pre=currentZ;
yaw_pre=yaw_0;
X=[];
Y=[];
Z=[];
for i = 1:1:lengthT
    if (t(i) < t1)
        a(i) = aMax;
        v(i) = aMax*t(i);
        q(i) = 0.5*aMax*t(i)^2;
    elseif (t(i) < t2)
        a(i) = 0;
        v(i) = vMax;
        q(i) = 0.5*aMax*t1^2 + vMax*(t(i)-t1);
    else
        a(i) = -aMax;
        v(i) = vMax - aMax*(t(i)-t2);
        q(i) = qMax - 0.5*aMax*(tmax-t(i))^2;
    end

    qStd = q/qMax;
    desiredPos = formula(qStd(i));
    yaw = yaw_0+(q(i)/qMax)*(yaw_1-yaw_0);
    X=[X, desiredPos(1)];
    Y=[Y, desiredPos(2)];
    Z=[Z, desiredPos(3)];
    [Theta_1, Theta_2, d3, Theta_4] = Inverse(desiredPos(1), desiredPos(2),desiredPos(3), yaw);


    v_end=[((desiredPos(1)-x_pre)/0.05);
           ((desiredPos(2)-y_pre)/0.05);
           ((desiredPos(3)-z_pre)/0.05);
           ((yaw-yaw_pre)/0.05)];
    Jacobian_Matrix=[   -a2*sin(Theta_1+Theta_2)-a1*sin(Theta_1)    -a2*sin(Theta_1+Theta_2)   0   0;
                         a2*cos(Theta_1+Theta_2)+a1*cos(Theta_1)     a2*cos(Theta_1+Theta_2)   0   0;
                         0                                           0                         1   0;
                         1                                           1                         0   1];
    v_joint=(Jacobian_Matrix)\v_end;
    v_th1(i)=v_joint(1,1);
    v_th2(i)=v_joint(2,1);
    v_d3(i) =v_joint(3,1);
    v_th4(i)=v_joint(4,1);
    x_pre=desiredPos(1);
    y_pre=desiredPos(2);
    z_pre=desiredPos(3);
    yaw_pre=yaw;


end

figure;
scatter3(X, Y, Z, 'r', 'filled');
xlabel('X');
ylabel('Y');
zlabel('Z');

% Đặt tên cho biểu đồ
title('Các điểm trong không gian 3D');

% Hiển thị biểu đồ
grid on;
axis equal;
view(3);

pause(0.1);
figure(1)
subplot(4,1,1)
hold on
grid on
plot(t,q,'LineWidth',2);
legend('q(t)');
pause(0.1);
subplot(4,1,2)
hold on
grid on
plot(t,v,'LineWidth',2);
legend('v(t)');
pause(0.1);
subplot(4,1,3)
hold on
grid on
plot(t,a,'LineWidth',2);
legend('a(t)');

% x_0 = 800;
% y_0 = 0;
% z_0 = 0;
% yaw_0= 0;
% yaw_1= 1;
% 
% x_1 = 600;
% y_1 = -600;
% z_1 = 0;
% 
% x_2 = -600;
% y_2 = -600;
% z_2 = 0;
% 
% 
% AC = [x_2-x_0;y_2-y_0;z_2-z_0];
% AB = [x_1-x_0;y_1-y_0;z_1-z_0];
% n = cross(AC,AB);
% d = n'*[x_0;y_0;z_0];
% % Tam duong tron
% M = [n';...
%     2*(x_1-x_0) 2*(y_1-y_0) 2*(z_1-z_0);...
%     2*(x_2-x_0) 2*(y_2-y_0) 2*(z_2-z_0)];
% N = [d;x_1^2+y_1^2+z_1^2-x_0^2-y_0^2-z_0^2;x_2^2+y_2^2+z_2^2-x_0^2-y_0^2-z_0^2];
% O = M\N;
% R= sqrt((x_1-O(1))^2+(y_1-O(2))^2 );
% 
% %%%%%%%%%%%%
% alpha_0=acos((2*R*R-(x_0-O(1)-R)^2-(y_0-O(2))^2)/(2*R*R));
% alpha_1=acos((2*R*R-(x_1-O(1)-R)^2-(y_1-O(2))^2)/(2*R*R));
% alpha_2=acos((2*R*R-(x_2-O(1)-R)^2-(y_2-O(2))^2)/(2*R*R));
% if (y_0<O(2))
%     alpha_0=2*pi-alpha_0;
% end
% if (y_1<O(2))
%     alpha_1=2*pi-alpha_1;
% end
% if (y_2<O(2))
%     alpha_2=2*pi-alpha_2;
% end
% alpha= alpha_2-alpha_0;
% if (alpha>0)
%     dau=-1;
%     detaapha=2*pi-alpha;
% else
%     dau=1;
%     detaapha=alpha;
% end
% 
% disp(alpha_0*180/pi);
% disp(alpha_1*180/pi);
% disp(alpha_2*180/pi);
% disp(alpha*180/pi);
% disp(detaapha*180/pi);
% 
% 
% % 
% % if y_1 > y_0
% %     dau=1;
% % else 
% %     dau=-1;
% % end
% % if alpha>alpha_1
% %     alpha_1=alpha;
% % end
% % if (y_0 < O(2))&&(y_2<O(2))
% %     alpha_0 = -alpha_0;
% %     alpha_1 = -alpha_1;
% % end
% % if (y_0 < O(2))&&(y_2>= O(2))
% %     alpha_0 = -alpha_0;
% %     alpha_1 =alpha_1-2*pi;
% % end
% % if (y_0 >= O(2))&&(y_2< O(2))
% %     alpha_1 = -alpha_1;
% % end
% % if (y_0 >= O(2))&&(y_2>= O(2))
% %     alpha_1 =alpha_1-2*pi;
% % end
% % disp(O);
% 
% % dentaalpha = abs(alpha_0-alpha_1);
% % disp(dentaalpha*180/pi);
% % 
% % 
% X=[];
% Y=[];
% Z=[];
% 
% 
% % qMax    = R*abs(dentaalpha);
% 
% aMax = 100;
% vMax = sqrt(qMax*aMax);
% 
% t1      = vMax/aMax;
% tm      = (qMax - aMax*t1^2)/vMax;
% tmax    = 2*t1 + tm;
% t2      = tmax - t1;
% 
% t       = 0:0.2:tmax;
% lengthT = length(t);
% a       = zeros(lengthT,1);
% v       = zeros(lengthT,1);
% q       = zeros(lengthT,1);
% X=[];
% Y=[];
% Z=[];
% for i = 1:1:lengthT
%     if (t(i) < t1)
%         a(i) = aMax;
%         v(i) = aMax*t(i);
%         q(i) = 0.5*aMax*t(i)^2;
%     elseif (t(i) < t2)
%         a(i) = 0;
%         v(i) = vMax;
%         q(i) = 0.5*aMax*t1^2 + vMax*(t(i)-t1);
%     else
%         a(i) = -aMax;
%         v(i) = vMax - aMax*(t(i)-t2);
%         q(i) = qMax - 0.5*aMax*(tmax-t(i))^2;
%     end
% 
%     x = O(1)+R*cos(alpha_0+(q(i)/qMax)*detaapha*dau);
%     y = O(2)+R*sin(alpha_0+(q(i)/qMax)*detaapha*dau);
% 
%     z = -x*n(1)/n(3)-y*n(2)/n(3)+d/n(3)- (q(i)/qMax)*(abs(z_0-z_1));
%  
%     
%     yaw = yaw_0+(q(i)/qMax)*(yaw_1-yaw_0);
% 
%     X=[X, x];
%     Y=[Y, y];
%     Z=[Z, z];
% 
% end
% 
% 
% figure;
% scatter3(X, Y, Z, 'r', 'filled');
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% 
% % Đặt tên cho biểu đồ
% title('Các điểm trong không gian 3D');
% 
% % Hiển thị biểu đồ
% grid on;
% axis equal;
% view(3);
% 
% pause(0.1);
% figure(1)
% subplot(4,1,1)
% hold on
% grid on
% plot(t,q,'LineWidth',2);
% legend('q(t)');
% pause(0.1);
% subplot(4,1,2)
% hold on
% grid on
% plot(t,v,'LineWidth',2);
% legend('v(t)');
% pause(0.1);
% subplot(4,1,3)
% hold on
% grid on
% plot(t,a,'LineWidth',2);
% legend('a(t)');
