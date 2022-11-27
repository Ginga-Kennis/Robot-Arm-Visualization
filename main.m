%set initial angles
th1_init = 0;
th2_init = -50;
th3_init = -30;

%Ask for the goal cordinates
prompt_x = "X cordinate -200<X<200：";
x = input(prompt_x);
%Calculate the range of Y using the X input
y_limit = sqrt(200*200-x*x);
prompt_y = "Y cordinate" + -y_limit+ " < Y <" + y_limit + ":";
y = input(prompt_y);
%Calculate the range of Z using the X,Y input
z_up_limit = 50 + sqrt(200*200-x*x-y*y);
z_down_limit = 50 - sqrt(200*200-x*x-y*y);
prompt_z = "Y cordinate" + z_down_limit + "< Z <" + z_up_limit + ":";
z = input(prompt_z);



%Use Inverse Kinematics to calculate the angles of the robot arm
[th_1 th_2 th_3] = inv_kinematics(x,y,z)

%Calculate the trajectory of the robot arm
[th1 th2 th3] = traj3(th1_init,th2_init,th3_init,th_1,th_2,th_3);

%Vizualize
l = length(th1);
frames_per_second = 15;
r = rateControl(frames_per_second)
for i=1:l
    [posh,pos3,pos2,pos1] = kinematics(th1(i),th2(i),th3(i));
    xp = [pos1(1) pos2(1) pos3(1) posh(1)];
    yp = [pos1(2) pos2(2) pos3(2) posh(2)];
    zp = [pos1(3) pos2(3) pos3(3) posh(3)];
    
    
    plot3(xp,yp,zp,"-or","LineWidth",5);
    xlim([-200 200])
    ylim([-200 200])
    zlim([-150 250])
    grid on
    axis square
    waitfor(r);

    
end



%Caluculate the Jacobian
function J = Jac1(th1,th2,th3)
    %Length of each parts
    L1=50;
    L2=80;
    L3=120;
    %同時変換行列計算/Homogeneous Transformation Matrix
    A1=trans(0,0,0)*rot_x(0)*trans(0,0,L1)*rot_z(deg2rad(th1));
    A2=trans(0,0,0)*rot_x(deg2rad(th2))*trans(0,0,L2)*rot_z(0);
    A3=trans(0,0,0)*rot_x(deg2rad(th3))*trans(0,0,L3)*rot_z(0);
    %Calculate cordinates of each joints
    pos_h = A1*A2*A3*[0;0;0;1];
    pos_3 = A1*A2*[0;0;0;1];
    pos_2 = A1*[0;0;0;1];
    pos_1 = [0;0;0;1];
    %同時変換行列から3×3部分のみ取り出す
    A11 = A1(1:3,1:3);
    A22 = A2(1:3,1:3);
    A33 = A3(1:3,1:3);
    %Rotation matrix
    z1 = [0 0 1]';
    z2 = A11*[1 0 0]';
    z3 = A11*A22*[1 0 0]';
    %Caluculate Jacobian
    J = [cross(z1,pos_h(1:3)-pos_1(1:3)) cross(z2,pos_h(1:3)-pos_2(1:3)) cross(z3,pos_h(1:3)-pos_3(1:3))];
end

%Inverse Kinematics
function [th1 th2 th3] = inv_kinematics(goal_x,goal_y,goal_z)
    r_goal = [goal_x;goal_y;goal_z];
    %Set angles to initial 
    th1 = 0;
    th2 = 10;
    th3 = 20;
    
    th = [th1;th2;th3];

    %数値計算を安定させる係数
    delta = 0.2;

    %difference between goal and current position
    dr_abs= 1.5;
    
    while dr_abs > 0.1
        %Caluculate current position
        [pos_h,pos_3,pos_2,pos_1] = kinematics(th(1),th(2),th(3));
        r_now = [pos_h(1) pos_h(2) pos_h(3)]';
        %Caluculate difference
        dr = r_goal - r_now;
        dr_abs = sqrt(dot(dr,dr));
        %Calculate Jacobian
        J = Jac1(th(1),th(2),th(3));
        d_th = delta*inv(J)*dr;
        th = th + d_th;
    end
    if th(1) < 0
        th1 = th(1) + 180;
    elseif th(1) > 0
        th1 = th(1) - 180;
    else
        th1 = th(1)
    end
    th2 = th(2)*-1;
    th3 = th(3)*-1;
end

function trans = trans(px,py,pz)
    trans = [1 0 0 px;0 1 0 py;0 0 1 pz;0 0 0 1];
end

function rot_x = rot_x(th)
    c = cos(th);
    s = sin(th);
    rot_x = [1 0 0 0;0 c -s 0;0 s c 0;0 0 0 1];
end

function rot_y = rot_y(th)
    c = cos(th);
    s = sin(th);
    rot_y = [c 0 s 0;0 1 0 0;-s 0 c 0;0 0 0 1];
end

function rot_z = rot_z(th)
    c = cos(th);
    s = sin(th);
    rot_z = [c -s 0 0;s c 0 0;0 0 1 0;0 0 0 1];
end


%Kinematics
function [pos_h,pos_3,pos_2,pos_1] = kinematics(th1,th2,th3)
    L1=50;
    L2=80;
    L3=120;
    
    
    A1=trans(0,0,0)*rot_x(0)*trans(0,0,L1)*rot_z(deg2rad(th1));
    A2=trans(0,0,0)*rot_x(deg2rad(th2))*trans(0,0,L2)*rot_z(0);
    A3=trans(0,0,0)*rot_x(deg2rad(th3))*trans(0,0,L3)*rot_z(0);
    
    pos_h = A1*A2*A3*[0;0;0;1];
    pos_3 = A1*A2*[0;0;0;1];
    pos_2 = A1*[0;0;0;1];
    pos_1 = [0;0;0;1];

    

end


%Calculate Trajectory
function [th1_ref th2_ref th3_ref]=traj3(start_th1,start_th2,start_th3,goal_th1,goal_th2,goal_th3) 
%Time to reach goal
T = 2;%[sec]
Ts = 0.05;

t = 0:Ts:T;

A = [1 0   0     0;
     1 T T^2   T^3;
     0 1   0     0;
     0 1 2*T 3*T^2];
Y = [start_th1 start_th2 start_th3;goal_th1 goal_th2 goal_th3;0 0 0;0 0 0];
C = (A)\Y;


th1_ref = [C(1,1)+C(2,1)*t+C(3,1)*t.^2+C(4,1)*t.^3];
th2_ref = [C(1,2)+C(2,2)*t+C(3,2)*t.^2+C(4,2)*t.^3];
th3_ref = [C(1,3)+C(2,3)*t+C(3,3)*t.^2+C(4,3)*t.^3];
end

