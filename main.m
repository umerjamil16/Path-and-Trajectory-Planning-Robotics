
%% Initialization
clear ; close all; clc

%% sec Plane
y=linspace(0,200,40);
z=linspace(-0,200,40);
[y,z]=meshgrid(y,z);
x=linspace(0,-200,40);
mesh(x,y,z)
grid on
box on
view([130,30])
surf(x,y,z)
alpha 0
hold on

% a=0;
% if a ~= 1
%   return;
% end
%% sec Plane
y=linspace(0,-200,40);
z=linspace(-0,200,40);
[y,z]=meshgrid(y,z);

x=linspace(0,-200,40);%x.^4;
mesh(x,y,z)
grid on
box on
view([130,30])
surf(x,y,z)
alpha 0
hold on

%% the curve
% R = linspace(0,-200*sqrt(2),80);
R = -200*sqrt(2);

for i = 1:length(R)
theta = linspace(-pi/4,pi/4,40);
zt = linspace(0,200,40);
yt = R(i).*sin(theta);
xt = R(i).*cos(theta);
[yt,zt]=meshgrid(yt,zt);

 surf(xt,yt,zt)

 hold on
 alpha 0

end

alpha 0

%% labels
xlabel('x-axis');
ylabel('y-axis');
zlabel('z-axis');
%% Robot Model Definition
d1 = 5;
L1 = Revolute('a', 0, 'd', d1, 'qlim', [(1*pi/4),(3*pi/4)],'standard');
L2 = Prismatic('a', 0, 'alpha', -pi/2, 'qlim',[5,200],'standard');
L3 = Prismatic('a', 0, 'qlim',[2,abs(R)],'standard');
robot1 = SerialLink( [L1 L2 L3],'name', 'RPP/Cake Robot');
text(0,0,0,' RPP ROBOT')

pf = [];
pl = [];

while(1)
    if length(pf) <1
        prompt = 'Enter first position ';
        pfinal = input(prompt);
        pf = [-pfinal(2),-pfinal(1),pfinal(3)];
        T1 = transl(pf);
        EE_first = inv_kine(T1(1,4),T1(2,4),T1(3,4));
        hold on
    end

    prompt = 'Enter last position ';
%     pl = input(prompt);
        p2 = input(prompt);
        pl = [-p2(2),-p2(1),p2(3)];
    hold on

    text(p2(1),p2(2),p2(3),['(' num2str(p2(1)) ',' num2str(p2(2))  ',' num2str(p2(3)) ')'])
    hold on
    text(pfinal(1),pfinal(2),pfinal(3),['(' num2str(pfinal(1)) ',' num2str(pfinal(2))  ',' num2str(pfinal(3)) ')'])
    hold on
    
    T2 = transl(pl);
    EE_last = inv_kine(T2(1,4),T2(2,4),T2(3,4));

    via = [ EE_last];
    q =  mstraj(via, 1, [], EE_first,2, 2); %

%    r = @() mstraj(via, 1, [], EE_first,2, 2); %
    


    for m=1:length(q)
      vars = q(m,:);
      pos = robot1.fkine(vars);
      q1_limit = L1.islimit(vars(1));
      q2_limit = L2.islimit(vars(2));
      q3_limit = L3.islimit(vars(3));

      jval=robot1.jacob0(vars);
      hold on

      if det(jval(1:3,1:3)) == 0
          fprintf('\n Singularity reached\n');
          plot3(pos(1,1).t(1),pos(1,1).t(2),pos(1,1).t(3),'.b')
          hold on
      elseif q1_limit ~=0 
           fprintf('\n Joint 1 out of limit\n');
           fprintf('\n Quiting program\n');
            return;
      elseif q2_limit ~=0 
           fprintf('\n Joint 2 out of limit\n');
            return;
      elseif q3_limit ~=0
           fprintf('\n Joint 3 out of limit\n');
            return;    
       else
           plot3(pos(1,1).t(1),pos(1,1).t(2),pos(1,1).t(3),'.r')
           hold on
      end
    hold on
    end
    
    hold on
    t1 = cputime;

    
    robot1.plot(q,'fps',400,'noname','noshadow','notiles','jointdiam',1, 'workspace', [-400,400,-400,400,-0,200])
    fprintf('\nTime Taken(secs) to reach the goal: %f\n', cputime-t1);
    
    hold on;
    
%     %% plot q, qd, qdd graphs

[x, xd, xdd]=jtraj(EE_first, EE_last, 100);
    figure('Name','q, qd, qdd Plots');

    subplot(2,2,[1 3])
    plot(x);
    title('q')  
    xlabel('Value of q1, q2, q3'); 
    ylabel('t/sec');
    legend('q1','q2','q3')

    subplot(2,2,2)
    plot(xd);
    title('qd')  
    xlabel('Value of q1d, q2d, q3d'); 
    ylabel('t/sec');
    legend('q1d','q2d','q3d')


    subplot(2,2,4)
    plot(xdd);
    title('qdd')  
    xlabel('Value of q1dd, q2dd, q3dd'); 
    ylabel('t/sec');
    legend('q1dd','q2dd','q3dd')
    
    %% for the next iteration
    EE_first =EE_last;
end
% working good now, multiple points can be entered
robot1.teach('rpy')

%% labels
xlabel('x-axis');
ylabel('y-axis');
zlabel('z-axis');