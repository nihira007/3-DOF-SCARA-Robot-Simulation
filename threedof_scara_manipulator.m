clc; clear; close all;
clear drawRobot
global RESET_OBJECT
RESET_OBJECT = true;

%% ================= PARAMETERS =================
L2 = 0.28;
L3 = 0.22;

z_min = 0.08;
z_max = 0.28;

%% ================= FIGURE =================
figure('Color','w','Name','3-DOF SCARA Pick & Place');
axis equal; grid on; hold on;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
xlim([-0.6 0.6]); ylim([-0.4 0.4]); zlim([0 0.6]);
view(40,25);

title('3-DOF SCARA Pick-and-Place Manipulator', ...
      'FontSize',12,'FontWeight','bold');

%% ================= INITIAL VALUES =================
t1 = 0; t2 = 0; z = z_min;

%% ================= SLIDERS =================
sZ = uicontrol('Style','slider','Min',z_min,'Max',z_max,'Value',z,...
    'Units','normalized','Position',[0.1 0.02 0.25 0.04]);

sT1 = uicontrol('Style','slider','Min',-pi,'Max',pi,'Value',t1,...
    'Units','normalized','Position',[0.4 0.02 0.25 0.04]);

sT2 = uicontrol('Style','slider','Min',-pi,'Max',pi,'Value',t2,...
    'Units','normalized','Position',[0.7 0.02 0.25 0.04]);

%% ================= TASK POINTS =================
home  = [0.25  0.00  0.20];
pick  = [0.30  0.10  0.10];
place = [0.20 -0.15  0.12];

%% ================= AUTONOMOUS MODE =================
sZ.Enable  = 'off';
sT1.Enable = 'off';
sT2.Enable = 'off';

pause(1);  
autonomousPickPlace(sT1,sT2,sZ,home,pick,place,L2,L3,z_min,z_max);

global RESET_OBJECT
RESET_OBJECT = true;

clear drawRobot          
drawRobot(0,0,z_min,L2,L3,home,pick,place);

%% ================= ENABLE MANUAL =================
if ishandle(sZ)
    sZ.Enable  = 'on';
end
if ishandle(sT1)
    sT1.Enable = 'on';
end
if ishandle(sT2)
    sT2.Enable = 'on';
end


%% ================= MANUAL LOOP =================
while ishandle(sZ)
    drawRobot(sT1.Value,sT2.Value,sZ.Value,L2,L3,home,pick,place);
    pause(0.02);
end

%% ================= AUTONOMOUS =================
function autonomousPickPlace(sT1,sT2,sZ,home,pick,place,L2,L3,z_min,z_max)

% initial wait
q_home = IK(home, L2, L3);

sT1.Value = q_home(1);
sT2.Value = q_home(2);
sZ.Value  = min(max(q_home(3), z_min), z_max);

drawRobot(q_home(1), q_home(2), sZ.Value, L2, L3, home, pick, place);
pause(3); 

poses = {
    home
    pick + [0 0 0.08]
    pick
    pick + [0 0 0.08]
    place + [0 0 0.08]
    place
    place + [0 0 0.08]
    home
};

for k = 1:size(poses,1)-1
    q1 = IK(poses{k},L2,L3);
    q2 = IK(poses{k+1},L2,L3);
    if isempty(q1) || isempty(q2), continue; end

    for s = linspace(0,1,40)

    if ~ishandle(sT1) || ~ishandle(sT2) || ~ishandle(sZ)
        return; 
    end

    q = (1-s)*q1 + s*q2;

    sT1.Value = q(1);
    sT2.Value = q(2);
    sZ.Value  = min(max(q(3),z_min),z_max);

    drawRobot(q(1),q(2),sZ.Value,L2,L3,home,pick,place);
    pause(0.04);
    end

end

% ===== END OF AUTO =====
q_home = IK(home, L2, L3);

sT1.Value = q_home(1);
sT2.Value = q_home(2);
sZ.Value  = min(max(q_home(3), z_min), z_max);

drawRobot(q_home(1), q_home(2), sZ.Value, L2, L3, home, pick, place);
pause(2);

end

%% ================= INVERSE KINEMATICS =================
function q = IK(p,L2,L3)
x=p(1); y=p(2); z=p(3);
D=(x^2+y^2-L2^2-L3^2)/(2*L2*L3);
if abs(D)>1, q=[]; return; end
t2=atan2(sqrt(1-D^2),D);
t1=atan2(y,x)-atan2(L3*sin(t2),L2+L3*cos(t2));
q=[t1 t2 z];
end

%% ================= FORWARD KINEMATICS =================
function [p0,p1,p2]=forwardKinematics(t1,t2,z,L2,L3)
p0=[0 0 z];
p1=[L2*cos(t1) L2*sin(t1) z];
p2=[p1(1)+L3*cos(t1+t2) p1(2)+L3*sin(t1+t2) z];
end

%% ================= DRAW ROBOT =================
function drawRobot(t1,t2,z,L2,L3,home,pick,place)

global RESET_OBJECT
persistent obj_pos picked placed

if isempty(obj_pos) || RESET_OBJECT
    obj_pos = pick;
    picked  = false;
    placed  = false;
    RESET_OBJECT = false;   
end


if ~ishandle(gca)
    return;
end
cla;

[p0,p1,p2]=forwardKinematics(t1,t2,z,L2,L3);

drawBox([-0.4 -0.3 0],[0.8 0.6 0.05],[0.9 0.9 0.95]);
drawCylinder([0 0 0],0.06,0.35,[0.4 0.4 0.7]);

drawBoxArm(p0,p1,0.05,0.04,[0.85 0.15 0.15]);
drawBoxArm(p1,p2,0.05,0.04,[0.85 0.15 0.15]);
plot3(p2(1),p2(2),p2(3),'ko','MarkerFaceColor','k');


xy_tol = 0.015;     
z_tol  = 0.008;     
hold_frames = 6;   

persistent pick_hold place_hold
if isempty(pick_hold)
    pick_hold = 0;
    place_hold = 0;
end

% -------- PICK LOGIC --------
if ~picked && ...
   norm(p2(1:2) - pick(1:2)) < xy_tol && ...
   abs(p2(3) - pick(3)) < z_tol

    pick_hold = pick_hold + 1;
else
    pick_hold = 0;
end

if pick_hold >= hold_frames
    picked = true;
    pick_hold = 0;
end

% -------- OBJECT FOLLOW --------
if picked
    obj_pos = p2;
end

% -------- PLACE LOGIC --------
if picked && ~placed && ...
   norm(p2(1:2) - place(1:2)) < xy_tol && ...
   abs(p2(3) - place(3)) < z_tol

    place_hold = place_hold + 1;
else
    place_hold = 0;
end

if place_hold >= hold_frames
    picked  = false;
    placed  = true;      
    obj_pos = place;
    place_hold = 0;
end

drawBox(obj_pos-[0.012 0.012 0.012],[0.025 0.025 0.025],[1 0.4 0]);
drawTaskPoints(home,pick,place);
axis([-0.6 0.6 -0.4 0.4 0 0.6]); 

drawnow;
end

%% ================= VISUAL HELPERS =================
function drawTaskPoints(h,p,pl)

sz = 0.012;    
z_off = 0.002;  

% ---- HOME (green) ----
drawBox([h(1)-sz/2, h(2)-sz/2, h(3)+z_off], ...
        [sz sz sz], [0 0.7 0]);
% ---- PICK (blue) ----
drawBox([p(1)-sz/2, p(2)-sz/2, p(3)+z_off], ...
        [sz sz sz], [0 0.4 0.9]);
% ---- PLACE (red) ----
drawBox([pl(1)-sz/2, pl(2)-sz/2, pl(3)+z_off], ...
        [sz sz sz], [0.9 0.2 0.2]);

end


function drawBox(pos,sz,color)
[x,y,z]=ndgrid([0 sz(1)],[0 sz(2)],[0 sz(3)]);
V=[x(:)+pos(1) y(:)+pos(2) z(:)+pos(3)];
F=[1 3 4 2;5 6 8 7;1 2 6 5;3 4 8 7;1 3 7 5;2 4 8 6];
patch('Vertices',V,'Faces',F,'FaceColor',color,'EdgeColor','none');
end

function drawCylinder(p,r,h,c)
[x,y,z]=cylinder(r,30); z=z*h+p(3);
surf(x+p(1),y+p(2),z,'FaceColor',c,'EdgeColor','none');
end

function drawBoxArm(p1,p2,w,h,c)
v=p2-p1; L=norm(v); a=atan2(v(2),v(1));
V=[0 -w/2 -h/2;L -w/2 -h/2;L w/2 -h/2;0 w/2 -h/2;
   0 -w/2 h/2;L -w/2 h/2;L w/2 h/2;0 w/2 h/2];
R=[cos(a) -sin(a) 0; sin(a) cos(a) 0; 0 0 1];
V=(R*V')'+p1;
F=[1 2 3 4;5 6 7 8;1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8];
patch('Vertices',V,'Faces',F,'FaceColor',c,'EdgeColor','none');
end

