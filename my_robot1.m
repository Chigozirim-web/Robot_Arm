
% Using the make_scara_robot as a reference, 
%I was able to create my own robot(my_robot1) which, in its simplest form, is a robot arm (without the fingers).
%I visualized my robot arm on paper initially by drawing it and that made it easier to create.
%The links(objects) were created first, and then connected via joints which
%either rotated or translated, or were fixed,
%Then the various animations were applied for the robot arm to move in a 3d
%space. Basically, I made the lower arm move in virtually all directions, which is
%not totally similar to the human arm movement, but considering that it is a robot,
%I thought it could and should move in any direction.
%The code and a few more comments are written below

clear
close all
clf
handle_axes= axes('XLim', [-0.7,0.7], 'YLim', [-0.7,0.7], 'ZLim', [-1,1]);

xlabel('e_1'); 
ylabel('e_2');
zlabel('e_3');

view(-130, 20);
grid on;
axis; 
camlight
axis_length= 0.05;
%% %% Root Frame C
trf_C_axes= hgtransform('Parent', handle_axes); 
%creating the root link and set the parent as the axes earlier
%defined (handle_axes) 
%% Link-0: Base-link

trf_link0_C= make_transform([0, 0, 0], 0, 0, pi/4, trf_C_axes);
plot_axes(trf_link0_C, 'L_0', false, axis_length); 
%create the first transform 'link0' with respect to Frame C and perform the
%translation and rotation respectively. Set the root_link as its parent

trf_viz_link0= make_transform([0, 0, 0.05], 0, 0, 0, trf_link0_C);
radius0= 0.04;
h(1)= link_sphere(radius0, trf_viz_link0, [1, 0, 0.89373526]); %Visualizing link0- a magenta sphere- and plot the axes
plot_axes(trf_viz_link0, ' ', true, axis_length); %V_0
%% Link-1
trf_viz_link1= make_transform([0, 0, -0.1], 0, 0, 0); % Creating second object, do not specify parent yet: It will be done in the joint
length1= 0.3; radius1= 0.04;
h(2)= link_cylinder(radius1, length1, trf_viz_link1, [0, 1, 0]); %V_1
% plot_axes(trf_viz_link1, 'L1', false, axis_length);
% V_1 and L_1 are the same.
%%  Link-1_2
trf_viz_link1p2= make_transform([0, 0, -0.08], 0, 0, 0); % Do not specify parent yet: It will be done in the joint
%a sphere as a link between object 1 and 2 for more fluid movement between
%the objects
h(3)= link_sphere(0.04, trf_viz_link1p2, [0, 0, 1]); 
 plot_axes(trf_viz_link1p2, ' ', true, axis_length); % V_{1-2}
%%  Link-2
 trf_viz_link2 = make_transform([0, 0,-0.2], 0, 0, 0); % Creating third object, do not specify parent yet: It will be done in the joint
 h(4)= link_cylinder(0.04, 0.25, trf_viz_link2, [0, 0, 1]);
%V_2 and L_2 are the same
%% link-3
trf_viz_link3 = make_transform([0, 0, -0.018], 0, 0, 0);% Creating fourth object, do not specify parent yet: It will be done in the joint
h(5) = link_sphere(0.039, trf_viz_link3, [0.8786545, 0.15795326, 0]);


%% Link-End-Effector
trf_viz_linkEE= make_transform([0, 0, 0.001], 0, 0, 0); % Do not specify parent yet: It will be done in the joint
h(6)= link_sphere(0.005, trf_viz_linkEE, [1, 0, 0]); 
% plot_axes(trf_viz_linkEE, ' ', true, axis_length); 
%% Now define the joints
%% Joint 1: Links 0,1: Revolute
j1_rot_axis_j1= [0,0,1]';
j1_rot_angle= 0; % [-pi/2, pi/2]

trf_joint1_link0= make_transform([0, 0, 0], 0, 0, 0, trf_link0_C); 
trf_link1_joint1= make_transform_revolute(j1_rot_axis_j1, j1_rot_angle, trf_joint1_link0); 
plot_axes(trf_link1_joint1, 'L_1', false, axis_length); 
make_child(trf_link1_joint1, trf_viz_link1);
%Joint 1 coonects link0 and link1. It is a revolute joint and rotates about
%the z-axis by an (a resultant) angle of 0(from -90degrees to 90degrees)

%% Joint 2: Links 1,2: Revoulute
j2_rot_axis_j2= [0,1,0]';
j2_rot_angle= 0; % [-pi/2, pi/2]

trf_joint2_link1= make_transform([0, 0, -0.2], 0, 0, 0, trf_link1_joint1); 
trf_link2_joint2= make_transform_revolute(j2_rot_axis_j2, j2_rot_angle, trf_joint2_link1); 
plot_axes(trf_link2_joint2, 'L_2', false, axis_length); 
make_child(trf_link2_joint2, trf_viz_link2);
%Joint 2 is another revolute joint that rotates about the y-axis and
%connects links 1 and 2, links 1 and 1_2, and links 2 and 1_2
%% %%  Joint: Links 1,1_2: Fixed
trf_link1p2_link1= make_transform([0, 0, 0], 0, 0, 0, trf_link2_joint2); 
make_child(trf_link1p2_link1, trf_viz_link1p2);
%%   Joint: Links 2,1_2: Fixed
trf_link1p2_link2 = make_transform([0, 0, 0,], 0, 0, 0,trf_link2_joint2);
make_child(trf_link1p2_link2, trf_viz_link1p2);

% I had to connect Link 1_2 and 2 as well because when the former was connected to
% just link1 it moved seperately from link 2( it did not exactly rotate
% together with it). Connecting them both via joint 2 makes their together
% movement possible
%% Joint 3: Links 2,3: Prismatic
j3_translation_axis_j3= [0,0,1]';
j3_translation= 0; % [-0.04, 0.04]

trf_joint3_link2= make_transform([0, 0, -0.35], 0, 0, 0, trf_link2_joint2); 
trf_link3_joint3= make_transform_prismatic(j3_translation_axis_j3, j3_translation, trf_joint3_link2);
plot_axes(trf_link3_joint3, 'L_3', false, axis_length); 
make_child(trf_link3_joint3, trf_viz_link3);
% Joint 3 is a prismatic joint which translates about the z-axis and
% connects link2 and link3 

%% Joint 4: Links 4,EE: Fixed
trf_linkEE_link3= make_transform([0, 0, 0], 0, 0, 0, trf_link3_joint3); 
make_child(trf_linkEE_link3, trf_viz_linkEE);

%%  Animation: One joint at a time

%a for loop has to be created to animate the 3d objects

for q1=[linspace(0, -pi/2, 30), linspace(-pi/2, pi/2, 30), linspace(pi/2, 0, 30)]
    set(handle_axes, 'XLim', [-0.7,0.7], 'YLim', [-0.7,0.7], 'ZLim', [-1,1]);
    trf_q1= makehgtform('axisrotate', j1_rot_axis_j1, q1);
    set(trf_link1_joint1, 'Matrix', trf_q1);
    drawnow;
    pause(0.02);
end

%The first for-loop animates all objects connected via Joint1. This joint
%animation also moves all other objects since link0 is the initial object
%created and was set as a parent from the beginning
%It rotates about the z-axis from 0 to -90degrees to 90degrees and then back to 0 based on the array defined by q1 

for q2=[linspace(0, -pi/4, 30), linspace(-pi/4, pi/4, 30), linspace(pi/4, 0, 30)]
    set(handle_axes, 'XLim', [-0.7,0.7], 'YLim', [-0.7,0.7], 'ZLim', [-1,1]);
    trf_q2= makehgtform('axisrotate', j2_rot_axis_j2, q2);
    set(trf_link2_joint2, 'Matrix', trf_q2);
    drawnow;
    pause(0.02);
end

%second loop animates the objects connected by Joint2. It rotates about the
%y-axis from 0 to -45degrees to 45degrees and back to 0 based on the array
%defined by q2

for  q3=[linspace(0, -0.07, 30), linspace(-0.07, 0.07, 30), linspace(0.07, 0, 30)]
    set(handle_axes, 'XLim', [-0.7,0.7], 'YLim', [-0.7,0.7], 'ZLim', [-1,1]);
    trf_q3= makehgtform('translate', j3_translation_axis_j3*q3);
    set(trf_link3_joint3, 'Matrix', trf_q3);
    drawnow;
    pause(0.02);
end

%third loop animates the objects connected via joint3. 
%It translates by the array of numbers contained in q3, about the z-axis 

%% Animation: All joints together.
q_init= 0.2*ones(4,1); % This leads to all joints being at 0.
     %ones(4,1) creates a 4 by 1 matrix with figure 1 as its only elements
for i= 1:20
    q_next= rand(4,1); 
    % rand() gives uniformly distributed random numbers in the interval [0,1]
    
    for t=0:0.02:1 %setting t with intervals of 0.02 makes the animation a little bit slower than if it were 0.05 or 0.1 
        q=  q_init + t*(q_next - q_init);
        q1= (pi/4)*(2*q(1)-1);
        q2= (pi/4)*(2*q(2)-1);
        q3= (0.05)*(2*q(3)-1);
        
        set(handle_axes, 'XLim', [-0.7,0.7], 'YLim', [-0.7,0.7], 'ZLim', [-1,1]);
        trf_q1= makehgtform('axisrotate', j1_rot_axis_j1, q1);
        set(trf_link1_joint1, 'Matrix', trf_q1);
        
        set(handle_axes, 'XLim', [-0.7,0.7], 'YLim', [-0.7,0.7], 'ZLim', [-1,1]);
        trf_q2= makehgtform('axisrotate', j2_rot_axis_j2, q2);
        set(trf_link2_joint2, 'Matrix', trf_q2);
        
        set(handle_axes, 'XLim', [-0.7,0.7], 'YLim', [-0.7,0.7], 'ZLim', [-1,1]);
        trf_q3= makehgtform('axisrotate', j3_translation_axis_j3, q3);
        set(trf_link3_joint3, 'Matrix', trf_q3);
        drawnow;
        pause(0.005);
        
    end
    
    q_init= q_next;
    
end





