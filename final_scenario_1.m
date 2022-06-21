clear all;
clc;

BEGIN_ACADO;                                % Always start with "BEGIN_ACADO". 
    
    acadoSet('problemname', 'final_project');
    
    % (a) Define states and controls 
    
    
    DifferentialState px;                    % x-position
    DifferentialState py;                    % y-position
    DifferentialState theta;                 % direction
    DifferentialState delta;                 % wheel angle
    DifferentialState v;                     % velocity
    DifferentialState S1x;                   % obstacle_1 x position
    DifferentialState S1y;                   % obstacle_1 y position  
    DifferentialState S2x;                   % obstacle_2 x position
    DifferentialState S2y;                   % obstacle_2 y position 
    DifferentialState S3x;                   % obstacle_3 x position
    DifferentialState S3y;                   % obstacle_3 y position 
    DifferentialState t;                     % time
    DifferentialState ex;                    % error between px and S1x
    DifferentialState ey;                    % error between py and S1y
%     DifferentialState J;                     % cost accumulation
    
    
    Control u_a;                             % Control input for acceleration
    Control u_delta;                         % Control input for jet angle

    
     Parameter t_f;
     Parameter g1;
     Parameter g2;
     Parameter g3;
     Parameter ff;
     
     g1 = 1.56e-5;
     g2 = 2e-4;
     g3 = 5.63e-3;
     ff = 0.001;
     t_f = 15;
     num_steps = 50;

     
    %% (a) Differential Equation
    
    
    f = acado.DifferentialEquation(0, t_f);
    
    % graviation force
    F_s1 = g1/((px-S1x)^2+(py-S1y)^2);
    F_s2 = g2/((px-S2x)^2+(py-S2y)^2);
    F_s3 = g3/((px-S3x)^2+(py-S3y)^2);
%     fi_1 = atan2(S1y-py,S1x-px);
%     fi_2 = atan2(S2y-py,S2x-px);
%     fi_3 = atan2(S3y-py,S3x-px);
    
    f.add(dot(px)     == v*cos(theta));
    f.add(dot(py)     == v*sin(theta));
    f.add(dot(theta)  == v*tan(delta));
    % f.add(dot(v)      == u_a);
    f.add(dot(v*cos(theta))== u_a*cos(theta)-ff*(v*cos(theta))^2+F_s1+F_s2+F_s3);
    f.add(dot(v*sin(theta))== u_a*sin(theta)-ff*(v*sin(theta))^2+F_s1+F_s2+F_s3);
    f.add(dot(delta)  == u_delta);   
    
    f.add(dot(t)      == 1.0);
    
    f.add(dot(S1x)    == -1*0.1*pi*sin(pi/3+0.1*pi*t)); %S1x = S0x+r1*cos(theta10+w1*t); ie. -5+1*cos(pi/3+0.1*pi*t)
    f.add(dot(S1y)    == 1*0.1*pi*cos(pi/3+0.1*pi*t));  %S1y = S0y+r1*sin(theta10+w1*t); ie. -0.5+1*sin(pi/3+0.1*pi*t)  
    f.add(dot(S2x)    == -8*0.02*pi*sin(2/3*pi+0.02*pi*t)); %S2x = S0x+r2*cos(theta20+w2*t); ie. -5+8*cos(2/3*pi+0.02*pi*t)
    f.add(dot(S2y)    == 8*0.02*pi*cos(2/3*pi+0.02*pi*t));  %S2y = S0y+r2*sin(theta20+w2*t); ie. -0.5+8*sin(2/3*pi+0.02*pi*t)  
    f.add(dot(S3x)    == -5*0.04*pi*sin(pi/2+0.04*pi*t)); %S3x = S0x+r3*cos(theta30+w3*t); ie. -5+5*cos(pi/2+0.04*pi*t)
    f.add(dot(S3y)    == 5*0.04*pi*cos(pi/2+0.04*pi*t));  %S3y = S0y+r3*sin(theta30+w3*t); ie. -0.5+5*sin(pi/2+0.04*pi*t)    
    
    f.add(dot(ex)    == (v*cos(theta))-(-1*0.1*pi*sin(pi/3+0.1*pi*t)));
    f.add(dot(ey)    == (v*sin(theta))-(1*0.1*pi*cos(pi/3+0.1*pi*t)));
    
%     f.add(dot(J) == (u_a*0.5*u_a+u_delta*0.5*u_delta));
    
    %% (b) Optimal Control Problem
    
    
    ocp = acado.OCP(0.0, t_f, num_steps);

   
    % (b) Minimize control effort

    ocp.minimizeLSQ(u_a, 0);  
    ocp.minimizeLSQ(u_delta, 0);     
    
    % (c) Path constraints
        
    ocp.subjectTo(f);
    
    ocp.subjectTo(-1 <= v <=1);
    ocp.subjectTo(-0.5 <= u_a <=0.5);
    ocp.subjectTo(-pi/4 <= delta <= pi/4);
    ocp.subjectTo(-pi/6 <= u_delta <= pi/6);
%     ocp.subjectTo( 1.0 <= sqrt((px-S1x)^2+(py-S1y)^2 ));  
    ocp.subjectTo( 1.2 <= sqrt((px-S2x)^2+(py-S2y)^2 ));  
    ocp.subjectTo( 1.2 <= sqrt((px-S3x)^2+(py-S3y)^2 ));  
    ocp.subjectTo( 0.65 <= sqrt((px+5)^2+(py+0.5)^2 ));  
    
    % (d) Initial Conditions
    
    ocp.subjectTo('AT_START', px == -13);
    ocp.subjectTo('AT_START', py == 0); 
    ocp.subjectTo('AT_START', v == 0.0);
    ocp.subjectTo('AT_START', theta == 0.0);
    ocp.subjectTo('AT_START', delta == 0.0);
    ocp.subjectTo('AT_START', t == 0);
    
    ocp.subjectTo('AT_START', S1x == -4.5);%-5+1*cos(pi/3)
    ocp.subjectTo('AT_START', S1y == 0.366025);%-0.5+1*sin(pi/3)  
    ocp.subjectTo('AT_START', S2x == -9);
    ocp.subjectTo('AT_START', S2y == 6.4282);
    ocp.subjectTo('AT_START', S3x == -5.0);
    ocp.subjectTo('AT_START', S3y == 4.5);
    
    ocp.subjectTo('AT_START', ex == -8.5);
    ocp.subjectTo('AT_START', ey == -0.366025);
    
    % (d) Final boundary conditions
    
%     ocp.subjectTo('AT_END', px == -5);
%     ocp.subjectTo('AT_END', py == -0.5);
    ocp.subjectTo('AT_END', v == 0.0);
    ocp.subjectTo('AT_END', theta == 0.0);
    ocp.subjectTo('AT_END', ex == 0.0);
    ocp.subjectTo('AT_END', ey == 0.0);
    
    %% (e) Optimization Algorithm
    
    algo =acado.OptimizationAlgorithm(ocp);
    algo.set( 'KKT_TOLERANCE', 1e-5 );        %revise presion
    algo.set( 'DISCRETIZATION_TYPE', 'MULTIPLE_SHOOTING');
    algo.set('MAX_NUM_ITERATIONS', 500); 

    
END_ACADO;           % Always end with "END_ACADO".
                     % This will generate a file problemname_ACADO.m. 
                     % Run this file to get your results. You can
                     % run the file problemname_ACADO.m as many
                     % times as you want without having to compile again.

% Run the test
out = final_RUN();