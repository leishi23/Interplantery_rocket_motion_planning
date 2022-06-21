BEGIN_ACADO;                                % Always start with "BEGIN_ACADO". 
    acadoSet('problemname', 'initialization');
    % (a) Define states and controls 
    DifferentialState       t;
    
    DifferentialState       p_x;              % The differential states 
    DifferentialState       p_y;              % The differential states 
    
    DifferentialState       v_x;              % The differential states 
    DifferentialState       v_y;
    
    DifferentialState       theta;            % The differential states 
    DifferentialState       theta_dot;        % The differential states 
    
    DifferentialState       targetx;
    DifferentialState       targety;
    
    DifferentialState       ex;
    DifferentialState       ey;
    
    Control                 u_l;
    Control                 u_r;
    
    G = 2e-2;
    %% (a) Differential Equation
    f = acado.DifferentialEquation(0, 10);
    f.add(dot(p_x) == v_x);
    f.add(dot(p_y) == v_y);
    f.add(dot(v_x) == u_l*cos(theta)-p_x*G/(p_x^2+p_y^2)^1.5);
    f.add(dot(v_y) == u_l*sin(theta)-p_y*G/(p_x^2+p_y^2)^1.5);
    f.add(dot(theta) == theta_dot);
    
    f.add(dot(t) == 1);
    f.add(dot(theta_dot)== u_r);
    
    f.add(dot(targetx) == -11.0*sin(0.05*t)*0.05);
    f.add(dot(targety) == 11.0*cos(0.05*t)*0.05);
    
    f.add(dot(ex) == v_x+11.0*sin(0.05*t)*0.05);
    f.add(dot(ey) == v_y-11.0*cos(0.05*t)*0.05);
    
    %% (b) Optimal Control Problem
    ocp = acado.OCP(0.0,15,64);
    % (b) Minimize control effort
    ocp.minimizeLSQ(u_l,0);
    ocp.minimizeLSQ(u_r,0);
    % (c) Path constraints
    % the constrain that do not allow track to go into circle r=1 centered at -7,0
    ocp.subjectTo( 8 <= ((p_x)^2+(p_y)^2));
    ocp.subjectTo(-1 <= v_x <= 1);
    ocp.subjectTo(-1 <= v_y <= 1);
    ocp.subjectTo(-.5 <= u_l <= .5);
    ocp.subjectTo(-pi/40 <= u_r <= pi/40);
    %ocp.subjectTo(3 <= T <= 10);
    ocp.subjectTo(f);
    % (d) Initial Conditions
    ocp.subjectTo('AT_START',p_x == 2.0*sqrt(2));
    ocp.subjectTo('AT_START',p_y == 0.0);
    ocp.subjectTo('AT_START',v_x == 0.0);
    ocp.subjectTo('AT_START',v_y == 0.0);
    ocp.subjectTo('AT_START',theta == 0.0);
    ocp.subjectTo('AT_START',theta_dot == 0.0);
    ocp.subjectTo('AT_START',t == 0.0);

    ocp.subjectTo('AT_START',targetx == 11.0);
    ocp.subjectTo('AT_START',targety == 0.0);
    ocp.subjectTo('AT_START',ex == 2.0*sqrt(2)-11.0);
    ocp.subjectTo('AT_START',ey == 0.0);
    % (d) Final boundary conditions
    ocp.subjectTo('AT_END',ex == 0);
    ocp.subjectTo('AT_END',ey == 0);
 
    % (e) Optimization Algorithm
    % here u_initial is the direct result from problem 1
    algo = acado.OptimizationAlgorithm(ocp);
    algo.set('KKT_TOLERANCE',1e-8);
    algo.set('DISCRETIZATION_TYPE','SINGLE_SHOOTING');
    algo.set('MAX_NUM_ITERATIONS',400);
END_ACADO;           % Always end with "END_ACADO".
                     % This will generate a file problemname_ACADO.m. 
                     % Run this file to get your results. You can
                     % run the file problemname_ACADO.m as many
                     % times as you want without having to compile again.
% Run the test
out1 = initialization_RUN();