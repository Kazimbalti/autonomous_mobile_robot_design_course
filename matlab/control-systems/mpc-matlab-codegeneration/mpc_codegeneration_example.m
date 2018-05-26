%%  MPC Code Generation
%
%   MATLAB offers the ability to generate code in order to deploy a Model
%   Predictive Controller
%
%   Example: 
%   https://www.mathworks.com/help/mpc/ug/generate-code-for-mpc-controller-with-custom-qp-solver.html
%
%   References
%   [1] Bemporad, A. and Mosca, E. "Fulfilling hard constraints in 
%   uncertain linear systems by reference managing." Automatica, Vol. 34, 
%   Number 4, pp. 451-461, 1998.
    

%%  DC Servo Motor Model

[plant,tau] = mpcmotormodel;

%%  Design MPC Controller

plant = setmpcsignals(plant,'MV',1,'MO',1,'UO',2);
MV = struct('Min',-220,'Max',220,'ScaleFactor',440);
OV = struct('Min',{-Inf, [-tau;-tau;-tau;-Inf]},...
    'Max',{Inf, [tau;tau;tau;Inf]},'ScaleFactor',{2*pi, 2*tau});
Weights = struct('MV',0,'MVRate',0.1,'OV',[0.1 0]);

Ts = 0.1;           % Sample time
p = 10;             % Prediction horizon
m = 2;              % Control horizon
mpcobj = mpc(plant,Ts,p,m,Weights,MV,OV);

%%  Simulate in Simulink with Built-In QP Solver

if ~mpcchecktoolboxinstalled('simulink')
    disp('Simulink is required to run this example.')
    return
end

mdl = 'mpc_customQPcodegen';
open_system(mdl)

sim(mdl)

uKWIK = u;
yKWIK = y;

%%  Simulate in Simulink with a Custom QP Solver

mpcobj.Optimizer.CustomSolver = true;

src = which('mpcCustomSolverCodeGen_TemplateEML.txt');
dest = fullfile(pwd,'mpcCustomSolver.m');
copyfile(src,dest,'f')

sim(mdl)
uDantzigSim = u;
yDantzigSim = y;

%%  Generate Code with Custom QP Solver

if ~mpcchecktoolboxinstalled('simulinkcoder')
    disp('Simulink(R) Coder(TM) is required to run this example.')
    return
end

mpcobj.Optimizer.CustomSolverCodeGen = true;

src = which('mpcCustomSolverCodeGen_TemplateEML.txt');
dest = fullfile(pwd,'mpcCustomSolverCodeGen.m');
copyfile(src,dest,'f')

%%  Generate executable code from the Simulink model using the rtwbuild command from Simulink Coder.

rtwbuild(mdl)

%%  Run the Executable

if ispc
    status = system(mdl);
    load(mdl)
    uDantzigCodeGen = u;
    yDantzigCodeGen = y;
else
    disp('The example only runs the executable on Windows system.');
end

%%  Compare Simulation Results

if ispc
    figure
    subplot(2,1,1)
    plot(u.time,uKWIK.signals.values,u.time,uDantzigSim.signals.values,...
        '+',u.time,uDantzigCodeGen.signals.values,'o')
    subplot(2,1,2)
    plot(y.time,yKWIK.signals.values,y.time,yDantzigSim.signals.values,...
        '+',y.time,yDantzigCodeGen.signals.values,'o')
    legend('KWIK','Dantzig Simu','Dantzig CodeGen','Location','northwest')
else
    figure
    subplot(2,1,1)
    plot(u.time,uKWIK.signals.values,u.time,uDantzigSim.signals.values,'+')
    subplot(2,1,2)
    plot(y.time,yKWIK.signals.values,y.time,yDantzigSim.signals.values,'+')
    legend('KWIK','Dantzig Simu','Location','northwest')
end


