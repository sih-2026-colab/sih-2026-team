function verify_autonex_simulink()
build_autonex_simulink;
model='main_autonomous_vehicle';
load_system(fullfile('simulink','integration',[model '.slx']));
result=sim(model,'StopTime','6');
values=result.autonex_log;
reference=run_autonex_simulation(struct('duration',6));
expected=[[reference.samples.x]' [reference.samples.y]' [reference.samples.speed]' ...
    [reference.samples.ax]' [reference.samples.ay]' [reference.samples.trackCount]' ...
    [reference.samples.emergency]' [reference.samples.collision]'];
assert(isequal(size(values),size(expected)),'Simulink sample count mismatch');
error=max(abs(values-expected),[],'all');
assert(error<1e-8,'MATLAB and Simulink outputs differ');
save('results/simulink_verification.mat','values','expected','error');
close_system(model,0);
fprintf('SIMULINK_SHARED_LOOP_PASS max error %.3g\n',error);
check_roadrunner_readiness;
end
