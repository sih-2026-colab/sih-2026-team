function autonex_sfun(block)
% Level-2 MATLAB S-function for normal-mode closed-loop simulation.
block.NumDialogPrms=0; block.NumInputPorts=0; block.NumOutputPorts=1;
block.OutputPort(1).Dimensions=8; block.OutputPort(1).DatatypeID=0;
block.OutputPort(1).Complexity='Real'; block.OutputPort(1).SamplingMode='Sample';
block.SampleTimes=[.05 0]; block.SimStateCompliance='HasNoSimState';
block.RegBlockMethod('Start',@start);
block.RegBlockMethod('Outputs',@outputs);
end
function start(~)
clear autonex_simulink_step;
end
function outputs(block)
block.OutputPort(1).Data=autonex_simulink_step(block.CurrentTime)';
end
