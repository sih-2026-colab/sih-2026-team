function [length,width]=autonex_actor_size(actor)
length=4.5; width=1.9;
if strcmpi(actor.type,'pedestrian'), length=.6; width=.6;
elseif strcmpi(actor.type,'animal'), length=2; width=.8;
elseif strcmpi(actor.type,'pothole'), length=3; width=2;
end
end
