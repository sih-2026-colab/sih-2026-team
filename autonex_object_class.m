function label=autonex_object_class(value)
% Stable public labels; unknown is not inferred to be stationary.
value=lower(strrep(char(value),'_','-'));
switch value
    case {'car','vehicle'}, label='car';
    case 'bus', label='bus';
    case {'bike','bicycle','motorcycle','scooter','two-wheeler'}, label='two-wheeler';
    case {'auto','rickshaw','auto-rickshaw'}, label='auto-rickshaw';
    case 'pedestrian', label='pedestrian';
    case 'animal', label='animal';
    case {'cart','pushcart'}, label='pushcart';
    case {'obstacle','static','static-obstacle'}, label='static obstacle';
    otherwise, label='unknown';
end
end
