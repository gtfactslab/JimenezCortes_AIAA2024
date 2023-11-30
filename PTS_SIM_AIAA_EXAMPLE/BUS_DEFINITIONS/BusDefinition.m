function BusDefinition(structure_in,bus_name) 
% BUSDEFINITION initializes a set of bus objects for a SIMULINK model in
% the MATLAB base workspace.
%   BUSDEFINITION(S,NAME) uses the structure (or array of structures) S and
%   defines buses recursively every time a field of S is another structure.
%   The name convention for the subbuses is to append the field name to the
%   current bus name. eg:
%       
%       CONTROL.PARAM.GRAVITY = 9.81;
%       BUSDEFINITION(CONTROL,'CONTROL_Bus');
%       
%   This will define a CONTROL_Bus and a CONTROL_PARAM_Bus appending the
%   field PARAM to the current bus CONTROL.
%   
%   '_Bus' is added to every bus name. This can be changed by the internal
%   variable 'append'.
%
%   It is conceptually equal to Simulink.Bus.createObject but has more
%   flexibility in naming and avoids possible collisions when two different
%   structures have the same field.

% Check if strcture
if ~isa(structure_in,'struct')
    error('Input to BusDefinition() must be a strcture.');
end

% Bus name appendix
append='_Bus';

% Naming type (regular or recursive)
naming_type=2;

% Fields loop
fieldnames=fields(structure_in);
for nn=1:numel(fieldnames)
    if isa(structure_in.(fieldnames{nn}),'struct')
        
        %Name format for sub buses (TODO: change this allowing for input)
        if naming_type==1
            %Non-recursive naming
            subbus_name=[fieldnames{nn},append];
        else
            %Recursive naming
            if numel(bus_name)>=numel(append) && strcmp(bus_name(end-numel(append)+1:end),append) %erase append of previous layer
                subbus_name=[bus_name(1:end-numel(append)),'_',fieldnames{nn},append];
            else
                subbus_name=[bus_name,'_',fieldnames{nn},append];
            end
        end
        BusDefinition(structure_in.(fieldnames{nn}),subbus_name); %create sub bus
        
        elems(nn) = Simulink.BusElement;
        elems(nn).Name = fieldnames{nn};
        elems(nn).Dimensions = size(structure_in.(fieldnames{nn})); %it could be an array of buses
        elems(nn).DimensionsMode = 'Fixed';
        elems(nn).DataType = ['Bus: ',subbus_name];
        elems(nn).SampleTime = -1;
        elems(nn).Complexity = 'real';
        elems(nn).SamplingMode = 'Sample based';
        elems(nn).Min = [];
        elems(nn).Max = [];
        elems(nn).DocUnits = '';
        elems(nn).Description = '';
                
    else %create elements
        if nn==6
            aaa=1;
        end        
        elems(nn) = Simulink.BusElement;
        elems(nn).Name = fieldnames{nn};
        elems(nn).Dimensions = size(structure_in.(fieldnames{nn}));
        elems(nn).DimensionsMode = 'Fixed';
        elems(nn).DataType = class(structure_in.(fieldnames{nn}));
        if isa(structure_in.(fieldnames{nn}),'logical')
            elems(nn).DataType = 'boolean'; %logical is boolean in Simulink
        end
        elems(nn).SampleTime = -1;
        elems(nn).Complexity = 'real';
        if ~isreal(structure_in.(fieldnames{nn}))
            elems(nn).Complexity = 'complex';
        end
        elems(nn).SamplingMode = 'Sample based';
        elems(nn).Min = [];
        elems(nn).Max = [];
        elems(nn).DocUnits = '';
        elems(nn).Description = '';
        
    end
end

Bus = Simulink.Bus;
Bus.HeaderFile = '';
Bus.Description = '';
Bus.DataScope = 'Auto';
Bus.Alignment = -1;
Bus.Elements = elems;

% check bus previous existance (possible collision with another structure)
if naming_type~=2 %possible collision shouldn't happen if recursive name used
    if evalin('base',['exist(''',bus_name,''',''var'')']) && evalin('base',['isa(',bus_name,',''Simulink.Bus'')'])
            warning(['Redefining Bus ''',bus_name,''' in base workspace. Possible collision with another structure.']);
    end
end
assignin('base',bus_name,Bus)

end

