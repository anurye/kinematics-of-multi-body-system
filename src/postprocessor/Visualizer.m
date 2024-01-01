function Visualizer(mechanism, option, varargin)
% Visualizer.m: Produces a visual summary of the mechanism kinematics by
% producing a plot for position, velocity, and acceleration for each body.
%
%   Parameter:
%             mechanism: a struct representing the mechanism.
%
%             option: string option used to specify whether Visualizer
%             should plot kinematics of bodies or markers.
%               * Valied options:
%                           1. 'b' or 'B' - for ploting bodies kinematics (i.e.,
%                           of their center of mass)
%                           2. 'm' or 'M' - for ploting markers kinematics
%
%             varargin: Optional argument to specify a particular body or
%             bodies of the the mechanism for which the plot should be
%             generated. It should br cell array.
%              * The maximum number of parameters that can be passed
%              through varargin is two. The first will be taken as name(s)
%              of bodies and the second if provided will be name(s) of
%              markers in that body
%
%
%   Returns:
%           None
%

if nargin > 4
    error("Visualizer: the maximum number of arguments that should be" + ...
        " specified is: %d", 4)
end

if nargin == 4
    if ~iscell(varargin{1}) && ~iscell(varargin{2})
        error("Visualizer: the name of body(s) should be specified as " + ...
            "a cell array: %s, %s", varargin{1}, varargin{2})
    end

    if  strcmp(option, 'm') || strcmp(option, 'M')
        % Pass it to a function, plotSpecified, which is dedicated
        % for plotting kinematics results for specified body/bodies
        plotSpecified(mechanism, option, varargin{1}, varargin{2})
        return
    else
        error("Visualizer: The specified option: %s is wrong." + ...
            "Please specify either 'm' or 'M' for marker.\n", option)
    end

elseif nargin == 3
    if ~iscell(varargin{1})
        error("Visualizer: the name of body(s) should be specified as " + ...
            "a cell array: %s", varargin{1})
    end

    if strcmp(option, 'b') || strcmp(option, 'B') ||...
            strcmp(option, 'm') || strcmp(option, 'M')
        % Pass it to a function, plotSpecified, which is dedicated
        % for plotting kinematics results for specified body/bodies
        plotSpecified(mechanism, option, varargin{1})
        return
    else
        error("Visualizer: The specified option: %s is wrong." + ...
            "Please specify one of (b, B, m, M).\n", option)
    end

elseif nargin == 2
    if strcmp(option, 'b') || strcmp(option, 'B') ||...
            strcmp(option, 'm') || strcmp(option, 'M')
        % Plot kinematics of all bodies in the mechanism
        plotAll(mechanism, option)
        return
    else
        error("Visualizer: The specified option: %s is wrong." + ...
            "Please specify one of (b, B, m, M).\n", option)
    end
else
    error("Visualizer: You are expected to specify at least two " + ...
        "parameters. Mechanism and option (b or m)")
end


end


function plotAll(mechanism, option)
% plotAll: generates plot for the kinematic parameters of all bodies or
% markers, depending on the option, of the mechanism.
%
%   Parameter:
%            mechanism: a struct representing the mechanism.
%
%            option: string option to specify whether to plot for bodies or
%            markers.
%

% Get the list of body names in the mechanism and iterate through each body
bodyNames = fieldnames(mechanism);
for i = 1:numel(bodyNames)
    % Get current body name
    bodyName = bodyNames{i};

    % Check if body name is ground and escape it
    if strcmp(bodyName, "ground") || strcmp(bodyName, "time")
        continue
    end

    % Plot for the current body - bodyName
    Plotter(mechanism, option, bodyName)
end
end


function plotSpecified(mechanism, option, bodies, varargin)
% plotSpecified: plots kinematic solution results for specified bodies
%
%   Parameter:
%          mechanism: a struct representing the mechanism.
%
%          option: string option to specify whether to plot for bodies or
%          markers.
%
%          varargin: used for specifying marker name to generate plot for
%          that marker only.
%
%          bodies: a cell array that specifies a particular body or bodies
%          of the the mechanism for which the plot should be
%          generated.
%

% Plot kinematic parameters for each body iteratively
for i = 1:numel(bodies)
    bodyName = bodies{i};
    % Plot for the current body - bodyName
    if ~isempty(varargin)
        Plotter(mechanism, option, bodyName, varargin{1})
    else
        Plotter(mechanism, option, bodyName)
    end
end
end



function Plotter(mechanism, option, bodyName, varargin)
% Plotter: generates plot of position, velocity and acceleration of a
% specified body of the mechanism using the Graph function.
%
%   Parameter:
%          mechanism: a struct representing the mechanism.
%
%          option: string option to specify whether to plot for bodies or
%          markers.
%
%          varargin: used for specifying marker name to generate plot for
%          that marker only.
%
%          bodyName: a string representing the name of a body in the
%          mechanism
%
%   Returns:
%           None
%

% Get all body names to check if the specified bodyName is indeed part of
% the mechanism
bodyList = bodyOrder(mechanism);

% Check if bodyName is part of the mechanism
if ~ismember(bodyName, bodyList) && ~strcmp(bodyName, "time")
    warning("The specified bodyName: %s is not part of the mechanism", bodyName)
    return
end

if strcmp(option, 'b') || strcmp(option, 'B')
    % Get position
    X = mechanism.(bodyName).q0(1, :);
    Y = mechanism.(bodyName).q0(2, :);
    phi = mechanism.(bodyName).q0(3, :);
    % Get velocity
    Vx = mechanism.(bodyName).dq(1, :);
    Vy = mechanism.(bodyName).dq(2, :);
    dPhi = mechanism.(bodyName).dq(3, :);
    % Get acceleration
    aX = mechanism.(bodyName).ddq(1, :);
    aY = mechanism.(bodyName).ddq(2, :);
    ddPhi = mechanism.(bodyName).ddq(3, :);
    % Get simulation time instants
    try
        time = mechanism.time;
    catch
        warning("There is no time information associated with the " + ...
            "mechanism. \n Setting default time step")
        time = 0:1:length(X);
    end
    % Create a new figure with a title of bodyName
    figure;
    sgtitle(['Body: ', bodyName]);
    % Pass the parameters to the graph function
    Graph(time, X, Y, phi, Vx, Vy, dPhi, ...
        aX, aY, ddPhi)

elseif strcmp(option, 'm') || strcmp(option, 'M')
    % Access the markers on this body
    if isfield(mechanism.(bodyName), 'markers')
        % Get the marker name(s) associated with body: bodyName
        markerNames = fieldnames( mechanism.(bodyName).markers);
        % Check if a specific marker is provided
        if ~isempty(varargin)
            invaliedMarkers = setdiff(varargin{1}, markerNames);
            if ~isempty(invaliedMarkers)
                warning("Marker(s): '%s' not found on body: %s", strjoin(invaliedMarkers, ", "), bodyName);
            end
            valiedMarkers = intersect(varargin{1}, markerNames);
            markers = valiedMarkers;
        else
            markers = markerNames;
        end

        % Iterate through each markers
        for k = 1:numel(markers)
            % Get the current marker name
            markerName = markers{k};
            % Position
            X =  mechanism.(bodyName).markers.(markerName).q(1, :);
            Y =  mechanism.(bodyName).markers.(markerName).q(2, :);
            phi =  mechanism.(bodyName).markers.(markerName).q(3, :);
            % Velocity
            Vx = mechanism.(bodyName).markers.(markerName).dq(1, :);
            Vy = mechanism.(bodyName).markers.(markerName).dq(2, :);
            dPhi = mechanism.(bodyName).markers.(markerName).dq(3, :);
            % Acceleration
            aX = mechanism.(bodyName).markers.(markerName).ddq(1, :);
            aY = mechanism.(bodyName).markers.(markerName).ddq(2, :);
            ddPhi = mechanism.(bodyName).markers.(markerName).ddq(3, :);
            % Get simulation time instants
            try
                time = mechanism.time;
            catch
                warning("There is no time information associated with the " + ...
                    "mechanism. \n Setting default time step")
                time = 0:1:length(X);
            end
            % Create a new figure with a title of bodyName
            figure;
            sgtitle(['Body: ', bodyName, newline, 'Marker: ', markerName]);
            % Pass the parameters to the graph function
            Graph(time, X, Y, phi, Vx, Vy, dPhi, ...
                aX, aY, ddPhi)
        end
    else
        warning("There are no markers associated with body: %s", bodyName)
        return
    end
end

end


function Graph(time, X, Y, phi, Vx, Vy, dPhi, ...
    aX, aY, ddPhi)

% Position
subplot(3, 1, 1);
plot(time, X, "LineWidth", 1.3, "DisplayName", "q_x");
hold on;
plot(time, Y, "LineWidth", 1.3, "DisplayName", "q_y");
plot(time, phi, "LineWidth", 1.3, "DisplayName", "phi");
title("Position");
xlabel("Time [s]")
ylabel("q_x [m],  q_y [m/s],  phi [rad]")
axis padded
grid on
legend("Location", "best");

% Velocity
subplot(3, 1, 2);
plot(time, Vx, "LineWidth", 1.3, "DisplayName", "dq_x");
hold on;
plot(time, Vy, "LineWidth", 1.3, "DisplayName", "dq_y");
plot(time, dPhi, "LineWidth", 1.3, "DisplayName", "d-phi");
title("Velocity");
xlabel("Time [s]")
ylabel("dq_x [m/s],  dq_y [m/s],  d-phi [rad/s]")
axis padded
grid on
legend("Location", "best");

% Acceleration
subplot(3, 1, 3);
plot(time, aX, "LineWidth", 1.3, "DisplayName", "ddq_x");
hold on;
plot(time, aY, "LineWidth", 1.3, "DisplayName", "ddq_y");
plot(time, ddPhi, "LineWidth", 1.3, "DisplayName", "dd-phi");
title("Acceleration");
xlabel("Time [s]")
ylabel("ddq_x [m/s^2],  ddq_y [m/s^2],  dd-phi [rad/s^2]")
axis padded
grid on
legend("Location", "best");

hold off;

end
