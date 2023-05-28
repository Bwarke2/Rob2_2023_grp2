robotCmd = rospublisher("/cmd_vel", "DataFormat","struct");
velMsg = rosmessage(robotCmd);

% Asserting constants and lists
range_history = [];
error_old = 0;

if ismember('/scan',rostopic('list'))
    scansub = rossubscriber('/scan');

    while(1)
        linescan = receive(scansub); % Receive message
        ranges = linescan.Ranges;    % Extract Ranges
        ranges(ranges <= 0.1) = inf; % Remove ranges equal to zero
        range_minimum = min(ranges); % Find minimum distance
        angles = linescan.AngleMin:linescan.AngleIncrement:linescan.AngleMax;
        range_history = [range_history; range_minimum]; % Logging ranges
        
        % Plotting range data
        plot(angles(ranges==range_minimum), ranges(ranges==range_minimum),'o')
        plot(angles, ranges)
        xlabel('Angle [rad]')
        ylabel('Distance [m]')
    end
end
