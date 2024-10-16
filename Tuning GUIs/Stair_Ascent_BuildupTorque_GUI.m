function quadratic_fitter_GUI
    % Create a figure for the GUI
    fig = figure('Name', 'Quadratic Fitter', 'NumberTitle', 'off', ...
                 'Position', [100, 100, 800, 500]);
    ax = axes('Parent', fig, 'Position', [0.1, 0.3, 0.6, 0.6]);
    hold(ax, 'on');
    xlim(ax, [0 300]);
    ylim(ax, [0 6]);
    xlabel(ax, 'X');
    ylabel(ax, 'Y');
    title(ax, 'Stair Ascent Torque Build Up Controller');
    
    % Initial point coordinates
    points = [0, 0; 75, 2; 175, 4.5];
    
    % Plot initial points
    hPoints = gobjects(1, 3);
    for i = 1:3
        hPoints(i) = plot(ax, points(i,1), points(i,2), 'ro', 'MarkerSize', 10, ...
                          'ButtonDownFcn', @(src, event) startDragFcn(src, event, i));
    end
    
    % Plot initial quadratic curve
    hCurve = plot(ax, nan, nan, 'b-', 'LineWidth', 1.5);
    
    % Display coefficients on the right side of the graph
    coeffText = uicontrol('Style', 'text', 'Position', [650, 300, 120, 60], ...
                          'FontSize', 12, 'HorizontalAlignment', 'left', ...
                          'String', 'a = ?, b = ?, c = ?');
        
    % Create horizontal input fields for manually entering points at the bottom
    inputFields = gobjects(1, 6);
    labels = ["X1:", "Y1:", "X2:", "Y2:", "X3:", "Y3:"];
    for i = 1:3
        % X coordinate label and input
        uicontrol('Style', 'text', 'Position', [100 + 200*(i-1), 40, 30, 20], ...
                  'String', labels(2*i-1), 'HorizontalAlignment', 'left');
        inputFields(2*i-1) = uicontrol('Style', 'edit', 'Position', [130 + 200*(i-1), 40, 50, 20], ...
                                       'String', num2str(points(i,1)), ...
                                       'Callback', @(src, ~) updatePoint(i, 1, src));
        
        % Y coordinate label and input
        uicontrol('Style', 'text', 'Position', [190 + 200*(i-1), 40, 30, 20], ...
                  'String', labels(2*i), 'HorizontalAlignment', 'left');
        inputFields(2*i) = uicontrol('Style', 'edit', 'Position', [220 + 200*(i-1), 40, 50, 20], ...
                                     'String', num2str(points(i,2)), ...
                                     'Callback', @(src, ~) updatePoint(i, 2, src));
    end

    % Update curve and coefficients on load
    updateQuadratic();
    
    % Nested functions for interactivity
    function startDragFcn(~, ~, idx)
        set(fig, 'WindowButtonMotionFcn', @(~,~) dragPoint(idx));
        set(fig, 'WindowButtonUpFcn', @(~,~) stopDragFcn());
    end
    
    function dragPoint(idx)
        cp = get(ax, 'CurrentPoint');
        points(idx, :) = cp(1, 1:2);
        set(hPoints(idx), 'XData', points(idx, 1), 'YData', points(idx, 2));
        updateInputs();  % Update the input fields with new point values
        updateQuadratic();
    end

    function stopDragFcn()
        set(fig, 'WindowButtonMotionFcn', '');
        set(fig, 'WindowButtonUpFcn', '');
    end
    
    function updatePoint(pointIdx, coordIdx, src)
        % Update point coordinates from input fields
        newValue = str2double(src.String);
        if isnan(newValue)
            src.String = num2str(points(pointIdx, coordIdx));  % Reset if input is invalid
            return;
        end
        points(pointIdx, coordIdx) = newValue;
        set(hPoints(pointIdx), 'XData', points(pointIdx, 1), 'YData', points(pointIdx, 2));
        updateQuadratic();
    end

    function updateInputs()
        % Update the input fields with the current point coordinates
        for i = 1:3
            inputFields(2*i-1).String = num2str(points(i, 1));
            inputFields(2*i).String = num2str(points(i, 2));
        end
    end
    
    function updateQuadratic()
        % Fit a quadratic to the three points
        x = points(:,1);
        y = points(:,2);
        p = polyfit(x, y, 2);  % p(1) = a, p(2) = b, p(3) = c
        
        % Update coefficients display
        coeffText.String = sprintf('a = %.6f\nb = %.6f\nc = %.6f', p(1), p(2), p(3));
        
        % Plot the quadratic curve
        xCurve = linspace(min(x)-1, max(x)+1, 100);
        yCurve = polyval(p, xCurve);
        set(hCurve, 'XData', xCurve, 'YData', yCurve);
    end
end
