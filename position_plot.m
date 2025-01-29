close all
data_ppo = readmatrix('goal_ppo.csv');
cutoff_index = 1227;
data_ppo = data_ppo(1:cutoff_index, :);
% Calculate total traveled distance

total_distance_ppo = 0;
for i = 2:size(data_ppo, 1)
    total_distance_ppo = total_distance_ppo + norm(data_ppo(i, :) - data_ppo(i-1, :));
end

% Plot the trajectory
t1 = plot3(data_ppo(:,1), data_ppo(:,2), data_ppo(:,3), 'b-', 'LineWidth', 3);
hold on;
%%
data = readmatrix('goal_neat.csv');
cutoff_index = 1025;
data = data(3:cutoff_index, :);
total_distance_neat = 0;
for i = 2:size(data, 1)
    total_distance_neat = total_distance_neat + norm(data(i, :) - data(i-1, :));
end

% Plot the trajectory
t2 = plot3(data(:,1), data(:,2), data(:,3), 'k', 'LineWidth', 3);
hold on;

xlabel('X');
ylabel('Z');
zlabel('Y');
grid on;

% Set axes to equal to ensure cubes are not distorted
axis equal;

% Plot cube for start point
start = [0, 0, 0];
plotCube(start, 0.5, 'g', 1); % Green cube of size 1.5
% Function to plot a cube at a specified point
% Plot cubes for checkpoints
checkpoints = [
    4,  3,  4;   % First checkpoint
   -2,  4,  3;   % Second checkpoint
   -3,  2, -2;   % Third checkpoint
    4,  5, -3;   % Fourth checkpoint
];
n_checkpoints = 10; 
for i = 1:size(checkpoints, 1)
    % Calculate alpha value: start more transparent, end less transparent
    alpha_value = i / n_checkpoints;
    plotCube(checkpoints(i,:), 0.5, 'r', alpha_value);
end

% Dummy plot for legend
hStart =fill3(NaN, NaN, NaN, 'g', 'EdgeColor', 'none');
hChpt =fill3(NaN, NaN, NaN, 'r', 'EdgeColor', 'none');
hExtraPPO = plot(NaN, NaN, 'b-', 'LineWidth', 3);  % Invisible, for legend entry
hExtraNEAT = plot(NaN, NaN, 'k-', 'LineWidth', 3);  % Invisible, for legend entry

hold off

legend([t1, t2, hStart, hChpt], sprintf('Trajektória PPO, vzdialenosť: %.2f [m]', total_distance_ppo), sprintf('Trajektória NEAT, vzdialenosť: %.2f [m]', total_distance_neat), 'Štart', 'Checkpoint')
%legend('Trajektória PPO', 'Trajektória NEAT', 'Checkpoint', 'Štart', sprintf('Celková vzdialenosť PPO', total_distance_ppo), sprintf('Celková vzdialenosť NEAT: %.2f units', total_distance_neat));

%%
figure
drone_radius = 0.5;
n_sides = 3
[x, y, z] = tubeplot(data_ppo(:,1:3)', drone_radius, n_sides);
surf(x', y', z', 'EdgeColor', 'b', 'FaceColor', 'non'); % Color the tube red
hold on
[x, y, z] = tubeplot(data(:,1:3)', drone_radius, n_sides);
surf(x', y', z', 'EdgeColor', 'k', 'FaceColor', 'non'); % Color the tube red
for i = 1:size(checkpoints, 1)
    plotCube(checkpoints(i,:), 0.5, 'r', 1); % Red cubes of size 1
end
axis equal;
grid on
hold off
%% Plot angular velocity
figure
%% Plot velocity
figure

subplot(2,1,1)
plot(data_ppo(:, 10), 'b', 'LineWidth', 2)
hold on
plot(data(:, 10), 'k', 'LineWidth', 2)
grid on

xlabel("Krok")
ylabel('$v$ [m/s]', 'Interpreter', 'latex')
title("Lineárna rýchlosť")

legend("ppo", "NEAT")
subplot(2,1,2)

plot(data_ppo(:, 11), 'b', 'LineWidth', 2)
hold on
plot(data(:, 11), 'k', 'LineWidth', 2)
grid on
xlabel("Krok")
ylabel('$\omega \ [\mathrm{rad/s}]$', 'Interpreter', 'latex')
title("Uhlová rýchlosť")
legend("ppo", "NEAT")
%%
figure;
hold on;
axis equal;
grid on;
xlabel('X');
ylabel('Y');
zlabel('Z');

% Define the axes limits
xlim([-1, 1]);
ylim([-1, 1]);
zlim([-1, 1]);

% Draw the global coordinate axes
quiver3(0, 0, 0, 1, 0, 0, 'k', 'LineWidth', 2, 'DisplayName', 'Global X');
quiver3(0, 0, 0, 0, 1, 0, 'k', 'LineWidth', 2, 'DisplayName', 'Global Y');
quiver3(0, 0, 0, 0, 0, 1, 'k', 'LineWidth', 2, 'DisplayName', 'Global Z');

% Sample inclination components
rightY = 1;   % Example value for the y-component of the right vector
upY = 1;      % Example value for the y-component of the up vector
forwardY = 1; % Example value for the y-component of the forward vector

% Draw the inclination vectors
quiver3(0, 0, 0, 1, rightY, 0, 'r', 'LineWidth', 2, 'DisplayName', 'Right Y');
quiver3(0, 0, 0, 0, upY, 1, 'g', 'LineWidth', 2, 'DisplayName', 'Up Y');
quiver3(0, 0, 0, forwardY, 0, 1, 'b', 'LineWidth', 2, 'DisplayName', 'Forward Y');

legend show;
title('3D Visualization of Object Inclination');

%%
function plotCube(center, size, color, alpha)
    S = size / 2;  % Half-size
    x = [1 1 -1 -1  1  1 -1 -1]*S + center(1);
    y = [1 -1 -1 1  1 -1 -1  1]*S + center(2);
    z = [1 1  1  1 -1 -1 -1 -1]*S + center(3);

    % List of vertices for each face of the cube
    faces = [ ...
        1 2 6 5; % +Y face
        2 3 7 6; % -X face
        3 4 8 7; % -Y face
        4 1 5 8; % +X face
        1 2 3 4; % +Z face
        5 6 7 8  % -Z face
    ];
    
    % Draw each face with the provided color and transparency
    for i = 1:6
        h = fill3(x(faces(i,:)), y(faces(i,:)), z(faces(i,:)), color, 'EdgeColor', 'none');
        set(h, 'FaceAlpha', alpha); % Set the transparency
    end
end