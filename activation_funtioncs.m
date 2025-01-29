% subplot(3, 1, 1)
% x = -5:0.1:5;
% plot(x, max(-1, min(1, x)), 'k', 'LineWidth', 2)
% title("Clamped")
% xlabel("Vstup")
% ylabel("Výstup")
% grid on
% ylim([-1.5 1.5])
% subplot(3, 1, 2)
% plot(x, tanh(x), 'k', 'LineWidth', 2)
% hold on
% plot(x, tanh(3*x), 'b.', 'LineWidth', 2)
% plot(x, tanh(5*x), 'r-.', 'LineWidth', 2)
% hold off
% legend(["tanh(x)", "tanh(3x)", "tanh(5x)"], "Interpreter","latex")
% title("Hyperbolický tangens")
% xlabel("Vstup")
% ylabel("Výstup")
% grid on
% ylim([-1.5 1.5])
% subplot(3, 1, 3)
% plot(x, 1./(1+exp(-x)), 'k', 'LineWidth', 2)
% title("Sigmoid")
% xlabel("Vstup")
% ylabel("Výstup")
% grid on
% ylim([-1.5 1.5])

% Plot the histogram and PDF together
% sigma = 0.5; % Standard deviation
% mu = 0.0; % Mean
% num_samples = 1000; % Number of random samples
% 
% % Generating the random numbers
% randomNumbers = mu + sigma * randn(num_samples, 1);
% 
% % Define the range of x values
% x = [-2:0.01:2];
% % Calculate the PDF
% y = normpdf(x, mu, sigma);
% 
% % Create a new figure window
% figure;
% 
% % Plot histogram of random numbers
% histogram(randomNumbers, 30, 'Normalization', 'pdf'); % Normalize as probability density
% hold on; % Hold on to plot PDF on top
% 
% % Plot the normal distribution PDF
% plot(x, y, 'LineWidth', 2);
% 
% % Adding labels and title
% xlabel('Hodnota');
% ylabel('Hustota pravdepodobnosti');
% title('Histogram a PDF náhodných čísel s gaussianským rozdelením');
% 
% % Adding legend
% legend('Histogram', 'Normal PDF');
% 
% hold off; % Release the hold
% 



% Define coefficients
coefficients = [1, 3, 10];

% Define the function using an anonymous function
f = @(error, coeff) min(1, -2 / (1 + exp(-error * coeff)) + 2);

% Generate a range of error values
error_values = linspace(-10, 10, 400);

% Create a new figure window
figure;

% Hold on to add multiple plots to the same figure
hold on;

% Define line styles
lineStyles = {'--', ':', '-'};

% Loop through each coefficient and plot
for i = 1:length(coefficients)
    % Compute the function values for each coefficient
    y_values = arrayfun(@(x) f(x, coefficients(i)), error_values);
    
    % Plot the function with specified line style
    plot(error_values, y_values, 'DisplayName', sprintf('Coefficient = %d', coefficients(i)), 'LineStyle', lineStyles{i}, 'LineWidth', 2, 'Color', 'k');
end

% Title and labels
title('Reward function');
xlabel('Input Value');
ylabel('Reward');

% Add grid
grid on;

% Enable the legend
legend show;

% Hold off to end adding multiple plots
hold off;

% Define coefficients
coefficients = [0.5, 1];

% Define the function using an anonymous function
f = @(value, coeff) 2 / (1 + exp(-value * coeff)) - 1;

% Generate a range of value
value_range = linspace(-10, 10, 400);

% Create a new figure window
figure;

% Hold on to add multiple plots to the same figure
hold on;

% Define line styles
lineStyles = {'--', '-', ':'};

% Loop through each coefficient and plot
for i = 1:length(coefficients)
    % Compute the function values for each coefficient
    y_values = arrayfun(@(x) f(x, coefficients(i)), value_range);
    
    % Plot the function with specified line style
    plot(value_range, y_values, 'DisplayName', sprintf('Coefficient = %.2f', coefficients(i)), 'LineStyle', lineStyles{i}, 'LineWidth', 2, 'Color', 'k');
end

% Title and labels
title('Shifted Sigmoid Function');
xlabel('Input');
ylabel('Output');

% Add grid
grid on;

% Enable the legend
legend show;

% Hold off to end adding multiple plots
hold off;

%%

value_range = linspace(-3, 3, 400);
v = 2;
d = max(0, min(value_range, v));
y = (1 - (d/v).^2).^2;
plot(value_range, y, 'k', 'LineWidth', 2)
grid on
% Title and labels
xlabel('Error');
ylabel('Reward');
title("Reward function for maintaining the desired value")
legend("Požadovaná hodnota = 2")

%% 
%% % Define parameters
% Parameters
target = 1; % Target threshold value (adjust as needed)
delta = linspace(-3, target, 1000); % Velocity difference range [0, target]

% Calculate reward
reward = (1 - (min(delta, target)/target).^2).^2;

% Plotting
% figure('Color', 'white');
plot(delta, reward, 'k', 'LineWidth', 2);
title('Reward for Maintaining Target Value (t=1)');
xlabel('Difference ($\|\vec{v}_a - \vec{v}_g\|$)','Interpreter','latex');
ylabel('Reward');
xlim([0 target]);
ylim([0 1.1]);
grid on;
