close all; % Ferme toutes les figures ouvertes
global realpos;
global t;
endtime = 100;
time = linspace(1,endtime, 1000); % Génère un vecteur de temps linéaire entre 1 et endtime avec 1000 points
realpos = sin(time/5); % Génère une position réelle en fonction du temps (sinus)
plot(time, realpos); % Trace le graphique de la position réelle en fonction du temps
hold on;

% The estimated noises of our process (AKA odometry, control), and measurement
processNoise     = 0.1;
measurementNoise = 0.1; 

% Initialisation de la position actuelle, du bruit de processus et de l'odométrie non filtrée
x = realpos(1);
p = processNoise;
unfilteredOdometry = x;
for t = 1:length(time)

  %-------------------- "Prediction Step" --------------------
  u = takeOdometry(); % Récupération de la mesure de l'odométrie
  x = x + u; % Met à jour la position estimée
  p = p + processNoise; % Met à jour le bruit de processus

  %-------------------- "Update Step" --------------------
  if mod(t, 50) == 0
    z = takeMeasurement(); % Récupère une mesure
    plot(time(t), z, 'r*'); % Trace la mesure sur le graphique
    alpha=0.9
    x = alpha*z+(1-alpha)*x  % Mise à jour de la position estimée en combinant la mesure et l'estimation actuelle                 

  end

  plot(time(t), x, 'k.');  % Trace la position estimée sur le graphique (en noir)

  % Plot the 'unfiltered odometry' in green: i.e. what we would see if we 
  % just tried to integrate the raw odometry readings without the kalman filter.
  unfilteredOdometry = unfilteredOdometry + u;
  plot(time(t), unfilteredOdometry, 'go');

  pause(0.05);
end
