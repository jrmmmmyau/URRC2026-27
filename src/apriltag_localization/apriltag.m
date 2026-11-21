data = readmatrix("tag_pose.csv");
t = data(:,1) + data(:,2)*1e-9;
t = t-t(1);
x = data(:,4);
y = data(:,5);
z = data(:,6);

figure
surface([x(:)'; x(:)'], [y(:)'; y(:)'], [z(:)'; z(:)'], ...
        [t(:)'; t(:)'], 'EdgeColor','interp', ...
        'FaceColor','none', 'LineWidth',2)
clim([0 max(t)])
xlabel("Position x")
ylabel("Position y")
zlabel("Position z")
colorbar
view(3)
grid on