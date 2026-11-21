function y = plant_table(x_table, y_table, x)
% PLANT_TABLE  Linear interpolation that holds the end values outside the table (the mod's lerp_table, cpp:47).
x_clamped = min(max(x, x_table(1)), x_table(end));
y = interp1(x_table, y_table, x_clamped, 'linear');
end
