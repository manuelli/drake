function plotResetBoundary(gammaInDegrees)
  theta_st_grid = linspace(-0.4,-0.2,100);
  gammaRadians = gammaInDegrees/180 * pi;
  theta_sw_grid = -theta_st_grid-2*gammaRadians;
  plot(theta_st_grid, theta_sw_grid)
end