# Matlab implementation of adaptive particle filter (AMCL)

<p align="center">
  <img width="752" height="663" src="https://github.com/LonghaoQian/ParticleFilter/blob/master/gifoutput/acm_output.gif">
</p>
<div align="center">
  Figure 1 Robot estimated position and particle position
</div>

## System model

## Dataset
- `dataset/dataset2.mat`: robot odometry and lidar measurements.
- `dataset/AMCL.mat`: AMCL particle filter result.
## AMCL
### Reference:
- [1] M. S. Arulampalam, S. Maskell, N. Gordon and T. Clapp, "A tutorial on particle filters for online nonlinear/non-Gaussian Bayesian tracking," in IEEE Transactions on Signal Processing, vol. 50, no. 2, pp. 174-188, Feb. 2002, doi: 10.1109/78.978374.
- [2] Lang, H., Li, T., Villarrubia, G., Sun, S., Bajo, J. (2015). An Adaptive Particle Filter for Indoor Robot Localization. In: Mohamed, A., Novais, P., Pereira, A., Villarrubia González, G., Fernández-Caballero, A. (eds) Ambient Intelligence - Software and Applications. Advances in Intelligent Systems and Computing, vol 376. Springer, Cham. https://doi.org/10.1007/978-3-319-19695-4_5
