# PathTracing
A CPU based Path Tracer for personal interest and education.

## Features
- Integrator
   - Path Tracing
   - BDPT (**Todo**)
- Matertial:
   - Lambertain Diffuse
   - Microfacet Reflectance (Cook Torrance Model with GGX distribution).
   - Perfect Reflective
   - Perfect Refractive.
- Sampling
   - Importance Sampling      
   - Multiple Importance Sampling
   - Next Event Estimation
- Texture Mapping
   - Albedo/Diffuse Map
   - Normal Map
   - Roughness Map
   - Metallicness Map
- Post Processing
   - Bloom
   - HDR
   - Tone Mapping / Gamma Correction
- Russian Roulette (Throughput)
## Usage
   Develop Environment: MSVC C++14, Visual Studio 2022

   cmakelist or makefile of the latest version is **TO-DO** in the future
   
   Refer to https://github.com/bobhansky/WhittedStyle_Raytracer/blob/main/README.md for more instruction on configuration file.




# Sample Result
### Cornell Box with 512 samples per pixel    
![alt text](https://github.com/bobhansky/PathTracing/blob/main/img/spp512_1900sec.png)

-----------------------------
### Contrast  

left: Whitted style Ray Tracing (shading with Blinn Phong Model)  
right: Path Tracing  
![alt text](https://github.com/bobhansky/PathTracing/blob/main/img/contrast.png)

----------------------------
### others  
![alt text](https://github.com/bobhansky/PathTracing/blob/main/img/junge.png)
![alt text](https://github.com/bobhansky/PathTracing/blob/main/img/rust.png)
![alt text](https://github.com/bobhansky/PathTracing/blob/main/img/copper_1600_512spp.png)
![alt text](https://github.com/bobhansky/PathTracing/blob/main/img/gkass_sphere.png)
![alt text](https://github.com/bobhansky/PathTracing/blob/main/img/glass_bunny.png)
![alt text](https://github.com/bobhansky/PathTracing/blob/main/img/space.png)
![alt text](https://github.com/bobhansky/PathTracing/blob/main/img/wall.png)  
![alt text](https://github.com/bobhansky/PathTracing/blob/main/img/256_wall.png)


