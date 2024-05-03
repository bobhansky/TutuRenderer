# PathTracing
A CPU based Path Tracer for personal interest and education.

## Features
- Integrator
   - Path Tracing
- Matertial:
   - Lambertain Diffuse (cos weighted)
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
- Acceleration
   - BVH default (midpoint)
   - CPU Multi-Threading (std::thread)   
- Post Processing
   - Bloom
   - HDR
   - Tone Mapping / Gamma Correction
- Russian Roulette (Throughput)

## Todo List
   - Microfacet refractive/translucent
   - BDPT 
   - don't rely on .ppm image file
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
![alt text](https://github.com/bobhansky/PathTracing/blob/main/img/copper_1024.png)
![alt text](https://github.com/bobhansky/PathTracing/blob/main/img/gkass_sphere.png)
![alt text](https://github.com/bobhansky/PathTracing/blob/main/img/glass_bunny.png)
![alt text](https://github.com/bobhansky/PathTracing/blob/main/img/theSpace.png)
![alt text](https://github.com/bobhansky/PathTracing/blob/main/img/space_flaw.png)
![alt text](https://github.com/bobhansky/PathTracing/blob/main/img/MIS-4.png)

<pre>
upperleft: BSDF sampling                                 upperright: light Sampling

lowerleft: NEE only                                      lowerright: NEE + MIS
</pre>



