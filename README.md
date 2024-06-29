# PathTracing
A CPU based Path Tracer for personal interest and education.

## Features
- Integrator
   - Path Tracing
   - Light Tracing
   - Bidirectional Path Tracing (BDPT)
- Matertial:
   - Lambertain Diffuse (cos weighted)
   - Microfacet Reflection and Transmittance (Cook Torrance Model with GGX distribution).
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
   - Microfacet refractive/translucent  ✅
   - BDPT ✅
   - don't rely on .ppm image file
## Usage
   Develop Environment: MSVC C++17, Visual Studio 2022

   cmakelist or makefile of the latest version is **TO-DO** in the future
   
   Refer to https://github.com/bobhansky/WhittedStyle_Raytracer/blob/main/README.md for more instruction on configuration file.




# Sample Result
### Cornell Box with 512 samples per pixel    
<img src="https://github.com/bobhansky/PathTracing/blob/main/img/spp512_1900sec.png" width=500 height=500/> 


-----------------------------
### Contrast  

left: Whitted style Ray Tracing (shading with Blinn Phong Model)  
right: Path Tracing  
![alt text](https://github.com/bobhansky/PathTracing/blob/main/img/contrast.png)

----------------------------
### others  
<img src="https://github.com/bobhansky/PathTracing/blob/main/img/junge.png" width=500 height=500/> <img src="https://github.com/bobhansky/PathTracing/blob/main/img/rust.png" width=500 height=500/>

<img src="https://github.com/bobhansky/PathTracing/blob/main/img/copper_1024.png" width=500 height=500/> 

<img src="https://github.com/bobhansky/PathTracing/blob/main/img/glassSphere_512.png" width=500 height=500/>  <img src="https://github.com/bobhansky/PathTracing/blob/main/img/roughGlass_512.png" width=500 height=500/> 

<img src="https://github.com/bobhansky/PathTracing/blob/main/img/glass_bunny.png" width=500 height=500/>   <img src="https://github.com/bobhansky/PathTracing/blob/main/img/glass_bunny_bdpt_spp256.png" width=500 height=500/> 

<img src="https://github.com/bobhansky/PathTracing/blob/main/img/theSpace.png" width=500 height=500/>  <img src="https://github.com/bobhansky/PathTracing/blob/main/img/space_flaw.png" width=500 height=500/> 
<img src="https://github.com/bobhansky/PathTracing/blob/main/img/radio.png" width=500 height=500/>  <img src="https://github.com/bobhansky/PathTracing/blob/main/img/buddha.png" width=500 height=500/> 
![alt text](https://github.com/bobhansky/PathTracing/blob/main/img/MIS-4.png)

<pre>
upperleft: BSDF sampling                                 upperright: light Sampling

lowerleft: NEE only                                      lowerright: NEE + MIS
</pre>


<img src="https://github.com/bobhansky/PathTracing/blob/main/img/veach_bdpt_spp512_v1.2.png" width=500 height=500/> 
<pre>
↑ render with bidirection path tracing
</pre>

