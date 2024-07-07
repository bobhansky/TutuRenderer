
A CPU based offline Renderer for personal interest and education.

Last Update: 7/4/2024
<img src="https://github.com/bobhansky/PathTracing/blob/main/img/broom_spp512.png" width=900 height=506/>


## Features
- Integrator
   - Path Tracing
   - Light Tracing
   - Bidirectional Path Tracing (BDPT)
        <pre>
        Stick closely to veach97. Learning and Coding refer a lot to 
        https://rendering-memo.blogspot.com/2016/03/bidirectional-path-tracing-1-kickoff.html
         
        0. No t == 0 case 
        1. Shading normal asymmetric Bsdf correction ✅
        2. Some problem with Microfacet Transmissive material (when path length <= 5 is fine) ❌ TODO
        </pre>
- Matertial:
   - Lambertain Diffuse (cos weighted)
   - Microfacet Reflection and Transmittance (Cook Torrance Model with GGX distribution).
   - Perfect Reflective
   - Perfect Refractive.
   - Unlit. (incompatible with bdpt)
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
- Russian Roulette (Throughput) in path tracing

## Todo List
   - Microfacet refractive/translucent  ✅
   - BDPT ✅
   - don't rely on .ppm image file
   - 6/29/2024:
     
         1.better config file
     
         2.image based lighting
     
         3.Subsurface scattering

         4.SAH BVH
## Usage
   <pre>
      $ ./PathTracing.exe config.txt
   </pre>

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
<p float="left">
   
<img src="https://github.com/bobhansky/PathTracing/blob/main/img/junge.png" width=400 height=400/> <img src="https://github.com/bobhansky/PathTracing/blob/main/img/rust.png" width=400 height=400/>

<img src="https://github.com/bobhansky/PathTracing/blob/main/img/copper_1024.png" width=400 height=400/>  <img src="https://github.com/bobhansky/PathTracing/blob/main/img/glass_bunny_bdpt_spp256.png" width=400 height=400/> 

<img src="https://github.com/bobhansky/PathTracing/blob/main/img/glassSphere_512.png" width=400 height=400/>  <img src="https://github.com/bobhansky/PathTracing/blob/main/img/roughGlass_512.png" width=400 height=400/> 

<img src="https://github.com/bobhansky/PathTracing/blob/main/img/theSpace.png" width=400 height=400/>  <img src="https://github.com/bobhansky/PathTracing/blob/main/img/space_flaw.png" width=400 height=400/> 

<img src="https://github.com/bobhansky/PathTracing/blob/main/img/radio.png" width=400 height=400/>  <img src="https://github.com/bobhansky/PathTracing/blob/main/img/buddha.png" width=400 height=400/> 

</p>


![alt text](https://github.com/bobhansky/PathTracing/blob/main/img/MIS-4.png)

<pre>
upperleft: BSDF sampling                                 upperright: light Sampling

lowerleft: NEE only                                      lowerright: NEE + MIS
</pre>


<img src="https://github.com/bobhansky/TutuRenderer/blob/main/img/veach_bdpt_spp512.png" width=400 height=400/>  <img src="https://github.com/bobhansky/TutuRenderer/blob/main/img/veach_bdpt_front_spp1024.png" width=600 height=400/> 

