![space](https://github.com/bobhansky/PathTracing/assets/59004766/8a6571a5-6719-44df-b30a-fd628b0de355)# PathTracing

## Usage

1. After getting the executable PathTracer.exe, move it under the Source folder "PathTracing"
   ![alt text](https://github.com/bobhansky/PathTracing/blob/main/img/s1.png)
-----------------------
   ![alt text](https://github.com/bobhansky/PathTracing/blob/main/img/s2.png)  

-----------------------
3. Run it with the configuration file as the first command line argument:
    ![alt text](https://github.com/bobhansky/PathTracing/blob/main/img/s3.png)

   refer to https://github.com/bobhansky/WhittedStyle_Raytracer/blob/main/README.md for more instruction on configuration file.

-----------------------
4. Then a .ppm output file will be generated under the same folder.

### Note 
edit 
>int SPP = 1;



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
![alt text](https://github.com/bobhansky/PathTracing/blob/main/img/2glossy.png)
![alt text](https://github.com/bobhansky/PathTracing/blob/main/img/mirrorBall1600.png)
![alt text](https://github.com/bobhansky/PathTracing/blob/main/img/copper_1600_512spp.png)
![alt text](https://github.com/bobhansky/PathTracing/blob/main/img/gkass_sphere.png)
![alt text](https://github.com/bobhansky/PathTracing/blob/main/img/glass_bunny.png)
![alt text](https://github.com/bobhansky/PathTracing/blob/main/img/space.png)
![alt text](https://github.com/bobhansky/PathTracing/blob/main/img/space_512_bloomHDR.png)
![alt text](https://github.com/bobhansky/PathTracing/blob/main/img/wall.png)  
![alt text](https://github.com/bobhansky/PathTracing/blob/main/img/256_wall.png)


