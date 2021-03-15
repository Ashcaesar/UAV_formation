![](https://img.shields.io/github/downloads/Ashcaesar/UAV_formation/total?label=Downloads)
![](https://img.shields.io/github/license/Ashcaesar/UAV_formation?color=20)
![](https://img.shields.io/github/watchers/Ashcaesar/UAV_formation?style=social)  
C code for UAV formation based on `Cucker-Smale` model.  

_Swarming_
=
Swarming is a collective behaviour exhibited by entities, particularly animals,of similar size which aggregate together, perhaps milling about the same spot or perhaps moving en masse or migrating in some direction.  

The simplest mathematical models of animal swarms generally represent individual animals as following three rules:  
* Move in the same direction as their neighbors  
* Remain close to their neighbors  
* Avoid collisions with their neighbors  
("Swarm behaviour",from Wikipedia)

_Advanced Cucker-Smale model_
=
The whole program is divided into three steps:  
* Assemble  
  UAVs gather and go to the specified starting point at a similar velocity  
* Formation  
  Divide UAVs into several groups based on location coordinates  
* Flight  
  UAVs complete the flight mission following a certain formation  


*Each UAV is regarded as a mass point, the simuulation environment is 3D.*
