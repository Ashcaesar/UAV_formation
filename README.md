C code for UAV formation based on Cucker-Smale model.  

_Swarming_
=
Swarming is a collective behaviour exhibited by entities, particularly animals,of similar size which aggregate together, perhaps milling about the same spot or perhaps moving en masse or migrating in some direction.  

The simplest mathematical models of animal swarms generally represent individual animals as following three rules:  
1.Move in the same direction as their neighbors  
2.Remain close to their neighbors  
3.Avoid collisions with their neighbors  
("Swarm behaviour",from Wikipedia)

_Advanced Cucker-Smale model_
=
The whole program is divided into three steps:  
1.Assemble  
  UAVs gather and go to the specified starting point at a similar velocity  
2.Formation  
  Divide UAVs into several groups based on location coordinates  
3.Flight  
  UAVs complete the flight mission following a certain formation  


*Each UAV is regarded as a mass point, the simuulation environment is 3D.*
