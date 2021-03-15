![Banner](https://github.com/Ashcaesar/UAV_formation/blob/master/header.png)  
# UAV Formation Flight  
![GitHub release (latest by date including pre-releases)](https://img.shields.io/github/v/release/Ashcaesar/UAV_formation?include_prereleases)
![GitHub downloads](https://img.shields.io/github/downloads/Ashcaesar/UAV_formation/total?label=Downloads)
![GitHub last commit](https://img.shields.io/github/last-commit/Ashcaesar/UAV_formation)
![GitHub](https://img.shields.io/github/license/Ashcaesar/UAV_formation)
![GitHub watchers](https://img.shields.io/github/watchers/Ashcaesar/UAV_formation?style=social)  

C code for UAV formation based on `Cucker-Smale` model.  

# Demo-Preview
Swarming is a collective behaviour exhibited by entities, particularly animals,of similar size which aggregate together, perhaps milling about the same spot or perhaps moving en masse or migrating in some direction.  

The simplest mathematical models of animal swarms generally represent individual animals as __following three rules:__  
* Move in the same direction as their neighbors  
* Remain close to their neighbors  
* Avoid collisions with their neighbors  
("Swarm behaviour",from Wikipedia)

Use the coordinate(data is stored in coordinate.txt) of each UAV to simulate the process  
  
Here's a gif made by Matlab(It's actually 3D, but 2D animate looks more clearly)  

![Flock GIF](https://github.com/Ashcaesar/UAV_formation/blob/master/demo.gif)

# Table of contents  
- [UAV Formation Flight](#uav-formation-flight)
- [Demo-Preview](#demo-preview)
- [Table of contents](#table-of-contents)
- [Installation](#installation)
- [Usage](#usage)
- [Development](#development)
- [Contribute](#contribute)
    - [Sponsor](#sponsor)
    - [Adding new features or fixing bugs](#adding-new-features-or-fixing-bugs)
- [License](#license)
- [Footer](#footer)

# Installation
[(Back to top)](#uav-formation-flight)  
To use this project, first clone the repo on your device using the command below:

```git init```

```git clone https://github.com/Ashcaesar/UAV_formation.git```

# Usage
[(Back to top)](#uav-formation-flight)

To be filled.

# Development
[(Back to top)](#uav-formation-flight)  
Advanced Cucker-Smale model.  

The whole program is divided into three steps:  
* Assemble  
  UAVs gather and go to the specified starting point at a similar velocity  
  
* Formation  
  Divide UAVs into several groups based on location coordinates  
  
* Flight  
  UAVs complete the flight mission following a certain formation  
  
*Each UAV is regarded as a mass point, the simuulation environment is 3D.*

For more information, check out [this wiki](https://github.com/Ashcaesar/UAV_formation/wiki) for inspiration.

# Contribute
[(Back to top)](#uav-formation-flight)

If you have contributions, need support, have suggestions, or just want to get in touch with me, just email me!  

3170102203@zju.edu.cn

# License
[(Back to top)](#uav-formation-flight)

[GNU General Public License version 3](https://opensource.org/licenses/GPL-3.0)

# Footer
[(Back to top)](#uav-formation-flight)

![Footer](https://github.com/Ashcaesar/UAV_formation/blob/master/fooooooter.png)
