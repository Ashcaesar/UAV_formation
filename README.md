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
[(Back to top)](#table-of-contents)  
To use this project, first clone the repo on your device using the command below:

```git init```

```git clone https://github.com/Ashcaesar/UAV_formation.git```

# Usage
[(Back to top)](#table-of-contents)

To be filled.

# Development
[(Back to top)](#table-of-contents)  
Advanced Cucker-Smale model.  

The whole program is divided into three steps:  
* Assemble  
  UAVs gather and go to the specified starting point at a similar velocity  
  
* Formation  
  Divide UAVs into several groups based on location coordinates  
  
* Flight  
  UAVs complete the flight mission following a certain formation  
  
*Each UAV is regarded as a mass point, the simuulation environment is 3D.*

Check out [this wiki](https://github.com/navendu-pottekkat/nsfw-filter/wiki) for inspiration.

# Contribute
[(Back to top)](#table-of-contents)

This is where you can let people know how they can **contribute** to your project. Some of the ways are given below.

Also this shows how you can add subsections within a section.

### Sponsor
[(Back to top)](#table-of-contents)

Your project is gaining traction and it is being used by thousands of people(***with this README there will be even more***). Now it would be a good time to look for people or organisations to sponsor your project. This could be because you are not generating any revenue from your project and you require money for keeping the project alive.

You could add how people can sponsor your project in this section. Add your patreon or GitHub sponsor link here for easy access.

A good idea is to also display the sponsors with their organisation logos or badges to show them your love!(*Someday I will get a sponsor and I can show my love*)

### Adding new features or fixing bugs
[(Back to top)](#table-of-contents)

This is to give people an idea how they can raise issues or feature requests in your projects. 

You could also give guidelines for submitting and issue or a pull request to your project.

Personally and by standard, you should use a [issue template](https://github.com/navendu-pottekkat/nsfw-filter/blob/master/ISSUE_TEMPLATE.md) and a [pull request template](https://github.com/navendu-pottekkat/nsfw-filter/blob/master/PULL_REQ_TEMPLATE.md)(click for examples) so that when a user opens a new issue they could easily format it as per your project guidelines.

You could also add contact details for people to get in touch with you regarding your project.

# License
[(Back to top)](#table-of-contents)

Adding the license to README is a good practice so that people can easily refer to it.

Make sure you have added a LICENSE file in your project folder. **Shortcut:** Click add new file in your root of your repo in GitHub --> Set file name to LICENSE --> GitHub shows LICENSE templates ---> Choose the one that best suits your project!

I personally add the name of the license and provide a link to it like below.

[GNU General Public License version 3](https://opensource.org/licenses/GPL-3.0)

# Footer
[(Back to top)](#table-of-contents)

![Footer](https://github.com/Ashcaesar/UAV_formation/blob/master/fooooooter.png)
