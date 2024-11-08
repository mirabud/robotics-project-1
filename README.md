<!-- PROJECT LOGO -->
<div align="left">
    <img src="docs/media/banner.png" alt="Banner" >
</div>
 
 # robotics-project-1

<a id="readme-top"></a>


  
 

<h3 align="left">Controlling a robot in an evironment with ROS2 and Gazebo Ignition</h3>



<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
      <ul>
        <li><a href="#built-with">Built With</a></li>
      </ul>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="#usage">Usage</a></li>
    <li><a href="#contact">Contact</a></li>
  </ol>
</details>



<!-- ABOUT THE PROJECT -->
## About The Project

In this project, we aim to develop a computer vision system to estimate the number of people present on a beach, a task commonly known as "crowd counting." Lifeguards typically monitor beaches during summer, gathering data on occupancy, sea conditions, wind, and more. By applying image processing techniques, we can automate the counting of beachgoers, reducing manual efforts and improving monitoring accuracy. This project will encompass all key stages of an image processing workflow, including data annotation, algorithm design and implementation, and result validation.




### Built With
* [![Python 3][Python-badge]][Python-url]
* [![OpenCV][OpenCV-badge]][OpenCV-url]






<!-- GETTING STARTED -->
## Getting Started

This is an example of how you may give instructions on setting up your project locally.
To get a local copy up and running follow these simple example steps.

### Prerequisites

First we have to set up a conda environment with all the necessary packages:

* Install conda environment with all requirements
  ```sh
    conda create --name Project1 --file requirements.txt
  ```
* If necessary install this version with pip inside the conda env
  ```sh
    pip install opencv-contrib-python==4.5.5.64
  ```
* Activate conda environment
  ```sh
    conda activate Project1
  ```

### Installation

1. Get a free API Key at [https://example.com](https://example.com)
2. Clone the repo
   ```sh
   git clone https://github.com/github_username/repo_name.git
   ```
3. Install NPM packages
   ```sh
   npm install
   ```
4. Enter your API in `config.js`
   ```js
   const API_KEY = 'ENTER YOUR API';
   ```
5. Change git remote url to avoid accidental pushes to base project
   ```sh
   git remote set-url origin github_username/repo_name
   git remote -v # confirm the changes
   ```




<!-- USAGE EXAMPLES -->
## Usage

Use this space to show useful examples of how a project can be used. Additional screenshots, code examples and demos work well in this space. You may also link to more resources.



<!-- CONTACT -->


<!-- MARKDOWN LINKS & IMAGES -->
[OpenCV-badge]: https://img.shields.io/badge/OpenCV-5C3EE8?style=for-the-badge&logo=opencv&logoColor=white
[OpenCV-url]: https://opencv.org/
[Python-badge]: https://img.shields.io/badge/Python-3.8%2B-3776AB?style=for-the-badge&logo=python&logoColor=white
[Python-url]: https://www.python.org/

# Participants
- [Mira Budenova](https://github.com/mirabud)
- [Alejandro Cedillo Gámez](https://github.com/alexcega)
