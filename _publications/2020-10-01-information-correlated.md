---
title: "Information Correlated Lévy Walk Exploration and Distributed Mapping Using a Swarm of Robots"
collection: publications
category: manuscripts
permalink: /publication/2020-10-01-information-correlated
# excerpt: 'This paper is about fixing template issue #693.'
date: 2020-10-01
venue: 'IEEE Transactions on Robotics (T-RO)'
slidesurl: 'http://academicpages.github.io/files/slides3.pdf'
paperurl: 'http://academicpages.github.io/files/paper3.pdf'
citation: 'R. K. Ramachandran, Z. Kakish and S. Berman, "Information Correlated Lévy Walk Exploration and Distributed Mapping Using a Swarm of Robots," in IEEE Transactions on Robotics, vol. 36, no. 5, pp. 1422-1441, Oct. 2020, doi: 10.1109/TRO.2020.2991612.'
---

***Abstract***: In this article, we present a novel distributed method for constructing an occupancy grid map of an unknown environment using a swarm of robots with global localization capabilities and limited interrobot communication. The robots explore the domain by performing Lévy walks in which their headings are defined by maximizing the mutual information between the robot's estimate of its environment in the form of an occupancy grid map and the distance measurements that it is likely to obtain when it moves in that direction. Each robot is equipped with laser range sensors, and it builds its occupancy grid map by repeatedly combining its own distance measurements with map information that is broadcast by neighboring robots. Using results on average consensus over time-varying graph topologies, we prove that all robots’ maps will eventually converge to the actual map of the environment. In addition, we demonstrate that a technique based on topological data analysis, developed in our previous work for generating topological maps, can be readily extended for adaptive thresholding of occupancy grid maps. We validate the effectiveness of our distributed exploration and mapping strategy through a series of two-dimensional simulations and multirobot experiments.
