---
title: "Controllability and Stabilization for Herding a Robotic Swarm Using a Leader: A Mean-Field Approach"
collection: publications
category: manuscripts
permalink: /publication/2021-04-01-herding-mean-field
# excerpt: 'This paper is about fixing template issue #693.'
date: 2021-04-01
venue: 'IEEE Transactions on Robotics (T-RO)'
slidesurl: 'http://academicpages.github.io/files/slides3.pdf'
paperurl: 'http://academicpages.github.io/files/paper3.pdf'
citation: 'K. Elamvazhuthi, Z. Kakish, A. Shirsat and S. Berman, "Controllability and Stabilization for Herding a Robotic Swarm Using a Leader: A Mean-Field Approach," in IEEE Transactions on Robotics, vol. 37, no. 2, pp. 418-432, April 2021, doi: 10.1109/TRO.2020.3031237.'
---

***Abstract***: In this article, we introduce a model and a control approach for herding a swarm of “follower” agents to a target distribution among a set of states using a single “leader” agent. The follower agents evolve on a finite state space that is represented by a graph and transition between states according to a continuous-time Markov chain (CTMC), whose transition rates are determined by the location of the leader agent. The control problem is to define a sequence of states for the leader agent that steers the probability density of the forward equation of the Markov chain. For the case, when the followers are possibly interacting, we prove local approximate controllability of the system about equilibrium probability distributions. For the case, when the followers are noninteracting, we design two switching control laws for the leader that drive the swarm of follower agents asymptotically to a target probability distribution that is positive for all states. The first strategy is open-loop in nature, and the switching times of the leader are independent of the follower distribution. The second strategy is of feedback type, and the switching times of the leader are functions of the follower density in the leader's current state. We validate our control approach through numerical simulations with varied numbers of follower agents that evolve on graphs of different sizes, through a 3-D multirobot simulation in which a quadrotor is used to control the spatial distribution of eight ground robots over four regions, and through a physical experiment in which a swarm of ten robots is herded by a virtual leader over four regions.
