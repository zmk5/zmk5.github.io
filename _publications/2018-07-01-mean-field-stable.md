---
title: "Mean-Field Stabilization of Markov Chain Models for Robotic Swarms: Computational Approaches and Experimental Results"
collection: publications
category: manuscripts
permalink: /publication/2018-07-01-mean-field-stable
# excerpt: 'This paper is about fixing template issue #693.'
date: 2018-07-01
venue: 'IEEE Robotics and Automation Letters (RA-L)'
slidesurl: 'http://academicpages.github.io/files/slides3.pdf'
paperurl: 'http://academicpages.github.io/files/paper3.pdf'
citation: 'V. Deshmukh, K. Elamvazhuthi, S. Biswal, Z. Kakish and S. Berman, "Mean-Field Stabilization of Markov Chain Models for Robotic Swarms: Computational Approaches and Experimental Results," in IEEE Robotics and Automation Letters, vol. 3, no. 3, pp. 1985-1992, July 2018, doi: 10.1109/LRA.2018.2792696.'
---

***Abstract***: In this letter, we present two computational approaches for synthesizing decentralized density-feedback laws that asymptotically stabilize a strictly positive target equilibrium distribution of a swarm of agents among a set of states. The agents' states evolve according to a continuous-time Markov chain on a bidirected graph, and the density-feedback laws are designed to prevent the agents from switching between states at equilibrium. First, we use classical linear matrix inequality (LMI)-based tools to synthesize linear feedback laws that (locally) exponentially stabilize the desired equilibrium distribution of the corresponding mean-field model. Since these feedback laws violate positivity constraints on the control inputs, we construct rational feedback laws that respect these constraints and have the same stabilizing properties as the original feedback laws. Next, we present a sum-of-squares (SOS)-based approach to constructing polynomial feedback laws that globally stabilize an equilibrium distribution and also satisfy the positivity constraints. We validate the effectiveness of these control laws through numerical simulations with different agent populations and graph sizes and through multirobot experiments on spatial redistribution among four regions.
