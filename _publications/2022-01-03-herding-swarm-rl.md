---
title: "Using Reinforcement Learning to Herd a Robotic Swarm to a Target Distribution"
collection: publications
category: conferences
permalink: /publication/2022-01-03-herding-swarm-rl
# excerpt: 'This paper is about fixing template issue #693.'
date: 2022-01-03
venue: 'Distributed Autonomous Robotic Systems (DARS)'
slidesurl: 'http://academicpages.github.io/files/slides3.pdf'
paperurl: 'http://academicpages.github.io/files/paper3.pdf'
citation: 'Kakish, Z., Elamvazhuthi, K., Berman, S. (2022). Using Reinforcement Learning to Herd a Robotic Swarm to a Target Distribution. In: Matsuno, F., Azuma, Si., Yamamoto, M. (eds) Distributed Autonomous Robotic Systems. DARS 2021. Springer Proceedings in Advanced Robotics, vol 22. Springer, Cham. https://doi.org/10.1007/978-3-030-92790-5_31'
---

***Abstract***: In this paper, we present a reinforcement learning approach to designing a control policy for a “leader” agent that herds a swarm of “follower” agents, via repulsive interactions, as quickly as possible to a target probability distribution over a strongly connected graph. The leader control policy is a function of the swarm distribution, which evolves over time according to a mean-field model in the form of an ordinary difference equation. The dependence of the policy on agent populations at each graph vertex, rather than on individual agent activity, simplifies the observations required by the leader and enables the control strategy to scale with the number of agents. Two Temporal-Difference learning algorithms, SARSA and Q-Learning, are used to generate the leader control policy based on the follower agent distribution and the leader’s location on the graph. A simulation environment corresponding to a grid graph with 4 vertices was used to train and validate the control policies for follower agent populations ranging from 10 to 1000. Finally, the control policies trained on 100 simulated agents were used to successfully redistribute a physical swarm of 10 small robots to a target distribution among 4 spatial regions.
