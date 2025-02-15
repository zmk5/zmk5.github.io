---
title: "Effectiveness of Warm-Start PPO for Guidance with Highly Constrained Nonlinear Fixed-Wing Dynamics"
collection: publications
category: conferences
permalink: /publication/2023-05-31-warm-start
# excerpt: 'This paper is about fixing template issue #693.'
date: 2023-05-31
venue: 'American Control Conference (ACC)'
slidesurl: 'http://academicpages.github.io/files/slides3.pdf'
paperurl: 'http://academicpages.github.io/files/paper3.pdf'
citation: 'C. T. Coletti, K. A. Williams, H. C. Lehman, Z. M. Kakish, D. Whitten and J. J. Parish, "Effectiveness of Warm-Start PPO for Guidance with Highly Constrained Nonlinear Fixed-Wing Dynamics," 2023 American Control Conference (ACC), San Diego, CA, USA, 2023, pp. 3288-3295, doi: 10.23919/ACC55779.2023.10156267.'
---

***Abstract***: Reinforcement learning (RL) may enable fixedwing unmanned aerial vehicle (UAV) guidance to achieve more agile and complex objectives than typical methods. However, RL has yet struggled to achieve even minimal success on this problem; fixed-wing flight with RL-based guidance has only been demonstrated in literature with reduced state and/or action spaces. In order to achieve full 6-DOF RL-based guidance, this study begins training with imitation learning from classical guidance, a method known as warm-staring (WS), before further training using Proximal Policy Optimization (PPO). We show that warm starting is critical to successful RL performance on this problem. PPO alone achieved a 2% success rate in our experiments. Warm-starting alone achieved 32% success. Warm-starting plus PPO achieved 57% success over all policies, with 40% of policies achieving 94% success.
