---
layout: post
title: "Part4: Supervised Learning: Weighted regression"
date: 2020-04-11 012:15:00-0400
description: Building blocks of supervised learning
categories: Machine-Learning
enable_math: true
---

<!-- <div style="text-align: justify"> <a href="https://www.ros.org/"> ROS </a> --> <!-- </div> -->
`What happens if the data is non-linear?`


<!-- \section{Locally weighted linear regression} -->
If the data points are non-linear in nature, performing a linear regression does not make sense. When we require the output(**Y**) at a particular input point ($$X^*$$), a linear regression in the neighbourhood of $$X^*$$ would be more accurate than performing a regression all through the data points. For this, a weighting function, `v` is defined as

\begin{align}
    v_i(X^\*) = \exp{\frac{(X^* - X)^2}{2h}}
\end{align}
where $$h$$ decides the width of the gaussian and is chosen manually from test data. The new loss function would become
\begin{align}
    L(X^\*) &= \sum_i (X_iw - Y_i)^2 ~v_i(X^\*) + \lambda w^Tw \nonumber \newline
    L(X^\*) &= \sum_i (\hat{X}\hat{w} - Y)^T V (\hat{X}\hat{w} - Y) + \lambda w^Tw 
    \label{eqn:lossfunLWLR}
\end{align}
And the solution would be

\begin{align}
    w(X^*) = (X^TV + \lambda I)^{-1} X^T VY
\end{align}