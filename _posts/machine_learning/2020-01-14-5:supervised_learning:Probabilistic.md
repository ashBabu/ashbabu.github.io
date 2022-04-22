---
layout: post
title: "Part5: Supervised Learning: Probabilistic regression"
date: 2020-05-01 012:15:00-0400
description: Building blocks of supervised learning
categories: Machine-Learning
enable_math: true
---

<!-- <div style="text-align: justify"> <a href="https://www.ros.org/"> ROS </a> --> <!-- </div> -->

<!-- \section{Probabilistic Regression} -->
The mathematical model for Probabilistic Regression is $$P(Y | X)$$  which means probability of output ($$Y$$) given input ($$X$$). This is assumed to be a normal distribution with mean $$w$$ and variance $$\sigma^2$$ which are unknown. \\
**Assumption:** Independent and identically distributed (i.i.d). For i.i.d's, the total probability is the product of individual probabilities
\begin{align}
    P(Y|X) &= \prod p(y_i|x_i) = \mathcal{N}(Y|X^Tw, \sigma^2I)\, \nonumber \newline
    &=\frac{1}{(2\pi)^{D/2}} \frac{1}{|\Sigma|^{1/2}} \exp [-\frac{1}{2} (X^Tw - Y)^T \Sigma^{-1} (X^Tw - Y) ]
    \label{eqn:multivarGaussian}
\end{align}
where $$\Sigma = \sigma^2I $$.   Eq. \ref{eqn:multivarGaussian} is the equation for Multivariate Gaussian Distribution

#### Maximum Likelihood Estimation
The loss function becomes
\begin{align}
    L = P(Y|X) - \prod p(y_i|x_i) 
    \label{eqn:maxLikeli}
\end{align}
Eq. \ref{eqn:maxLikeli} is very difficult to solve and hence a mathematical trick is done by trying to solve the maximum **log** likelihood because the maximum of both would be the same
\begin{align}
    L = \rm{log}~ P(Y|X) &= \sum_i \rm{log}~ p(y_i|x_i) \nonumber \newline
                &= \sum_i(-\rm{log} ~Z) - \frac{1}{2} (X^Tw - Y)^T \Sigma^{-1} (X^Tw - Y)
\end{align}
where Z = 2$$\pi \Sigma$$ terms which are constants. Removing the constant terms which does not dependent on the maximization, it becomes

\begin{align}
    L = \rm{log} ~ P(Y|X) = (X^Tw - Y)^T \Sigma^{-1} (X^Tw - Y)
\end{align}
which is exactly same as linear regression with $$\Sigma$$ as identity.

#### Maximum a-posteriori solution
A prior distribution is required to predict the posterior distribution

The mathematical model is $$P(Y|X,w)$$
\begin{align}
    \rm{posterior}~ p(w|D) = \frac{p(D|w)p(w)}{p(D)} \nonumber\\
\end{align}
where D represents data 
\begin{align}
    L_{MAP} &= \rm{log} ~p(w|D) \nonumber \\
           &= \rm{log} ~p(D|w) + p(w) - p(D) \nonumber \\
           &= \rm{log} ~p(Y|X,w) + p(w)
\end{align}
where `p(D)` is a constant and is not considered for optimization and `p(w)` is assumed to be normal distribution with mean zero and variance $$\lambda I$$ or $$p(w) = \mathcal{N}(w|0, \lambda I)$$. This makes MAP solution exactly same as Ridge regression

