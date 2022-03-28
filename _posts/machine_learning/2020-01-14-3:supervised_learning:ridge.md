---
layout: post
title: "Part3: Supervised Learning: Ridge regression"
date: 2020-03-21 012:15:00-0400
description: Building blocks of supervised learning
categories: Machine-Learning
enable_math: true
---

<!-- <div style="text-align: justify"> <a href="https://www.ros.org/"> ROS </a> --> <!-- </div> -->
In this post, I will address the question `What happens if the feature matrix is singular?`



<!-- \section{Ridge regression} -->
Singularity means the determinant of the matrix goes to zero and this would happen if any two of the columns or rows become linearly dependant. It is often said as the feature matrix loses rank and hence $$X^TX$$ may not be invertible. In order to avoid this, the ridge regression adds a penalty so that the value of the weight vector does not go very large. Th loss function, `L`, becomes
\begin{align}
    L &= (\hat{X}\hat{w} - Y)^T (\hat{X}\hat{w} - Y) + \lambda w^Tw\label{eqn:lossfunRidge}
\end{align}
where $$\lambda$$ is called the regularization coefficient. The choice of $$\lambda$$ is done manually by optimizing a test data set. $$\lambda$$ also helps to prevent over-fitting (Ignore test data and over-fitting for now if you dont understand this).
Following the same procedure as in [Part1](/blog/2020/1-supervised_learning-basic/), i.e., by taking the derivative and taking transpose, the weight vector (or parameters) can be obtained as
\begin{align}
    w &= (X^TX + \lambda I)^{-1} X^TY
\end{align}
**Notes:** 
* Most cases, when linear regression is done means it's ridge regression.
* A very intuitive example of over-fitting is given in Chapter 1 under the Example: Polynomial Curve Fitting in the book `Pattern Recognition and Machine Learning` [book](https://link.springer.com/book/9780387310732) by Christopher Bishop
* A crude approach to start would be to just set $$\lambda$$ to a very small value, for instance, $$10^{-03}$$ and start varying it to see the change in `w`

