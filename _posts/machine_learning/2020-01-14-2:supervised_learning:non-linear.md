---
layout: post
title: "Part2: Supervised Learning: Extending to higher dimensions and non-linear regression"
date: 2020-02-17 012:15:00-0400
description: Building blocks of supervised learning
categories: Machine-Learning
enable_math: true
---

<!-- <div style="text-align: justify"> <a href="https://www.ros.org/"> ROS </a> --> <!-- </div> -->
In [Part1](/blog/2020/1-supervised_learning-basic/), the aim was to predict house prices given the number of bedrooms. What if the house prices are dependant not just on the number of bedrooms but on other parameters as well, for instance, floor area. Let's build a mathematical model for the same and it isn't too much different from [Part1](/blog/2020/1-supervised_learning-basic/)'s model.

<br>

#### Mathematical model 2:
\begin{align}
	y = w_0 + w_1 \times nB + w_2 \times fA
	\label{line1}
\end{align}
where `w's` are weights that need to be found out and `nB` and `fA`'s (or `x`) are respectively the number of bedrooms and floor area in square units.

`house prices, (y):` 150k, 220k, 320k \\
`Number of bedrooms, (nB):` 1, 2, 3  
`Floor Area, (fA):` 42, 59, 61  


\begin{align}
	Y &= \begin{bmatrix}
           1 & nB & fA
        \end{bmatrix} \begin{bmatrix}
                              w_0 \nonumber \newline
                              w_1 \nonumber \newline
                              w_2
                        \end{bmatrix} \newline
      &= X^Tw
\end{align}
The matrix equation becomes
\begin{align}
	\begin{bmatrix}
		150 \nonumber \newline
		220 \nonumber \newline
		320
    \end{bmatrix}  = \begin{bmatrix}
       					1 & 1 & 42 \nonumber \newline
       					1 & 2 & 59 \nonumber \newline
       					1 & 3 & 61
    				 \end{bmatrix}
        			 \begin{bmatrix}
                      	w_0 \nonumber \newline
                      	w_1 \nonumber \newline
                        w_2
                     \end{bmatrix} 
\end{align}

The equation to find the parameters would remain the same as in Part1 (reproduced in eq. \ref{parameter}), the change being in the feature matrix and also an added parameter (`w_2`).
\begin{align}
	\hat{w} = (X^TX)^{-1}X^TY
	\label{parameter}
\end{align}

Code in `Python` to solve the same

```python

import numpy as np       # library for mathematics (linear algebra)

nB = np.array([1.0, 2.0, 3.0, ])  # Num bedrooms
fA = np.array([42.0, 59.0, 61.0])  # Floor area
Y = np.array([150.0, 220.0, 320.0])  # Cost


s = np.ones(len(nB)).reshape(-1,1)
X = np.hstack((s, nB.reshape(-1, 1), fA.reshape(-1, 1)))
a = X.transpose() @ X
b = X.transpose() @ Y
w = np.linalg.solve(a, b)  # To find parameters
print(w)

######
[130.0, 104.0, -2.0]
```

<br>

#### Non-linear regression
The above case can be extended to non-linear cases as long as the weight vector $$w$$ stays linear

\begin{align}
	y = \phi(x)^T w
\end{align}

where $$\phi(x)$$ can be a polynomial or radial basis or any other functions.
The non-linear case for our house pricing example can be written explicitly as
\begin{align}
	y &= w_0 + w_1 \times nB^2 + w_2 \times fA^2 \nonumber \newline
	  &= \begin{bmatrix}
           1 & nB^2 & fA^2
        \end{bmatrix} \begin{bmatrix}
                              w_0 \nonumber \newline
                              w_1 \nonumber \newline
                              w_2
                        \end{bmatrix} \newline
      &= \phi(x)^Tw
      \label{eq:non-linear}
\end{align}

```python
X = np.hstack((s, nB.reshape(-1, 1)**2, fA.reshape(-1, 1)**2))
a = X.transpose() @ X
b = X.transpose() @ Y
w = np.linalg.solve(a, b)  # To find parameters
print(w)

######
[119.1, 19.69, 0.00635]
```
The $$\phi(x)$$ in eq. \ref{eq:non-linear} is just one of the many forms $$\phi(x)$$ can take. Another possible one could be 
\begin{align}
    y &= w_0 + w_1 \times nB^2 + w_2 \times fA^2 + w_3 \times nB \times fA \nonumber \newline
      &= \begin{bmatrix}
           1 & nB^2 & fA^2 & nB \times fA
        \end{bmatrix} \begin{bmatrix}
                              w_0 \nonumber \newline
                              w_1 \nonumber \newline
                              w_2 \nonumber \newline
                              w_3
                        \end{bmatrix} \newline
      &= \phi(x)^Tw \nonumber
\end{align}

<br>

#### A few points to note

I mentioned about some small techiques used in practice to get better estimates of the parameters. 

* One of the most important one is [Feature Scaling](https://en.wikipedia.org/wiki/Feature_scaling). There are also a lot of other resources available on this and it's beyond the scope to discuss it here.
* What happens if the feature matrix is singular? I will briefly touch upon this in [Ridge regression](/blog/2020/3-supervised_learning-ridge/)
* What happens if the data is non-linear? A short intro to [locally weighted regression](/blog/2020/4-supervised_learning-weighted/) can be found here
