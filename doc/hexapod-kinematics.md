---
title: Hexapod kinematics
author: Nicolas Jeanmonod
header-includes: |
    \usepackage{amsmath}
---

# Hexapod kinematics

> Nicolas Jeanmonod, May 2019

The coordinates of the platform joints at home position are called $\text{P0}_{i}$.

Note that $\text{P0z}_{i}=0$ and $i=\text{motor index}=0..5$.

$$
\text{P0}_{i}=\{P0x_i,P0y_i,0,1\}
$$

The platform has six degrees of freedom, three rotations and three translations with the following transformation matrices :

$$
\begin{gathered}

\text{Rx}=\left(
\begin{array}{cccc}
 1 & 0 & 0 & 0 \\
 0 & \phantom{-}\cos (A) & \sin (A) & 0 \\
 0 & -\sin (A) & \cos (A) & 0 \\
 0 & 0 & 0 & 1 \\
\end{array}
\right)
\\
\\
\text{Rz}=\left(
\begin{array}{cccc}
 \phantom{-}\cos (C) & \sin (C) & 0 & 0 \\
 -\sin (C) & \cos (C) & 0 & 0 \\
 0 & 0 & 1 & 0 \\
 0 & 0 & 0 & 1 \\
\end{array}
\right)

\end{gathered}




\begin{gathered}

\text{Ry}=\left(
\begin{array}{cccc}
 \cos (B) & 0 & -\sin (B) & 0 \\
 0 & 1 & 0 & 0 \\
 \sin (B) & 0 & \phantom{-}\cos (B) & 0 \\
 0 & 0 & 0 & 1 \\
\end{array}
\right)
\\
\\
\text{Txyz}=\left(
\begin{array}{cccc}
 1 & 0 & 0 & 0 \\
 0 & 1 & 0 & 0 \\
 0 & 0 & 1 & 0 \\
 \text{Tx} & \text{Ty} & \text{Tz} & 1 \\
\end{array}
\right)

\end{gathered}
$$

The coordinates of the platform joints after movement are called $\text{P1}_{i}$ and can be found by calculating the following dot product :

$$
\text{P1}_{i}=\left(\text{P0}_{i}\cdot\text{Rx}\cdot\text{Ry}\cdot\text{Rz}\cdot\text{Txyz}\right) =
$$

$$
\left(
\begin{array}{c}
\text{P1x}_i\\
\text{P1y}_i\\
\text{P1z}_i\\
1
\end{array}
\right)^T
=

\left(
\begin{array}{c}
 P0x_i \cos  B \cos  C + P0y_i (\sin  A \sin  B \cos  C-\cos  A \sin  C)  +   \text{Tx} \\
 P0x_i\cos  B \sin  C+ P0y_i (\cos  A \cos  C + \sin  A \sin  B \sin  C)  +   \text{Ty} \\
 -P0x_i  \sin  B + \sin  A \cos  B y  +   \text{Tz} \\
 1 \\
\end{array}
\right)^T
$$

We can now calculate the distances between the servo pivots $\text{B}_{i}$ and the platform joints $\text{P1}_{i}$ :

$$
\text{dPB}_{i} = \left(\text{P1}_{i} - \text{B}_{i}\right)  = \left(\text{P1}_{i} - \{Bx_i,By_i,-Z_{home},1\}\right) =
$$

$$
\left(
\begin{array}{c}
\text{dPBx}_i\\
\text{dPBy}_i\\
\text{dPBz}_i\\
1
\end{array}
\right)^T
=
\left(
\begin{array}{c}
 P0x_i \cos B \cos C + P0y_i (\sin  A \sin  B \cos  C-\cos  A \sin  C)  +   \text{Tx} -  \text{Bx}_{i}\\
 P0x_i \cos B \sin C + P0y_i (\cos  A \cos  C + \sin  A \sin  B \sin  C)  +  \text{Ty} -  \text{By}_{i}\\
 -P0x_i  \sin  B + \sin  A \cos  B y  +   \text{Tz} +  \text{Z}_{home}\\
 1 \\
\end{array}
\right)^T

\text{(eq 1)}
$$


And the squares of the length of the vectors are :

$$
d_{i}^2=\text{dDPx}_i^2+\text{dDPy}_i^2+\text{dDPz}_i^2
\text{    (eq 2)}
$$

For a platform using linear motors, the calculation can be done with $\text{eq 1}$ and $\text{eq 2}$. In our case we have rotational motors so we need to continue. Note that it is not a good idea to compute the square root of $d_{i}^2$ because it is a computer intensive operation and we don’t need the value of $d$.

Now, we need to check if the arm of length $a$ and the rod of length $s$ are long enoug to actualy go to the target position :

$$
d_{i}^2 \leq  (a + s)^2
\text{    (eq 3)}
$$

TO BE CONTINUED...
