<!--

---
title: Hexapod kinematics
author: Nicolas Jeanmonod
header-includes: |
    \usepackage{amsmath}
---

licensed under the GNU General Public License v3.0

-->

<!--

/**
 * S T E W A R T    P L A T F O R M    O N    E S P 3 2
 *
 * Copyright (C) 2019  Nicolas Jeanmonod, ouilogique.com
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

-->

# Hexapod kinematics

<p style="color:red">This document is a draft</p>

> Nicolas Jeanmonod, ouilogique.com, May 2019


## Displacements of the servo pivots $\text{B}_{i}$ and the platform joints $\text{P1}_{i}$

The coordinates of the platform joints at home position are called $\text{P0}_{i}$.

Note that $\text{P0z}_{i}=0$ and $i=\text{motor index}=0..5$.

$$
\text{P0}_{i}=\{P0x_i,P0y_i,0,1\}
$$

The platform has six degrees of freedom, three rotations and three translations with the following transformation matrices:

$$
\begin{gathered}

\text{Rx}=\left(
\begin{array}{cccc}
 1 & 0 & 0 & 0 \\
 0 & \phantom{-}\cos A & \sin A & 0 \\
 0 & -\sin A & \cos A & 0 \\
 0 & 0 & 0 & 1 \\
\end{array}
\right)
\\
\\
\text{Rz}=\left(
\begin{array}{cccc}
 \phantom{-}\cos C & \sin C & 0 & 0 \\
 -\sin C & \cos C & 0 & 0 \\
 0 & 0 & 1 & 0 \\
 0 & 0 & 0 & 1 \\
\end{array}
\right)

\end{gathered}




\begin{gathered}

\text{Ry}=\left(
\begin{array}{cccc}
 \cos B & 0 & -\sin B & 0 \\
 0 & 1 & 0 & 0 \\
 \sin B & 0 & \phantom{-}\cos B & 0 \\
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

The coordinates of the platform joints after movement are called $\text{P1}_{i}$ and can be found by calculating the following dot product:

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
P0x_i \cos B \cos C + P0y_i (\sin A \sin B \cos C - \cos A \sin C) + \text{Tx} \\
P0x_i \cos B \sin C + P0y_i (\sin A \sin B \sin C + \cos A \cos C) + \text{Ty} \\
-P0x_i\sin B + P0y_i \sin A \cos B+ \text{Tz} \\
1 \\
\end{array}
\right)^T
$$

We can now calculate the distances between the servo pivots $\text{B}_{i}$ and the platform joints $\text{P1}_{i}$:

$$
\text{dPB}_{i} = \left(\text{P1}_{i} - \text{B}_{i}\right)= \left(\text{P1}_{i} - \{Bx_i,By_i,-Z_{home},1\}\right) =
$$

$$
\boxed{
\left(
\begin{array}{c}
\text{dPBx}_i \\
\text{dPBy}_i \\
\text{dPBz}_i \\
1
\end{array}
\right)^T
=
\left(
\begin{array}{c}
P0x_i \cos B \cos C + P0y_i (\sin A \sin B \cos C - \cos A \sin C) + \text{Tx} - \text{Bx}_{i} \\
P0x_i \cos B \sin C + P0y_i (\sin A \sin B \sin C + \cos A \cos C) + \text{Ty} - \text{By}_{i} \\
-P0x_i\sin B + P0y_i \sin A \cos B+ \text{Tz} + \text{Z}_{home} \\
1 \\
\end{array}
\right)^T
}
\text{(eq 1)}
$$


And the squares of the length of the vectors are:

$$
\boxed{
d_{i}^2=\text{dPBx}_i^2+\text{dPBy}_i^2+\text{dPBz}_i^2
}
\text{    (eq 2)}
$$

For a platform using linear motors, the calculation can be done with $\text{eq 1}$ and $\text{eq 2}$. In our case we have rotational motors so we need to continue. Note that it is not a good idea to compute the square root of $d_{i}^2$ because it is a computer intensive operation and we don’t need the value of $d$.

Now, we need to check if the arm of length $a$ and the rod of length $s$ are long enoug to actualy go to the target position:

$$
d_{i}^2 \leq(a + s)^2
\text{    (eq 3)}
$$


<div style="page-break-before:always;"></div>

## Projection of the rod sphere on the arm plane

<p style="color:red">The calculation below works when point B is between P' and R. It doesn’t work when P' is between B and R.</p>


$P$ is the projection of $P_i$ on the $xy$ plane.


$\overline{PR} =$ rod length

$\overline{BR}$ is a segment that belongs to the plane formed by point A (joint at the end of servo arm) during rotation

$P'$ is the projection of $P$ on $\overline{BR}$

$\overline{PB'} = \text{dPBx}_i$

$\overline{BB'} = \text{dPBy}_i$

### Angles calculation
$$
\widehat{P'PB'} = \frac{\pi}{2} - \theta{s}_i
$$
$$
\widehat{BPB'} = atan\frac{\text{dPBy}_i}{\text{dPBx}_i}
$$
$$
\widehat{P'PB} =
\widehat{P'PB'} - \widehat{BPB'} =
\frac{\pi}{2} - \theta{s}_i - atan\frac{\text{dPBy}_i}{\text{dPBx}_i}
$$

### Length calculation

Projection of $P$ on the servo arm plane.

$$
\overline{PB} = \sqrt{\text{dPBx}_i^2 + \text{dPBy}_i^2}
$$
$$
\overline{P'P} = \overline{PB} \cdot cos\widehat{P'PB}
$$
$$
\boxed{
\overline{P'P} =
\sqrt{\text{dPBx}_i^2 + \text{dPBy}_i^2}
\cdot
cos\left(\frac{\pi}{2} - \theta{s}_i - atan\frac{\text{dPBy}_i}{\text{dPBx}_i}
\right)
}
$$

The intersection of the sphere formed by the rod and the plane of the rotation of the servo arm joint is a circle of radius $r_\text{cs}$.

$$
r_{cs}
= \overline{P'R}
= \sqrt{\overline{PR}^2 - \overline{P'P}^2}

$$
$$
\boxed{
r_{cs}
= \sqrt{\overline{\text{Rod Length}}^2 - \overline{P'P}^2}
}
$$

<!--
=SQRT(rod_length^2 - (dPBx^2+dPBy^2) * (COS(theta_s - PI()/2 -ATAN(dPBy/dPBx)))^2)
-->
$$
\tiny

r_{cs} =
\sqrt
{
\text{Rod Length}^2 -
\left(\text{dPBx}_i^2 + \text{dPBy}_i^2\right) \cdot
cos^2\left(\frac{\pi}{2} - \theta{s}_i - atan\frac{\text{dPBy}_i}{\text{dPBx}_i}\right)
}

$$

### In the arm plane

The center of this circle **in the plane of the servo arm** is at coordinates $c_x, c_z$ when the origin $0,0$ is set to point $B$, i.e. on the projection of the servo shaft.

$$

c_z = \text{dPBz}_i

$$

$$
c_x
= \overline{P'B_\text{⏥xy}}
= \sqrt{\overline{PB}^2 - \overline{P'P}^2}
$$

$$
\boxed{
c_x
= \sqrt{\text{dPBx}_i^2 + \text{dPBy}_i^2 - \overline{P'P}^2}
}
$$

$$
\tiny
c_x =
\sqrt{
\text{dPBx}_i^2 + \text{dPBy}_i^2
-
\left(\text{dPBx}_i^2 + \text{dPBy}_i^2\right) \cdot
cos^2\left(\frac{\pi}{2} - \theta{s}_i - atan\frac{\text{dPBy}_i}{\text{dPBx}_i}\right)}
$$


$$
\boxed{
\alpha = atan\left(\frac{\text{dPBz}_i}{c_x}\right)
}
$$

Cosinus theorem

$$
r_\text{cs} = \text{arm}^2 + \overline{P'B_\text{⏥arm}}^2 - 2 \text{ arm } \overline{P'B_\text{⏥arm}} \text{ } cos\beta
$$



$$
\boxed{
\beta = acos\left(
\frac{\text{arm}^2 + \overline{P'B_\text{⏥arm}}^2 - r_\text{cs}^2}
{2 \text{ arm } \overline{P'B_\text{⏥arm}}}
\right)
}
$$

$$
\boxed{
\overline{P'B_\text{⏥arm}}
=\sqrt{ \overline{P'B_\text{⏥xy}}^2 + \text{dPBz}_i^2}
}
$$


$$
\boxed{
\text{servo angle} = \pi - \alpha - \beta
}
$$

