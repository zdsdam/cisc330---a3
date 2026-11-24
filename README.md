# CISC/CMPE 330 Assignment 3

Tasks 1–4 model registration and targeting for the MRI-guided robot in the
assignment. Lengths are in millimetres and joint angles are in degrees.

## Run the verification

From the repository root, use Python 3.11 or newer in your Python environment.

~~~sh
python -m pip install -r requirements.txt
python -m unittest -v
~~~

The 10 tests print expected and computed values and exit nonzero on failure.
They use an absolute comparison tolerance of $10^{-9}$.

## 1. MRI-to-robot transform (13 points)

The MRI frame uses the scanner's fixed right-handed $x,y,z$ axes. The robot
frame is right-handed with its origin at the center of the shaft shown in the
handout. Its $+z$ axis is along the shaft, the home needle lies in the $yz$
plane toward $-y$ and $+z$, and $+x$ completes the right-handed frame. The
following markers are fixed in this shaft-centered frame and must be supplied
in the same order in MRI.

| Marker | $x$ (mm) | $y$ (mm) | $z$ (mm) |
| --- | ---: | ---: | ---: |
| M1 | 0 | −60 | −100 |
| M2 | 0 | 0 | −100 |
| M3 | 0 | −10 | −20 |

In each frame, call the three marker positions $A,B,C$. Build an auxiliary
marker frame with origin $o=(A+B+C)/3$ and right-handed basis

$$
i=\frac{B-A}{\lVert B-A\rVert},\qquad
k=\frac{i\times(C-A)}{\lVert i\times(C-A)\rVert},\qquad
j=k\times i.
$$

The columns $i,j,k$ form the rotation block of a homogeneous matrix, and $o$
is its translation column. This gives $F_{\mathrm{robot}\leftarrow\mathrm{marker}}$
and $F_{\mathrm{MRI}\leftarrow\mathrm{marker}}$ from the same marker geometry
expressed in the two frames. In robot coordinates the marker basis is
$i=(0,1,0)$, $j=(0,0,1)$, $k=(1,0,0)$, with centroid $(0,-70/3,-220/3)$ mm.
The composition uses both marker frames and returns coordinates relative to
the shaft origin.

$$
F_{\mathrm{robot}\leftarrow\mathrm{MRI}} =
F_{\mathrm{robot}\leftarrow\mathrm{marker}}
F_{\mathrm{MRI}\leftarrow\mathrm{marker}}^{-1},\qquad
p_{\mathrm{robot}}=F_{\mathrm{robot}\leftarrow\mathrm{MRI}}p_{\mathrm{MRI}}.
$$

`mri_to_robot(markers_mri)` in [mri_to_robot_transform.py](mri_to_robot_transform.py)
takes a $3\times3$ array with one marker per row and returns this $4\times4$
matrix. Points in the equation are homogeneous column vectors with last
coordinate $1$. The Assignment 2 geometry helpers
compute the bases, rigid inverse, and composition. [A2 geometry.py](https://github.com/simondeveaux06/cisc330---a2/blob/d1aacab33a06d3f179bfbb992c9ad3c198e63626/geometry.py)

### Hand calculations and verification

The pose in each row maps robot points into MRI, so the expected matrix
implements the inverse rule. The first two cases isolate the translation and
rotation blocks. The third rotates about a different axis and checks the
composition order with a nonzero translation.

| Robot-to-MRI pose and forward rule | Predicted MRI markers M1, M2, M3 (mm) | Expected MRI-to-robot rule on MRI point $(X,Y,Z)$ | Max matrix error | Max marker error (mm) |
| --- | --- | --- | ---: | ---: |
| Add $(10,20,30)$ | $(10,-40,-70)$, $(10,20,-70)$, $(10,10,10)$ | $(X-10,Y-20,Z-30)$ | $7.11\times10^{-15}$ | $7.11\times10^{-15}$ |
| Rotate $+90^\circ$ about $z$ to $(-y,x,z)$ | $(60,0,-100)$, $(0,0,-100)$, $(10,0,-20)$ | $(Y,-X,Z)$ | 0 | 0 |
| Rotate $+90^\circ$ about $x$ to $(x,-z,y)$, then add $(100,-50,200)$ | $(100,50,140)$, $(100,50,200)$, $(100,-30,190)$ | $(X-100,Z-200,50-Y)$ | 0 | 0 |

The test supplies these marker triples and the corresponding numerical $4\times4$
matrices independently of the registration code. It also recovers identity
when the MRI and robot frames coincide.

## 2. Workspace (5 points)

The shaft diameter is 30 mm, so the 10 mm uniform rectal wall puts its outer
surface 25 mm from the shaft axis. Model the largest prostate as a sphere of
radius $R=30$ mm tangent to that surface on the $-y$ side. Its center is
$(0,-D,z_c)$, where $D=15+10+30=55$ mm. The needle angle to $+z$ is
$45^\circ$. The handout leaves $z_c$ unspecified. We use $z_c=55$ mm for the
worked example to center the required translation stroke at zero.

Let $d=\sqrt{x^2+y^2}$ and $t=z-d$, as derived in Task 3.

- **Insertion.** The sphere projects onto a disk of radius $R$ in the $xy$
  plane. The nearest and farthest points to the shaft axis lie on the
  $-y$ ray at $z=z_c$ and give $D-R\le d\le D+R$. Physical travel from
  zero depth is $\sqrt{2}d$ along the $45^\circ$ needle.
- **Rotation.** The disk is bounded in angle by the two rays from the shaft
  axis tangent to it. The right triangle with hypotenuse $D$ and opposite
  side $R$ gives $|\theta|\le\arcsin(R/D)$, measured from $-y$.
- **Translation.** At $u=z-z_c$, the sphere slice is a disk of radius
  $r=\sqrt{R^2-u^2}$, so $D-r\le d\le D+r$. Therefore
  $|t-(z_c-D)|=|u-(d-D)|\le |u|+r\le\sqrt{2}R$. The lower
  limit occurs at $u=-R/\sqrt{2}$ and $d=D+R/\sqrt{2}$. The
  upper limit occurs at $u=R/\sqrt{2}$ and $d=D-R/\sqrt{2}$.

| Joint quantity | Exact required range | Computed at $z_c=55$ mm |
| --- | --- | --- |
| Translation (mm) | $z_c-D\pm\sqrt{2}R$ | −42.426 to +42.426 |
| Rotation (degrees) | $\pm\arcsin(R/D)$ converted to degrees | −33.056 to +33.056 |
| Depth coordinate (mm) | $[D-R,D+R]$ | 25 to 85 |
| Travel along the needle (mm) | $[\sqrt{2}(D-R),\sqrt{2}(D+R)]$ | 35.355 to 120.208 |

Every target in the sphere has nonzero radial distance, so its inverse
kinematics is defined. Each required joint value lies inside the corresponding
range above, and applying those settings reaches the target. Thus the ranges
cover the full sphere. Each joint bound is attained by a sphere boundary
point. The tests check all six witnesses, with joint-limit errors at most
$7.11\times10^{-15}$ on the verification run. The translation stroke is
$2\sqrt{2}R=84.853$ mm for any $z_c$. `workspace_limits(prostate_z_mm)` in
[workspace.py](workspace.py) returns the four ranges as named pairs of minimum
and maximum values.

## 3. Forward and inverse kinematics (18 points)

Joint zero is the nominal home position. Use the fixed home robot frame for
both target input and tip output. Positive translation $t$ is along $+z$,
positive shaft angle $\theta$ follows the right-hand rule about $+z$, and
$d\ge0$ is the depth coordinate in the handout's home-tip formula. At
$45^\circ$, $d$ is both the radial and the axial component of travel from
zero depth.

**Forward.** Rotate the local needle point $(0,-d,d)$ about $+z$ and add the
translation $(0,0,t)$. The three components are

$$
(x,y,z)=(d\sin\theta,-d\cos\theta,t+d).
$$

**Inverse.** The first two coordinates give the radial distance and the
signed bearing from $-y$. Subtract the insertion contribution from the
target's $z$ coordinate to recover translation.

$$
d=\sqrt{x^2+y^2},\qquad \theta=\operatorname{atan2}(x,-y),\qquad t=z-d.
$$

In [inverse_kinematics.py](inverse_kinematics.py),
`forward_kinematics(translation_mm, rotation_deg, depth_mm)` returns a tip
in millimetres. `inverse_kinematics(target_robot_mm)` returns the three
settings in that same order, with the principal angle in degrees. At
$d=0$ the forward result is on the shaft axis. An on-axis inverse target
raises an error because shaft rotation is indeterminate there.

### Hand calculations and verification

Begin at the inserted reference $(t,\theta,d)=(0,0^\circ,50)$, where the
tip is $(0,-50,50)$ mm. Keep $d=50$ mm through all three moves. A $10$ mm
translation adds $10$ to $z$. Rotation by $\pm30^\circ$ contributes
$x=\pm25$ mm and $y=-25\sqrt{3}$ mm by the sine and cosine of $30^\circ$.
The resulting targets are strictly inside the example sphere.

| Stage motion | Target by hand (mm) | Expected and recovered $(t,\theta,d)$ | Inverse max abs. error | FK from expected settings max abs. error (mm) | FK after IK max abs. error (mm) |
| --- | --- | --- | ---: | ---: | ---: |
| Translation only | $(0,-50,60)$ | $(10,0^\circ,50)$ | 0 | 0 | 0 |
| Rotation only | $(25,-25\sqrt{3},50)$ | $(0,+30^\circ,50)$ | $3.55\times10^{-15}$ | $7.11\times10^{-15}$ | 0 |
| Translation and rotation | $(-25,-25\sqrt{3},60)$ | $(10,-30^\circ,50)$ | $3.55\times10^{-15}$ | $7.11\times10^{-15}$ | 0 |

## 4. Maximum targeting error (6 points)

Model the ablation as a sphere of diameter 12 mm centered at the actual
needle tip, and the lesion as a sphere of diameter 6 mm centered at the
intended target. Let $e$ be the distance between the centers. The lesion
point farthest from the needle tip lies on the line through both centers and
is $e+3$ mm away. Full lesion coverage requires

$$
e+3\ \mathrm{mm}\le6\ \mathrm{mm},\qquad
e_{\max}=6-3=3\ \mathrm{mm}.
$$

At equality the lesion just touches the ablation boundary internally.
`MAX_TRE_MM` in [maximum_targeting_error.py](maximum_targeting_error.py)
evaluates to **3.0 mm**, which the test verifies and Task 7 can import as its
targeting-error limit.

## Course references

- **HW-Robot-Nav**, pp. 1–3 and 7, defines the model, Tasks 1–4, and their rubric.
- **Math Primer Part 1**, pp. 7, 17, 22, and 42, covers cylindrical coordinates,
  line and sphere geometry, and a frame from three points. **Part 2**,
  pp. 16–23 and 34–37, covers axial rotations and homogeneous frame changes.
  **Part 3**, pp. 2–3 and 19–23, covers transform direction and registration
  error terminology for the later simulation.
- **MRI Robotic Prostate Biopsy**, pp. 9–10 and 17, illustrates the three
  joint motions and marker-based frame composition. **MRI Basics**,
  pp. 14–17, explains the frequency encoding behind the MRI marker observations.
