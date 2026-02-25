import numpy as np
from numpy import cos, sin, radians, pi
from random import randint

# Remove scientific notation
np.set_printoptions(suppress=True)

print("=== [3 DOF Articulated Manipulator] ===\n(RRR)")

# Set the Link Lengths
a1, a2, a3 = [10, 5, 6]

PT = lambda T1, T2, T3: [
    [radians(float(T1)),    pi/2,   0,      a1],
    [radians(float(T2)),    0,      a2,     0 ],
    [radians(float(T3)),    0,      a3,     0 ]]

HTM_all = lambda table: np.around(np.linalg.multi_dot(
            [np.matrix([
                [cos(o),    -sin(o)*cos(a),     sin(o)*sin(a),  r*cos(o)],
                [sin(o),    cos(o)*cos(a),      -cos(o)*sin(a), r*sin(o)],
                [0,         sin(a),             cos(a),         d],
                [0,         0,                  0,              1]
            ]) for o, a, r, d in table]), 3)

# The data points were generated using the following code:
# for i in range(0, 5):
#     print([randint(-90, 90) for _ in range(0, 3)])

print("\nHomogenous Transformation Matrix Results:\n")

data_points = [ [    -80,    64,     24  ],
                [   51,     -2,      17  ],
                [   -69,    70,      -85 ],
                [   -38,    -21,     24  ],
                [   -65,    -30,     -27 ]]

for i in range(0, len(data_points)):
    data = data_points[i]
    print("Sample {}.) T1 = {}, T2 = {}, T3 = {}: ".format(i+1, *data))
    print("H0_3 = \n{}\n".format(HTM_all(PT(*data))))