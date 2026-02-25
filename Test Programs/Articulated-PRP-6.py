import numpy as np
from numpy import cos, sin, radians, pi
from random import randint

# Remove scientific notation
np.set_printoptions(suppress=True)

print("=== [3 DOF Articulated Manipulator] ===\n(PRP w/ Spherical Wrist)")

# Set the Link Lengths
a1, a2, a3, a4, a5, a6, a7 = [15, 10, 5, 5, 3, 3, 3]

PT = lambda d1, T2, d3, T4, T5, T6: [
                [0,                     pi/2,   0,     -a1      ],
                [0,                     0,      0,     a2 + d1  ],
                [radians(float(T2)),    pi/2,   a3,    0        ],
                [0,                     0,      0,     a4 + d3  ],
                [radians(float(T4)),    3*pi/2, 0,     a5       ],
                [radians(float(T5)),    pi/2,   0,     0        ],
                [radians(float(T6)),    0,      0,     a6 + a7  ]]

HTM_all = lambda table: np.around(np.linalg.multi_dot(
            [np.matrix([
                [cos(o),    -sin(o)*cos(a),     sin(o)*sin(a),  r*cos(o)],
                [sin(o),    cos(o)*cos(a),      -cos(o)*sin(a), r*sin(o)],
                [0,         sin(a),             cos(a),         d],
                [0,         0,                  0,              1]
            ]) for o, a, r, d in table]), 3)

print("\nHomogenous Transformation Matrix Results:\n")

# The data points were generated using the following code:
# for i in range(0, 5):
#     print([randint(0, 10), randint(-90, 90), randint(0, 10), randint(-90, 90), randint(-90, 90), randint(-90, 90)])

data_points = [ [1, 75,  0,  -25, 8,   50],
                [9, 59,  10, -87, 20,  46],
                [8, -64, 0,  70,  -87, 83],
                [6, -52, 5,  64,  80, -47],
                [3, -10, 6,  53,  27, -53]]

for i in range(0, len(data_points)):
    data = data_points[i]
    print("Sample {}.) d1 = {}, T2 = {}, d3 = {}, T4 = {}, T5 = {}, T6 = {}: ".format(i+1, *data))
    print("H0_7 = \n{}\n".format(HTM_all(PT(*data))))
    

    