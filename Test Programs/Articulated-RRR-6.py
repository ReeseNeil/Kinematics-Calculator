import numpy as np
from numpy import cos, sin, radians, pi
from random import randint

# Remove scientific notation
np.set_printoptions(suppress=True)

print("=== [6 DOF Articulated Manipulator] ===\n(RRR with Spherical Wrist)")

# Set the Link Lengths
a1, a2, a3, a4, a5, a6 = [15, 10, 6, 5, 5, 5]

PT = lambda T1, T2, T3, T4, T5, T6: [
    [radians(float(T1)),            pi/2,   0,  a1      ],
    [radians(float(T2)),            0,      a2, 0       ],
    [pi/2 + radians(float(T3)),     pi/2,   0,  0       ],
    [radians(float(T4)),            3*pi/2, 0,  a3 + a4 ],
    [3*pi/2 + radians(float(T5)),   pi/2,   0,  0       ],
    [radians(float(T6)),            0,      0,  a5 + a6 ]]

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
#     print([randint(-90, 90) for _ in range(0, 6)])

data_points = [ [-8, 6, 70, 72, 21, 48   ],
                [43, 85, -22, -58, 79, 41 ],
                [67, 52, -8, 90, -49, 69  ],
                [-49, 20, 78, -80, -16, 44 ],
                [-69, 67, 47, -21, -59, -75 ]]

for i in range(0, len(data_points)):
    data = data_points[i]
    print("Sample {}.) T1 = {}, T2 = {}, T3 = {}, T4 = {}, T5 = {}, T6 = {}: ".format(i+1, *data))
    print("H0_6 = \n{}\n".format(HTM_all(PT(*data))))