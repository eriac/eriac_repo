import cvxpy
import numpy as np
import matplotlib.pyplot as plt

import cvxopt
from cvxopt import matrix
import scipy.linalg

import time

"""
https://myenigma.hatenablog.com/entry/2017/02/07/084922
https://qiita.com/taka_horibe/items/47f86e02e2db83b0c570
"""


A = np.array([[1],[2]])
B = np.array([[10, 11],[21, 22]])
print(A)
print(B)
print(np.hstack([A,B]))

