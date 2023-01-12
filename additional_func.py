import numpy as np


def length(array: np.array) -> int:
    """
    Like a length() in Matlab.

    :param array: numpy array.
    :return: the length of the largest array dimension.
    """
    return max(array.shape)
