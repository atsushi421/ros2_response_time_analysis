import math


def useconds(n):
    frac, i = math.modf(n / 100)
    assert frac == 0
    return int(i)


def mseconds(n):
    return useconds(1000 * n)


def seconds(n):
    return mseconds(1000 * n)


def Hz(n):
    return int(seconds(1 / n))
