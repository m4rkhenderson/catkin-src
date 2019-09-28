import random as rdm


def sampling(xmax, ymax, xmin, ymin):
    qRand = [rdm.randint(xmin, xmax), rdm.randint(ymin, ymax)]
    return qRand
