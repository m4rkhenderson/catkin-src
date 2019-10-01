import random as rdm


def sampling(xmax, ymax, xmin, ymin):
    qRand = [rdm.randint(xmin, xmax), rdm.randint(ymin, ymax), float(rdm.randint(0, 628))/100]
    return qRand
