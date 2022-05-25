import numpy as np
import os
import dill
import random

org = os.getcwd()
def determine_noise_scale(dir, lim=2000):
    os.chdir(dir)
    finals = []
    lst = os.listdir(".")
    for _ in range(lim):
        it = random.choice(lst)
        fname = dir + "/" + it
        with open(fname, 'rb') as f:
            d = dill.load(f, encoding='latin1')
            finals.append(d["player_future"][-1][0] + d["player_future"][-1][1])
        f.close()
    os.chdir(org)
    return np.std(np.array(finals)) / 100
