import itertools
import random
import matplotlib.pyplot as plt
import sys
from matplotlib.ticker import (MultipleLocator, FormatStrFormatter, AutoMinorLocator)

f = open("orders.txt", "w")

l = range(1,7)

permu = list(itertools.permutations(l))

MAX = len(permu)
N = 20

print(MAX)


seed = random.randrange(sys.maxsize)
seed = 3203038706467100179
random.seed(seed)
print(f"\nSeed was: {seed}")
f.write(f"seed= {seed}\n")

def generate_new_int(a, b, L):
    new_int = random.randint(a, b)
    while new_int in L:
        new_int = random.randint(a, b)
    return new_int

def add_new_random_int(L):
    new_int = generate_new_int(0, MAX-1, L)
    L.append(new_int)
    L.sort()

orders = []
for i in range(N):
    add_new_random_int(orders)
print(orders)

print("Chosen orders:")
f.write("Chosen orders:\n")
for o in orders:
    print(permu[o])
    f.write(str(permu[o]) + "\n")
f.close()

fig, axs = plt.subplots()
axs.plot(range(len(orders)), orders, marker=".")
axs.xaxis.set_major_locator(MultipleLocator(2))
axs.xaxis.set_minor_locator(MultipleLocator(1))
plt.show()
