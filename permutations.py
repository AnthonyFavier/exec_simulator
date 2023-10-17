import itertools
import random
import matplotlib.pyplot as plt
import sys

pairs = [1,2,3,4,5]

P = {
    1:('R1-R2', 'R2-R1'),
    2:('R3-R4', 'R4-R3'),
    3:('R5-R6', 'R6-R5'),
    4:('R7-R8', 'R8-R7'),
    5:('R9-R10', 'R10-R9'),
}


pair_permutations = list(itertools.permutations(pairs))

final_permutations = []

for pair_permu in pair_permutations:
    print(pair_permu)

    l = [ P[x] for x in pair_permu]
    output = list(itertools.product(*l))
    final_permutations += output


f = open("permu.txt", "w")

f.write( str(len(final_permutations)) + "\n" )
# print(len(final_permutations))
for fp in final_permutations:
    f.write(str(fp)+"\n")
    # print(fp)
    pass
# f.close()



MAX = 3479
N = 20

seed = random.randrange(sys.maxsize)
seed = 4416936144151636634
random.seed(seed)
print(f"\nSeed was: {seed}")
f.write(f"seed= {4416936144151636634}\n")

def generate_new_int(a, b, L):
    new_int = random.randint(a, b)
    while new_int in L:
        new_int = random.randint(a, b)
    return new_int

def add_new_random_int(L):
    new_int = generate_new_int(1, MAX, L)
    L.append(new_int)
    L.sort()

orders = []
for i in range(N):
    add_new_random_int(orders)
print(orders)

print("Chosen orders:")
f.write("Chosen orders:\n")
for o in orders:
    print(final_permutations[o])
    f.write(str(final_permutations[o]) + "\n")

fig, axs = plt.subplots()
axs.plot(range(len(orders)), orders)
plt.show()

f.close()