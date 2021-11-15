

vect = [1,2,3]
combs = []

for i in range(len(vect)):
    elem = vect[i]
    # new_vect = vect[i+1:]
    for j in range(i+1, len(vect)):
        combs.append((vect[i], vect[j]))

for t in combs:
    print(t)
print(len(combs))