q = ["w", "xi", "yj", "zk"]
p = ["pi", "qj", "rk"]
qi = ["w", "-xi", "-yj", "-zk"]

def sortValue(x):
    vals = { "-": 0, "w": 1, "p": 2, "q": 3, "r": 4, "x": 5, "y": 6, "z": 7, "i": 8, "j": 8, "k": 8 }
    return vals[x]

def sort(x):
    return "".join(sorted(x, key=sortValue))

def translate(s):
    translateDict = { "ii":  "-", "ij":  "k", "ik": "-j",
                      "ji": "-k", "jj":  "-", "jk":  "i",
                      "ki":  "j", "kj": "-i", "kk":  "-",
                      "--":   "" }
    x = s
    while True:
        for key in translateDict.keys():
            x = x.replace(key, translateDict[key])
        x = sort(x)
        if x == s:
            break
        s = x
    return x

qp = [sort(i+j) for i in q
                for j in p]

qpq = [sort(i+j) for i in qp
                 for j in qi]

print("\r\nq*p:")
print(qp)

print("\r\nq*p*q^-1:")
print(qpq)

qpq_min = [translate(i) for i in qpq]

print("\r\nq*p*q^-1 with cancellations:")
print(qpq_min)

print("\r\nq*p*q^-1 by component:")
print("real: ", [w for w in qpq_min if not ('i' in w or 'j' in w or 'k' in w)], "=> 0")
print("i:    ", [x for x in qpq_min if 'i' in x])
print("j:    ", [y for y in qpq_min if 'j' in y])
print("k:    ", [z for z in qpq_min if 'k' in z])

print("\r\nj component for p = (1, 0, 0):")
print([y for y in qpq_min if ('j' in y and not('q' in y or 'r' in y))])
# print("\r\nk component for p = (1, 0, 0):")
# print([z for z in qpq_min if ('k' in z and not('q' in z or 'r' in z))])

# print("\r\nk component for p = (0, 1, 0):")
# print([z for z in qpq_min if ('k' in z and not('p' in z or 'r' in z))])

print("\r\nj component for p = (0, 0, 1):")
print([y for y in qpq_min if ('j' in y and not('p' in y or 'q' in y))])
# print("\r\nk component for p = (0, 0, 1):")
# print([z for z in qpq_min if ('k' in z and not('p' in z or 'q' in z))])
