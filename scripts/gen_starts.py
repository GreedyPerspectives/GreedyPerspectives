import random
import math

for i in range(10):
    s = []
    for r in range(18):
        while True:
            x = random.randint(10, 20)
            y = random.randint(10, 20)
            b = True
            for a in s:
                if a[0] == x and a[1] ==y:
                    b = False
            if b:
                break
        s.append([x,y,5,random.uniform(-math.pi, math.pi)])
    
    for p in s:
        print(f"{p[0]},{p[1]},{p[2]},{round(p[3],2)}")
