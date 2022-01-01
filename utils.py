def ccw(A, B, C) -> int:
    '''
    abc is counter-clockwise:
        return 1
    abc is clockwise:
        return -1
    abc are collinear:
        return 0
    '''
    d = (A[0]*B[1])+(B[0]*C[1])+(C[0]*A[1]) - \
        (C[0]*B[1])-(B[0]*A[1])-(A[0]*C[1])
    e = 10**-12
    if d < -e:
        return -1
    elif d < e:
        return 0
    else:
        return 1


def pointCCW(A, B, C) -> int:
    a = [A.x, A.y]
    b = [B.x, B.y]
    c = [C.x, C.y]
    return ccw(a, b, c)


def distance(a, b):
    return ((b.x-a.x)**2+(b.y-a.y)**2)**(1/2)


def det2x2(a, b):
    return a[0]*b[1]-a[1]*b[0]


def intersect(s1, s2, closed=True):
    d = det2x2([s1.start.x-s1.end.x, s2.start.x-s2.end.x],
               [s1.start.y-s1.end.y, s2.start.y-s2.end.y])
    if d == 0:
        return False
    t = det2x2([s1.start.x-s2.start.x, s2.start.x-s2.end.x],
               [s1.start.y-s2.start.y, s2.start.y-s2.end.y])/d
    u = det2x2([s1.start.x-s2.start.x, s1.start.x-s1.end.x],
               [s1.start.y-s2.start.y, s1.start.y-s1.end.y])/d
    if closed:
        return 0 <= u <= 1 and 0 <= t <= 1
    return intersect(s1, s2) and (s1.start != s2.start) and (s1.start != s2.end) and (s1.end != s2.end) and (s1.end != s2.start)
