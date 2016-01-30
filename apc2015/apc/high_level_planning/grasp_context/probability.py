def probability_of(ptable,event):
    if isinstance(event,list):
        pevent = 1.0
        for e in event:
            if e in ptable:
                pevent += ptable[e]
        return pevent
    if event not in ptable:
        return 0.0
    else:
        return ptable[event]

def probability_complement(ptable,event):
    return 1.0 - probability_of(ptable,event)

def probability_mul(ptable1,value):
    """Returns a copy of ptable1 scaled by value"""
    res = {}
    for p in ptable1:
        res[p] = ptable1[p]*value
    return res

def probability_dot(ptable1,ptable2):
    """Element-wise dot product of probability table ptable2 to ptable1"""
    res = 0.0
    for p in ptable1:
        if p in ptable2:
            res += ptable1[p]*ptable2[p]
    return res

def probability_product(ptable1,ptable2):
    """Element-wise multiply probability table ptable2 to ptable1"""
    res = {}
    for p in ptable1:
        if p in ptable2:
            res[p] = ptable1[p]*ptable2[p]
    return res

def probability_add(ptable1,ptable2):
    """Add probability table ptable2 to ptable1"""
    res = {}
    for p in ptable1:
        res[p] = ptable1[p]
    for p in ptable2:
        if p in res:
            res[p] += ptable2[p]
        else:
            res[p] = ptable2[p]
    return res


def probability_sub(ptable1,ptable2):
    """Subtract probability table ptable2 from ptable1"""
    res = {}
    for p in ptable1:
        res[p] = ptable1[p]
    for p in ptable2:
        if p in res:
            res[p] -= ptable2[p]
            if res[p] < 0:
                print "probability_sub: warning, element",p,"became negative"
        else:
            print "probability_sub: warning, element",p,"in second arg not in first arg"
    return res

def probability_normalize(ptable):
    """Inplace normalize of probability table"""
    psum = 0.0
    for p in ptable:
        psum += ptable[p]
    if psum == 0:
        return
    scale = 1.0/psum
    for p in ptable:
        ptable[p] *= scale
