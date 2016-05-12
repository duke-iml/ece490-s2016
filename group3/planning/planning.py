def cleanJointConfig(q):
    if len(q) != 7:
        print "This function is not being used as intended"
    else:
        q[6] = 0.0
    return q