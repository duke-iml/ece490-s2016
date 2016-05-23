import numpy

def rgb_to_yuv(r, g, b): # in (0,255) range
    y = .299*r + .587*g + .114*b
    u = 128 -.168736*r -.331364*g + .5*b            
    v = 128 +.5*r - .418688*g - .081312*b
    return y, u, v
    
def make_uv_hist(vals,binsize=4):
    '''
    make uv histogram
    vals is a list of lists [[u1, v1], [u2, v2], ..., [un, vn]]
    bins are from integers from 0 to M where M=256/binsize-1
    return a M-by-M np.matrix where (i,j) entry is the number of elements having i*binsize <= u < (i+1)*binsize and j*binsize<=v<=(j+1)*binsize
    '''
    if binsize==1:
        hist = numpy.zeros((256, 256))
        try:
            vals = vals.tolist()
        except:
            pass
        for val in vals:
            hist[int(val[0]), int(val[1])] += 1
        hist = hist / len(vals)
        return hist
    hist = numpy.zeros((256/binsize, 256/binsize))
    try:
        vals = vals.tolist()
    except:
        pass
    for val in vals:
        hist[int(val[0])/binsize, int(val[1])/binsize] += 1
    hist = hist / len(vals)
    return hist