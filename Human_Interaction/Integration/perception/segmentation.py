def distance_label(cloud, mask, threshold=0.005):
    threshold *= threshold
    mark = time()
    
    label = 0
    labeled = numpy.zeros(cloud.shape[:2], dtype=numpy.int)

    neighborhood = [ (0, -1), (-1, 0), (0, 1), (1, 0) ]
    
    stack = []
    for r in range(cloud.shape[0]):
        for c in range(cloud.shape[1]):
            if mask[r, c]:
                stack.append((r, c))

    while stack:
        (r, c) = stack.pop()

        if not labeled[r, c]:
            label += 1
            labeled[r, c] = label
         
        # now check all the neighbors
        for (nr, nc) in [ (r + dr, c + dc) for (dr, dc) in neighborhood ]:
            if not (0 <= nr < cloud.shape[0] and 0 <= nc < cloud.shape[1]) or not mask[r, c]:
                continue

            # check if this neighbor is connected
            d = sum((cloud[r, c] - cloud[nr, nc])**2)
            if d > threshold:                
                continue

            # recurse
            if not labeled[nr, nc]:
                # label the neighbor
                labeled[nr, nc] = label
                stack.append((nr, nc))

    return (labeled, label)