def coordinates(calib_coords,x_,y_,theta):
    x1,y1,x2,y2 = calib_coords
    tan = np.tan(theta)
    cos = np.cos(theta)
    sec = 1/(np.cos(theta))
    cosec = 1/(np.sin(theta))
    y_tmp = (y_ - y1 - (x_- x1)*tan)*sec
    x_tmp = (x_ - x1)*sec + y_tmp*tan
    s_ = np.sqrt((y2-y1)**2 + (x2-x1)**2)
    dist_fac = s_/s
    x = x_tmp/dist_fac
    y = y_tmp/dist_fac
    
    return x,y
def angle(y1_,y2_,x1_,x2_):
    tmp = float(y2_-y1_)/float(x2_ - x1_)
    invtan = np.arctan(tmp)
    return invtan
