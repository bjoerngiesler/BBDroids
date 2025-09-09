interp1 = (0, 0.25, 0.5, 0.75, 1.0)
interp2 = (-1, -0.5, 0, 0.5, 1.0)

min = 500
max = 1000

def interp(a, interp):
    frac = (a-min)/(max-min)
    if frac >= 0 and frac < 0.25:
        i0 = interp[0]
        i1 = interp[1]
        frac = frac*4
    elif frac >= 0.25 and frac < 0.5:
        i0 = interp[1]
        i1 = interp[2]
        frac = (frac-0.25)*4
    elif frac >= 0.5 and frac < 0.75:
        i0 = interp[2]
        i1 = interp[3]
        frac = (frac-0.5)*4
    elif frac >= 0.75 and frac < 1:
        i0 = interp[3]
        i1 = interp[4]
        frac = (frac-0.75)*4
    else:
        i0 = 0
        i1 = 0
    a = i0+(i1-i0)*frac
    print(i0, i1, frac, a)

interp(750, interp1)
interp(750, interp2)

