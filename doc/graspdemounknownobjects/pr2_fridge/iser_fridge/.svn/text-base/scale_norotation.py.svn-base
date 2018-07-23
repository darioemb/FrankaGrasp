dat = open('obj-to-scale.dat','r')

def isnum(s):
    try:
        float(s)
        return True
    except ValueError:
        return False

objset = set()

for line in dat.readlines():
  s = line.split('\t')
  if (s[0] != "Name"):
    f = open('%s.obj' % s[0], 'r')
    if (s[0] in objset):
        continue
    else:
        objset.add(s[0])    
    x_dat = float(s[1])/1000
    y_dat = float(s[2])/1000
    z_dat = float(s[3])/1000
    mn_x = mn_y = mn_z = float('inf') 
    mx_x = mx_y = mx_z = float('-inf')
    for ln in f.readlines():
      words = ln.split()
      if (len(words) != 0):
        if (words[0] == 'v'):
          if float(words[1]) < mn_x:
            mn_x = float(words[1])
          if float(words[1]) > mx_x:
            mx_x = float(words[1])
          if float(words[2]) < mn_y:
            mn_y = float(words[2])
          if float(words[2]) > mx_y:
            mx_y = float(words[2])
          if float(words[3]) < mn_z:
            mn_z = float(words[3])
          if float(words[3]) > mx_z:
            mx_z = float(words[3])

    print mn_x, mn_y, mn_z
    print mx_x, mx_y, mx_z

    print mx_x - mn_x 
    print mx_y - mn_y 
    print mx_z - mn_z 
   
    print x_dat, y_dat, z_dat

    print (mx_x - mn_x) / x_dat
    print (mx_y - mn_y) / y_dat 
    print (mx_z - mn_z) / z_dat 

    f = open('%s.obj' % s[0], 'r')
    nf = open('%s-scaled.obj' % s[0], 'w')
    while True:
        l = f.readline()
        if len(l) == 0:
          break
        words = l.split()
        if len(words) is 0:
          nf.write(l)
          continue
        if ((words[0] == 'vn') or (words[0] == 'v')):
          count = 0
          if isnum(words[1]):
            scaledx = (float(words[1])-mn_x) * (x_dat/(mx_x - mn_x))-(x_dat/2)
          if isnum(words[2]):
            scaledy = (float(words[2])-mn_y) * (y_dat/(mx_y - mn_y)) -(y_dat/2)
          if isnum(words[3]):
            scaledz = (float(words[3])-mn_z) * (z_dat/(mx_z - mn_z))
          nf.write(words[0]+' '+str(scaledx)+' '+str(scaledy)+' '+str(scaledz)+'\n')
        else:
          nf.write(l)
    nf.close()
    f.close()
    break
