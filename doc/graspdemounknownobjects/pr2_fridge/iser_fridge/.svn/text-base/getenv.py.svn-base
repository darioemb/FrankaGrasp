import math

dat = open('exp.dat','r')
env = open('exp.env.xml', 'w')

env.write('<Environment>\n\t<KinBody name="floor">\n\t\t<Body type="static">\n\t\t<Translation>0 0 0</Translation>\n\t\t\t<Geom type="box">\n\t\t\t\t<extents>3.5 3.5 0.005</extents>\n\t\t\t\t<translation>0 0 -0.005</translation>\n\t\t\t\t<diffuseColor>.6 .6 .6</diffuseColor>\n\t\t\t</Geom>\n\t\t</Body>\n\t</KinBody>\n\n')
env.write('\
    <Robot file="robots/pr2-beta-sim.robot.xml" name="Kodiak">\n\
        <translation>.6 1.0 0.01</translation>\n\
	    <RotationAxis>0 0 1 270.0</RotationAxis>\n\
    </Robot>\n')

cnt = 0
x0 = 0
y0 = 0
for line in dat.readlines():
  s = line.split('\t')
  if (s[0] != "Name"):
    print s[0]
    obj = s[0]
    cnt += 1
    objname = obj+str(cnt)
    x = -float(s[1])/100
    y = float(s[2])/100
    z = float(s[3])/100
    if (cnt==1):
        x0 = x
        y0 = y
    x = x-x0
    y = y-y0        
    ang = float(s[4].rstrip('\n'))/math.pi*180
    env.write('\t<KinBody name="'+objname+'" file="env/'+obj+'-scaled.obj">\n')
#    env.write('\t\t<Translation>'+str(0)+' '+str(0)+' '+str(0)+'</Translation>\n')
    env.write('\t\t<Translation>'+str(x)+' '+str(y)+' '+str(z)+'</Translation>\n')
    env.write('\t\t<RotationAxis>0 0 1 '+str(ang)+'</RotationAxis>\n')
#    env.write('\t\t<RotationAxis>1 0 0 90</RotationAxis>\n')
    env.write('\t</KinBody>\n')

env.write('</Environment>')
env.close()
dat.close()
    
    
