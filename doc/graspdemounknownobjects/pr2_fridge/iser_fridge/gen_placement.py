import glob
import copy
import random 

dats = glob.glob('../office6/office6-emptyplacing.dat')
places=[[]]*len(dats)
for i,f in enumerate(dats):
    print f
    fin = open(f,'r')
    places[i] = []
    for line in fin.readlines():
        s = line.strip().split('\t')
        print s
        if (not s):
            continue
        places[i].append(s)
    print len(places[i])
    fin.close()        

fout = open('placement.txt','w')    
for i in range(0,len(places[0])):
    fout.write(places[0][i][0]+'\n') 
    for j in range(0,len(places)):
        for k in range(1,5):
            fout.write(places[j][i][k]+'\t')
        fout.write(str(-random.random()*10)+'\n')
        
fout.close()            

    

 
