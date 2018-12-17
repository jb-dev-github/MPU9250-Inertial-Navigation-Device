import matplotlib.pyplot as plt
import csv
import numpy as np
import urllib.request
import time
import os
x = []
y = []


cx=0#coluna que conterá dados do eixo x
cy=1#coluna que conterá dados do eixo y


#para eliminar pontos repitidos

def read_csv():
  global x,y
  #como usa append tem que esvaziar as lista a cada iteração
  x = []
  y = []
  cnt=0
  #Baixar arquivo csv
  urllib.request.urlretrieve('http://192.168.0.107/softemb.csv','softemb.csv')
  #os.system("./wget.sh")
  #print('url')
  csvfile= open('softemb.csv','r')
  a=open('softemb.csv','r')
  
  plots = csv.reader(csvfile, delimiter=',')
  plot_length= int(len(a.readlines()))
  print(plot_length)
  
  for i,row in enumerate(plots):
       if(i>0)and(i<plot_length):
           #x.append(cnt) #descomentar caso queira somente a série de amostras
           x.append(float(row[cx]))
           y.append(float(row[cy]))
           cnt=cnt+1
  csvfile.close
  a.close
  os.system("rm softemb.csv")
  
  
read_csv()


fig = plt.gcf()
fig.show()
fig.canvas.draw()

while 1:
  time.sleep(1)
  
 
  read_csv()
  plt.clf()
  plt.plot(x, y) # plot something
  
  # update canvas immediately
  #plt.xlim([0, 100])
  #plt.ylim([0, 100])
  #plt.pause(0.01)  # I ain't needed!!!
  fig.canvas.draw()

  
  



print('p')

#plt.subplot(1,2,1)


