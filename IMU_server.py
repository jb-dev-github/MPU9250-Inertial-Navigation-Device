import socket
from threading import Thread
import matplotlib.pyplot as plt
import numpy as np
import time
import math
import os
f_r=[]
Psi=[]
p_l=[]
p_t=[]
abX=[]
abY=[]
stepcount=0
lin_dist=0
a=0.3#coeficiente linear
b=0.8#coeficiente angular
c=1.3#saturação do passo
d=1.3#COEFICIENTE DE REOTORNO
Ts=0.01
map_resolution=0#graus
v=1.0
driftcorr=0
driftcorrx=0#contador para iterar a correção
xc=1
yc=1
box=0
#angle parametro em radiano, scale1,scale2,e step em graus pois é necessário para o range
def rag(angle,scale1,scale2,step):#funçao para converter angulos para intervalos discretos
    if step==0:
      return angle
    angle=angle*180/math.pi
    map_array=range(scale1,scale2,step)
    adist=[]
    for i,k in enumerate(map_array):
      adist.append(abs(angle-map_array[i]))
    return map_array[adist.index(min(adist))]*math.pi/180  


def server():
    global stepcount,a,b,c,Ts,f_r,Psi,p_l,p_t,abX,abY
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(('', 7264))
    s.listen(1)
    conn, addr = s.accept()
    print(addr)
    while 1:
        conn, addr = s.accept()
        #print(addr)
        data = conn.recv(1024)
        #conn.sendall(data)
        str_data=data.decode()
        if(str_data!=""):#----------------plotar o dado recebido
        #  print( 'Received on server', str_data)
          splited=str_data.split(',')
          f_r.append(float(splited[0]))
          Psi.append(float(splited[1])) # usando o Psi diretamente ou
          stepcount+=1
        
    conn.close()


  
def calculate():
  global stepcount,a,b,c,Ts,f_r,Psi,p_l,p_t,abX,abY,map_resolution,v,driftcorr,lin_dist,xc,yc
  try:
    l=len(Psi)
    p_l=[0]*l
    p_t=[0]*l
    abX=[0]*l
    abY=[0]*l
    Psi_tr=[0]*l
    for i,k in enumerate(Psi):
      Psi_tr[i]=rag((v*Psi[i]),-180,180,map_resolution)
      if Psi_tr[i]>=0:
        Psi_tr[i]=Psi_tr[i]-driftcorr*i
      else:
        Psi_tr[i]=Psi_tr[i]+driftcorr*i
      p_t[i]=f_r[i]*Ts#periodo da passada
      p_l[i]=a+p_t[i]*b#tamanho da passada
      lin_dist=sum(p_l)
      if p_t[i]>c:
        p_l[i]=d
      # abX=abX-math.sin(Psi) #ORIGINAIS
      # abY=abY+math.cos(Psi) #ORIGINAIS
      if(i==0):
        abX[i]=0-math.sin(Psi_tr[i])*p_l[i] #PONDERADAS
        abY[i]=0+math.cos(Psi_tr[i])*p_l[i] #PONDERADAS
      else:
        abX[i]=abX[i-1]-math.sin(Psi_tr[i])*p_l[i]*xc #PONDERADAS
        abY[i]=abY[i-1]+math.cos(Psi_tr[i])*p_l[i]*yc #PONDERADAS
  except:
    pass
    
def dialog():
  global stepcount,a,b,c,d,Ts,f_r,Psi,p_l,p_t,abX,abY,map_resolution,v,driftcorr,lin_dist,xc,yc,box
  arg=''
  while(1):
    print('Passos dados: ',stepcount)
    try:
      print('Distância percorrida: ',lin_dist)
      print('Posição: ',str(abX[len(abX)-1]),' , ',str(abY[len(abY)-1]))
      print('Distância à origem: ', str ( math.sqrt( (abX[len(abX)-1]*abX[len(abX)-1] )+ (abY[len(abY)-1]*abY[len(abY)-1]))) ) 
    except:
      print('Posição: ',0,' , ',0)
      
    print('Último comando: ', arg)
    arg=input("Digite um comando (a,b,c,d,r): ")
    arg_s=arg.split(' ')
    if(arg_s[0]=='a'):
      a=float(arg_s[1])
    if(arg_s[0]=='b'):
      b=float(arg_s[1])
    if(arg_s[0]=='c'):
      c=float(arg_s[1])
    if(arg_s[0]=='d'):
      d=float(arg_s[1])
    if(arg_s[0]=='v'):
      v=float(arg_s[1])
    if(arg_s[0]=='m'):
      map_resolution=int(arg_s[1])
    if(arg_s[0]=='do'):
      driftcorr=float(arg_s[1])
    if(arg_s[0]=='x'):
      xc=float(arg_s[1]) 
    if(arg_s[0]=='y'):
      yc=float(arg_s[1])
    if(arg_s[0]=='box'):
      box=float(arg_s[1]) 
    os.system('clear')
      
   # b=float(input("Digite o coeficiente linear (default 0.3): "))
    #a=float(input("Digite o coeficiente angular (default 0.7): "))
    #c=float(input("Digite o coeficiente de saturação (default 1.3): "))
    #d=float(input("Digite o coeficiente de retorno (default 1.3): "))
    
    if(arg_s[0]=='r'):
      f_r=[]
      Psi=[]
      p_l=[]
      p_t=[]
      abX=[]
      abY=[]
      stepcount=0
      lin_dist=0
    

def disp():
  global stepcount,a,b,c,Ts,f_r,Psi,p_l,p_t,abX,abY,box,lin_dist
  fig = plt.gcf()
  fig=plt.figure(figsize=(100,100))#full screen
  fig.show()
  fig.canvas.draw()
  
  while(1):
      time.sleep(0.2)
      calculate()
      plt.clf()
      if(box!=0):
        plt.xlim([-box, box])
        plt.ylim([-box, box])
        
      plt.gca().set_aspect('equal', adjustable='box')
      plt.plot(abX, abY) # plot something
      if(abX!=[]):
        strtitle='Deslocamento   (Passos: '+str(round(stepcount,0))+' Distância: '+str(round(lin_dist,1))+ ' |r|:'+ str(round( math.sqrt( (abX[len(abX)-1]*abX[len(abX)-1] )+ (abY[len(abY)-1]*abY[len(abY)-1])) ,1 ))+')'
      else:
        strtitle='Deslocamento   (Passos: '+str(round(stepcount,0))+' Distância: '+str(round(lin_dist,1))+')'

              
      plt.title(strtitle)
      plt.ylabel('y        (m)')
      plt.xlabel('x        (m)')
      # update canvas immediately
      
      #plt.pause(0.01)  # I ain't needed!!!
      fig.canvas.draw()
      #fig.savefig('temp.png', transparent=True)
      #fig.savefig('temp.png')


server_thread=Thread(target=server,args='')
server_thread.start()

input_thread=Thread(target=dialog,args='')
input_thread.start()

disp_thread=Thread(target=disp,args='')
disp_thread.start()
