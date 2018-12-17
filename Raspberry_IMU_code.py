import MPU
import time
import math
import threading 
import os
import numpy as np
#---------------configurações de rede-------------------------------
import socket


TCP_IP = '192.168.0.101'
#TCP_IP='192.168.43.214'
TCP_PORT = 7264

BUFFER_SIZE = 1024



#--------------configurações do acelerômetro------------------------


mpu=MPU.MPU(0x68)                   #declaração da instância do sensor
mpu.begin()                         #inicialização, deve ser chamada primeiro
mpu.verbose=True                    #ativa modo verboso
mpu.setAccelRange(MPU.RANGE_2G)     #muda o fundo de escala Acelerômetro
mpu.setGyroScale(MPU.SCALE_250DPS) #muda o fundo de escala Giroscópio
mpu.calibrateGyro(800)             # Rotina para calibração, de Gyro
mpu.calibrateAccel(200) 
mpu.setAccelThreshold(0)            #define o limiar1
mpu.setGyroThreshold(3)           # define limiar
mpu.setupMag(MPU.AK8963_MODE_C100HZ,MPU.AK8963_BIT_16) #inicializa Mag
mpu.setRate(0b000)
mpu.ZACCEL_OFFSET=0#9.80665

def c(angle):
  return np.cos(angle)
def s(angle):
  return np.sin(angle)

  
  
def accel_inertial_frame(v,pitch,roll,yaw):
  psi=yaw
  theta=pitch
  phi=roll  
  #the complete rotation from the body frame to the inertial frame is given by
  RB2I=np.matrix([[ c(psi)*c(theta), c(psi)*s(phi)*s(theta)-c(phi)*s(psi),s(phi)*s(psi)+c(phi)*c(psi)*s(theta)],
                [ c(theta)*s(psi), c(phi)*c(psi)+ s(phi)*s(psi)*s(theta), c(phi)*s(psi)*s(theta)-c(psi)*s(phi)],
                [-1*s(theta),            c(theta)*s(phi),         c(phi)*c(theta)                      ]] )
  
  g=np.matrix( [ [0] ,[0], [-9.80665]  ] )
  am=np.matrix( [[v[0]],[v[1]],[v[2]] ])
  ai=RB2I*am-g
  an=[ai[0,0],ai[1,0],ai[2,0]]  
  #print('an',an)
  return an #aceleração descontada e já no eixo absoluto

def accel_body_frame(v,pitch,roll,yaw):
  psi=yaw
  theta=-pitch
  phi=roll  
  #the complete rotation from the body frame to the inertial frame is given by
  RI2B=np.matrix( [ [c(psi)*c(theta) , c(theta)*s(psi) , -1*s(theta)],
                  [c(psi)*c(phi)*s(theta)-c(phi)*s(psi), c(phi)*c(psi)+s(phi)*s(psi)*s(theta), c(theta)*s(phi)], 
                  [s(phi)*s(psi)+c(phi)*c(psi)*s(theta), c(phi)*s(psi)*s(theta)-c(psi)*s(phi),c(phi)*c(theta)] ] )
  
  g=np.matrix( [ [0] ,[0], [-9.80665]  ] )
  am=np.matrix( [[v[0]],[v[1]],[v[2]] ])
  ab=am-RI2B*g
  an=[ab[0,0],ab[1,0],ab[2,0]]  
  #print('an',an)
  return an #aceleração descontada e já no eixo absoluto  

#velocidade
vbX=0
vbY=0
vbZ=0
#Posição
abX=0
abY=0
abZ=0

#isr to be called every interval time
Phi =0  #Roll
Theta=0 #Pitch
Psi = 0 #Yaw
Ts=0.01#10ms tá bom
Ax=0
Ay=0
Az=0
Wx=0
Wy=0
Wz=0
xmag=1
ymag=1
zmag=1
Heading=0
B=0
Bdef=40
I=0
A=[0,0,0]
flag=False
disc_offset=np.array([0,0,0])
absA=0
vtot=0
ptot=0
stepcount=0
f_r=0
initg=False
Head_med=10*[0]

#buffer de aceleração para verificar posições
abufx=[]
abufy=[]
abufabs=[]
psibuf=[]




try:
  os.remove('/var/www/html/softemb.csv')
except:
  pass

try:
  f=open('/var/www/html/softemb.csv','w')
except:
  print('Deu problema nos arquivos')

if flag==False:
      cnt=0
      sumx=0
      sumy=0
      sumz=0
      smp=100
      while(cnt<smp):
        cnt+=1
        
        adisc=np.array(accel_inertial_frame([Ax,Ay,Az],Theta,Phi,Psi))
        sumx=sumx+adisc[0]
        sumy=sumy+adisc[1]
        sumz=sumz+adisc[2]
      disc_offset=np.array([sumx/smp,sumy/smp,sumz/smp])
      flag=True
 #--------------------------------------------------------------------------------------------------------------------------     
def read():
 
  global read_isr
  global Ts
  del(read_isr)
  read_isr=threading.Timer(Ts,read)
  read_isr.start()
  
  global Phi
  global Theta
  global Psi
  global A
  global Ax
  global Ay
  global Az
  global xmag
  global ymag
  global zmag
  global B
  global Heading
  global abX
  global abY
  global abZ
  global vbX
  global vbY
  global vbZ 
  global flag 
  global disc_offset
  global absA
  global vtot
  global ptot
  global stepcount
  global f_r
  global initg
  global Wx
  global Wy
  global Wz
  global f
  global abufabs ,abufx,abufy
  global psibuf
  
  try:
    W=mpu.readScaledGyro()
    A=mpu.readScaledAccel()
    M=mpu.readMagnet()

        
    alfa=0.2
    absA=alfa*absA+( (math.sqrt(Ax*Ax+Ay*Ay+Az*Az)-9.8)*(1-alfa))#só a parte que não é gravidade
    
    alfa=0.2
    alfaxy=0.6
    Ax=Ax*alfaxy+A[0]*(1-alfaxy)
    Ay=Ay*alfaxy+A[1]*(1-alfaxy)
    Az=Az*alfa-A[2]*(1-alfa)
    alfa=0
    Wx=Wx*alfa+ ((1-alfa)*W[0]*math.pi/180)
    Wy=Wy*alfa+((1-alfa)*W[1]*math.pi/180)
    Wz=Wz*alfa+((1-alfa)*W[2]*math.pi/180)
    ac_angle=0
    B=math.sqrt((M[0]*M[0])+(M[1]*M[1])+(M[2]*M[2]))
    xmag=M[0]
    ymag=M[1]
    zmag=M[2]
    #componentes derivativas
    dPhi = Wx + (Wy * math.sin(Phi)* math.tan(Theta)) + (Wz * math.cos(Phi) * math.tan(Theta))
    dTheta = (Wy*math.cos(Phi))-(Wz*math.sin(Phi))
    #dPsi=( Wy * math.sin(Phi) / math.cos(Theta) )+(Wz*math.cos(Phi) / math.cos(Theta))#melhora depois do fusion
    dPsi=1.5*Wz#Aplica fator de correção aqui
    #acelerometer
    acc_pitch = (math.atan2 (Ax, math.sqrt(Ay*Ay + Az*Az)))
    acc_roll = (math.atan2 (Ay, math.sqrt(Ax*Ax + Az*Az)))

    #calculo condicional de Phi e Theta , se a aceleração é só gravidade, objeto parado ou mru, então pitch e roll podem ser os do axcel
    Amod=math.sqrt(Ax*Ax+Ay*Ay+Az*Az)
    if(abs(Amod-9.80665)<=0.5):
      Phi=acc_roll
      Theta=-acc_pitch
    else:
      Phi= (Phi+dPhi*Ts)
      Theta=(Theta+dTheta*Ts)     
    

    #funciona relativamente bem
    if(xmag!=0):
      Heading=math.atan2(ymag,xmag)
    

    
    #condicional yaw - imunidade a ruido magnético
    if (initg==False): #---- se é a inicialização, então Psi começa igual a head, é necessario
      Psi=Heading
      initg=True
      
    if((abs(B-Bdef)<15)and  (abs(Heading)<1*np.pi/180) ) :
      Psi=Heading
    else:
      Psi=(Psi+dPsi*Ts)#*0.8+Heading*0.2#-0.001*(Heading)        #Yaw   ----isso funciona
    
    
    

    # passa todos angulos para -180 a 180,
    if(Psi>math.pi):
      Psi=-((2*math.pi)-Psi)
    if(Psi<-math.pi):
      Psi=(2*math.pi)+Psi  
      
    #adisc=np.array(accel_inertial_frame([Ax,Ay,Az],Theta,Phi,Psi))
    #adisc=adisc-disc_offset
    #lim=0.9
    #if(abs(adisc[0])<lim):
      #adisc[0]=0
    #else:
      #vbX=vbX+adisc[0]*Ts
      ##abX=abX+vbX*Ts  
    #if(abs(adisc[1])<lim):
      #adisc[1]=0
    #else:
      #vbY=vbY+adisc[1]*Ts
      ##abY=abY+vbY*Ts  
    #if(abs(adisc[2])<lim):
      #adisc[2]=0
    #else:
      #vbZ=vbZ+adisc[2]*Ts
      #abZ=abZ+vbZ*Ts
    
    #detecção da direção da aceleração em relação ao azimute do usuário----novidade
    abufy.append(Ay)
    abufx.append(Ax)
    abufabs.append((Ay*Ax))
    psibuf.append(Psi)
    
    #----------------------------------------------------função de detecção de passo e tratamento de comprimento,e direção do passo
    f_r+=1
    ac_angle=0
    #f_Psi=1.5*Psi# Psi aplicado fator de correção
    if(abs(Az)>11)and(f_r>50): #ay 10
      p_t=f_r*Ts#periodo da passada
      p_l=0.3+p_t*0.7#tamanho da passada
      if p_t>1:#--------------------------------------------ver isso
        p_l=0
        
      
      stepcount+=1
      
      ##checa aceleração 
      #try:
        #nabuf=[]
        #for k,i in enumerate(abufy):
            #print(i,' ',k)
            #nabuf.append(-math.atan2(abufy[k],abufx[k])+math.pi/2)
        #w=np.array(abufabs)/max(abufabs)
        #ac_angle=np.average(np.array(nabuf),weights=w)
        ##print(ac_angle)
        #abufx=[]
        #abufy=[]
        #abufabs=[]  
      #except:
        #pass
        #print('não deu para corrigir aceleração')
      #média de psi entre passos
      psi_mod=Psi #por padrão, psi_modificado é Psi instantaneo
      wp=range(len(psibuf))
      wp=np.array(wp)/max(wp)#peso aumento linear
      #wp=np.array([1]*len(psibuf))#peso uniforme
      try:
        psi_mod=np.average(np.array(psibuf),weights=wp)
        psibuf=[]
      except:
        pass
     # abX=abX-math.sin(Psi) #ORIGINAIS
     # abY=abY+math.cos(Psi) #ORIGINAIS
      abX=abX-math.sin(psi_mod)*p_l #PONDERADAS
      abY=abY+math.cos(psi_mod)*p_l #PONDERADAS
      initg=True
      message=str(f_r)+','+str(Psi) #---------------------------------------envio via socket
      f_r=0#zera timer de intervalo de passada
      s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
      s.connect((TCP_IP,TCP_PORT))
      s.send(message.encode())
      s.close
      
    
     
      
     

  except: 
    print("pau")
    pass
#---------------------------------------------------------------------------------------------------------------------------------
def print_gyro():

  global Phi
  global Theta
  global Psi
  global A
  global Ax
  global Ay
  global Az
  global xmag
  global ymag
  global zmag
  global B
  global Heading
  global abX
  global abY
  global abZ
  global vbX
  global vbY
  global vbZ 
  global flag 
  global disc_offset
  global absA
  global vtot
  global ptot
  global stepcount
  global f_r
  global initg
  global Wx
  global Wy
  global Wz
  global f
  global strx
  
  print_isr=threading.Timer(0.1,print_gyro)
  print_isr.start()
  os.system('clear')
  print('pitch:',round(Theta*180/math.pi,2),' roll:',round(Phi*180/math.pi,2),' yaw:',round(Psi*(180/math.pi),4), 'Head',round(Heading*180/math.pi,4))  
  print('ax:',round(Ax,3),'ay:',round(Ay,3),'az:',round(Az,3))
  print('|A|=',round(math.sqrt(Ax*Ax+Ay*Ay+Az*Az),3))
  #print('vx:',round(vbX,3),'vy:',round(vbY,3),'vz:',round(vbZ,3))
  #print('px:',round(abX,3),'py:',round(abY,3),'pz:',round(abZ,3))
  print('Bx:',round(xmag,3),'By:',round(ymag,3),'Bz:',round(zmag,3)) 
  print('|B|=',B)
  print('Posições absolutas:')
  print('X=',abX)
  print('Y=',abY)
  print('Z=',abZ)
  ad=accel_body_frame([Ax,Ay,Az],Theta,Phi,Psi)

  I=math.asin(zmag/B) *180/math.pi
  #print('I=',I)
  #print('atot:',round(absA,3))
  #print('comprimento:',round(ptot,3))
  print('Passos:', stepcount)
  #Atualização do csv px,py,pz,vx,vy,vz,ax,ay,az,wx,wy,wz,Heading,B
  csv_line=str(round(abX,2))+','+str(round(abY,2))+','+str(round(abZ,2))+','+str(round(vbX,2))+','+str(round(vbY,2))+','+str(round(vbZ,2))+','+str(round(Ax,2))+','+str(round(Ay,2))+','+str(round(Az,2))+','+str(round(Wx,2))+','+str(round(Wy,2))+','+str(round(Wz,2))+','+str(round(Heading*180/math.pi,2))+','+str(round(B,2))+'\r\n'
  f.write(csv_line)


#Create a time interruption to read in regular intervals
read_isr=threading.Timer(Ts,read)#50hz é um valor bom, infelizmente
read_isr.start()

print_isr=threading.Timer(0.2,print_gyro)
print_isr.start()

#!/usr/bin/env python








