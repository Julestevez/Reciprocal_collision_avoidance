
# Utiliza la camara del drone para detectar el color rojo, azul y amarillo.
# Una vez detecta los colores y calcula sus areas, el drone sigue las directrices
# en funcion del color predominante, el que mas area ocupa.
# Si ese color es azul o amarillo, el drone lo persigue, si es rojo, lo esquiva. 
# 
# 
# raduis area 3000 ta 0.3 0.4 arearangemax

import cv2
import numpy as np
import time, sys
import ps_drone
import imutils
from collections import deque
import math
import pandas as pd

#Declarar funciones
##Dibujar contornos de color en pantalla

def dibujar(mask,color):

    TargetCircleRaduis = 0
    area_pct = 0
    #Contornos en funcion de la mascara: amarillo, azul, rojo.
    contornos,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)        #_,

    for c in contornos:
        
        #Area del contorno en pixeles
        area = cv2.contourArea(c)
        #Porcentaje de area que ocupa el contorno detectado en nuestra pantalla (640x360px)
        area_pct = float(area/(640*360)) 

        #Filtramos las areas pequenas para que dibuje areas validas.
        if area > 2000:                        #500
            M = cv2.moments(c)
            if (M["m00"]==0): M["m00"]=1
            x = int(M["m10"]/M["m00"])
            y = int(M['m01']/M['m00'])
            nuevoContorno = cv2.convexHull(c)
            cv2.circle(frame,(x,y),7,(0,255,0),-1)
            cv2.putText(frame, '{},{},{}'.format(x,y,round(area_pct, 3)),(x+10,y), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,255,0),1,cv2.LINE_AA)
            cv2.drawContours(frame, [nuevoContorno], 0, color, 3)
            Circle = max(contornos, key=cv2.contourArea)
            ((x,y),raduis) = cv2.minEnclosingCircle(Circle)
            Middle = cv2.moments(Circle)
            center = (int(Middle["m10"] / Middle["m00"]), int(Middle["m01"] / Middle["m00"]))
            cv2.line(frame,(320,0),(320,360),(255,0,0), 2)  # linea que divide la mitad de la pantalla                                          

            area_pct = float(math.pi * math.pow(raduis, 2)/(640*360)) #area_pct se calcula como el area del circulo que encierra el area detectada (original)
            
            if raduis >= 10:             #5
                cv2.circle(frame,(int (x),int (y)),int(raduis),(0,0,255),2)
                TargetCircleRaduis = raduis * TargetCircleMultiplayer
                cv2.circle(frame,(int (x),int (y)),int(TargetCircleRaduis),(255,255,0),2)
                cv2.circle(frame,center,5,(0,255,255),-1)
                points.appendleft(center)

            else: 
                cv2.putText(frame,"Error, circulo no encontrado.",(10,700), cv2.FONT_HERSHEY_SIMPLEX, 1,(255,255,255),2,cv2.LINE_AA)
                #CircleLostCount+= 1
            
    return area_pct, TargetCircleRaduis
         
#Colores
azulBajo = np.array([100,100,20],np.uint8)
azulAlto = np.array([125,255,255],np.uint8)
amarilloBajo = np.array([15,100,20],np.uint8)
#amarilloAlto = np.array([45,255,255],np.uint8)
redBajo1 = np.array([0, 100, 20], np.uint8)
redAlto1 = np.array([8, 255, 255], np.uint8)
redBajo2=np.array([175, 100, 20], np.uint8)
redAlto2=np.array([179, 255, 255], np.uint8)

#Controller config
UpdateRate = 1
MaxMovementRatePositive = 0.02
MaxMovementRateNegative = -0.02
DivisionValueX = 14400
DivisionValueY = 9450
TargetCircleMultiplayer = 1 ##valor original 3

#Variables
points = deque(maxlen=32)
counter = 0
(dX, dY) = (0, 0)
direction = ""
Run = True
Direction1 = "error"
Direction2 = "error"
raduis = 0
CircleLostCount = 0
InsideCircle = False
TargetCircleRaduis = 0
col = ""

#Display variables
DisplayDx = 0.0
DisplayDy = 0.0
DisplayTreshhold = 0

#Area y Vel preset

PreMove = 0.2
moveRight = 0
moveForward = 0
moveUp = 0
moveTurnRight = 0
AreaRangeMax = 0.02    #0.02
AreaRangeMin = 0.01    #0.01

area_rojo = 0
#area_azul = 0
#area_amarillo = 0
area_pct = 0
area = 0
VelMultiplyF = 0

#Contador CSV

contadorCSV = 0

#Configuramos el drone
drone = ps_drone.Drone()
drone.startup()
drone.reset()
while (drone.getBattery()[0]==-1): time.sleep(0.1)
print "Battery: "+str(drone.getBattery()[0])+"% "+str(drone.getBattery()[1])
drone.useDemoMode(False)
drone.getNDpackage(["demo"]) 

#Comienza el programa del drone
drone.setConfigAllID()                                 
drone.sdVideo()                                
drone.frontCam()
CDC = drone.ConfigDataCount
while CDC==drone.ConfigDataCount: time.sleep(0.001)

print("starting....")

#Conectar camara del drone con opencv via IP
cap = cv2.VideoCapture('tcp://192.168.1.1:5555')

drone.takeoff()
drone.stop()

#drone.moveForward(0.02)

df = pd.DataFrame(columns = ["altitude" , "theta", "phi", "psi"])

running = True
while running:   

    drone.stop()
    drone.stop()
    drone.stop()
    drone.moveForward(0.1)     

    #Get current frame of video
    running, frame = cap.read()

    center = None
    
    #Cambia color de BGR (pantalla) a HSV (nuestros colores)
    frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    #Mascaras de colores a detectar
    maskAzul = cv2.inRange(frameHSV,azulBajo,azulAlto)
    #maskAmarillo = cv2.inRange(frameHSV,amarilloBajo,amarilloAlto) 
    maskRed1 = cv2.inRange(frameHSV, redBajo1, redAlto1)
    maskRed2 = cv2.inRange(frameHSV, redBajo2, redAlto2)
    maskRed = cv2.add(maskRed1, maskRed2)
    maskRedvis = cv2.bitwise_and(frame, frame, mask= maskRed)

    #Dibujar contornos de colores en pantalla  
    dibujar(maskAzul,(255,0,0))
    #dibujar(maskAmarillo,(0,255,255))
    dibujar(maskRed,(0,0,255))

    area_azul, radio_azul = dibujar(maskAzul,(255,0,0))
    #area_amarillo, radio_amarillo = dibujar(maskAmarillo,(0,255,255))
    #area_rojo, radio_rojo = dibujar(maskRed, (0,0,255))        

    #Visualizar contornos en pantalla
    cv2.imshow('frame', frame)

    # NOTA. REVISAR EL ERROR DE COL="NONE"

    if area_azul > area_rojo and area_azul > 0: # and area_azul > area_amarillo:
        
        TargetCircleRaduis = radio_azul
        area_pct = area_azul
        col = "azul"
        print "AZUL predomina"
    #elif area_amarillo > area_azul and area_amarillo > area_rojo:
    #    TargetCircleRaduis = radio_amarillo
    #    area_pct = area_amarillo
    #    col = "amarillo"
    #    print "AMARILLO predomina"
    elif area_rojo > area_azul and area_rojo > 0: #and area_rojo > area_amarillo:
        
        TargetCircleRaduis = radio_rojo 
        area_pct = area_rojo
        print "ROJO predomina"
        col = "rojo"
    else:

        #area_pct = 0
        col = "none"
        drone.stop()
        drone.moveForward(0.01)
        print "Error!! No encuentra contornos"                  

# --------------------------------------------------------------------------------------------------------------------------------------

    #BLOQUE2: EL DRONE ACTUA SEGUN LO QUE VE.

    DxCount = 0.0
    DyCount = 0.0

    for i in np.arange(1, len(points)):
        if points[i - 1] is None or points[i] is None:
            continue
        #check if 400,400 is in surface of circle ((x,y),raduis) if case then no need to do calculations

        if counter >= 10 and i == 1:
            
            #print("Y points: " + str(points[i][1]))

            DxCount = float(points[i][0])-320 
            DyCount = 170 - float(points[i][1])

            cv2.line(frame, points[i - 1], (320,170), (0, 0, 255), 5)

            #Si el R del contorno es mayor que la distancia entre centros (centro de la pantalla y centro del contorno dibujado), no hace nada.
            if float(math.pow(int(points[i][0]) - 320, 2) + math.pow(int(points[i][1]) - 170, 2)) < float(math.pow(TargetCircleRaduis, 2)):
                InsideCircle = True
            else:
                InsideCircle = False

    #cv2.putText(frame, "Inside Circle" + str(InsideCircle),(10,120), cv2.FONT_HERSHEY_SIMPLEX, 1,(255,0,0),2,cv2.LINE_AA)
    
    if(counter % UpdateRate) == 0:
    
        if area_pct > AreaRangeMax:
            
            VelMultiplyF = math.pow(area_pct - AreaRangeMax, 2) # Cuanto mas lejos este, mas rapido actua. #0.4
            moveForward = - VelMultiplyF  * PreMove # Se mueve hacia atras

        elif area_pct > 0.001 and area_pct < AreaRangeMin: #0.3

            VelMultiplyF = math.pow(area_pct - AreaRangeMax, 2)
            moveForward = + VelMultiplyF * PreMove # Se mueve hacia adelante

        else:

            moveForward = 0 # No se mueve

        # PERSEGUIR: VELOCIDAD X e Y ------------------------------------------------------------------------------

        VelMultiply = math.sqrt(math.pow(DxCount, 2) + math.pow(DyCount, 2)) / (320+170) #resolucion

        # Si detecta rojo (esquivar) las velocidades aumentan mientras mas centrada este el objeto.

        if col == "rojo":

            VelMultiply = 1 - VelMultiply

        else:

            VelMultiply = VelMultiply

        # Moverse izquierda/derecha y arriba/abajo en funcion del cuadran en la que se encuentra.

        if DxCount < 0 and DyCount < 0: # Cuadrante 3
                    
            moveRight = - VelMultiply * PreMove # Ezkerrera
            moveUp = -0.5 * VelMultiply * PreMove # Behera
            #moveTurnRight = -VelMultiply * PreMove
            print "Ezkerra eta behera"
            #cv2.putText(frame, "Objetivo: Ezkerra eta behera",(10,120), cv2.FONT_HERSHEY_SIMPLEX, 1,(255,0,0),2,cv2.LINE_AA)

        elif DxCount < 0 and DyCount > 0: # Cuadrante 2
                    
            moveRight = - VelMultiply * PreMove # Ezkerrera
            moveUp = +0.5 * VelMultiply * PreMove # Gora
            #moveTurnRight = +VelMultiply * PreMove
            print "Ezkerra eta gora"
            #cv2.putText(frame, "Objetivo: Ezkerra eta gora",(10,120), cv2.FONT_HERSHEY_SIMPLEX, 1,(255,0,0),2,cv2.LINE_AA)

        elif DxCount > 0 and DyCount < 0: # Cuadrante 4
                    
            moveRight = + VelMultiply * PreMove # Eskuminera
            moveUp = -0.5 * VelMultiply * PreMove # Behera
            #moveTurnRight = -VelMultiply * PreMove
            print "Eskuminera eta behera"
            #cv2.putText(frame, "Objetivo: Eskuminera eta behera",(10,120), cv2.FONT_HERSHEY_SIMPLEX, 1,(255,0,0),2,cv2.LINE_AA)
    
        elif DxCount > 0 and DyCount > 0: # Cuadrante 1
                    
            moveRight = + VelMultiply * PreMove # Eskuminera
            moveUp = +0.5 * VelMultiply * PreMove # Gora
            #moveTurnRight = +VelMultiply * PreMove
            print "Eskuminera eta gora"
            #cv2.putText(frame, "Objetivo: Eskuminera eta gora",(10,120), cv2.FONT_HERSHEY_SIMPLEX, 1,(255,0,0),2,cv2.LINE_AA)

        print("#----------------------------------------------------------------------------------------------------------------------------------------")  
        print("Right: " + str(round(moveRight, 2)) + "      Forward:" + str(round(moveForward, 2)) + "      Up:" + str(round(moveUp,2)) + "      TurnRight:" + str(round(moveTurnRight,2)))
        print("Area: " + str(area) + "                      AreaPct:" + str(round(area_pct, 3)) + "         Color:" + str(col) + "               VelMultiply:" + str(VelMultiply))
        print("Area Rojo: " + str(round(area_rojo, 3)) + "  Area Azul:" + str(round(area_azul, 3)))
        print("#----------------------------------------------------------------------------------------------------------------------------------------")

        # Mover drone en funcion del color que detecta (rojo = esquivar, azul = perseguir)

        if col == "rojo" and InsideCircle == False:

            # Esquivar
            drone.stop()
            drone.move(-moveRight, moveForward, -moveUp, moveTurnRight)
            print "Rojo esquivar"
            print("Right: " + str(round(-moveRight, 2)) + "      Forward:" + str(round(moveForward, 2)) + "      Up:" + str(round(-moveUp,2)) + "      TurnRight:" + str(round(moveTurnRight,2)))
        
        elif col == "rojo" and InsideCircle == True:
            
            # Esquivar
            drone.stop()
            drone.move(-moveRight, moveForward, -moveUp, moveTurnRight)
            print("Right: " + str(round(-moveRight, 2)) + "      Forward:" + str(round(moveForward, 2)) + "      Up:" + str(round(-moveUp,2)) + "      TurnRight:" + str(round(moveTurnRight,2)))
            print "Rojo esquivar"

        elif col == "azul" and InsideCircle == False:
            
            # Perseguir
            drone.stop()
            drone.move(moveRight, moveForward, moveUp, moveTurnRight)
            print("Right: " + str(round(moveRight, 2)) + "      Forward:" + str(round(moveForward, 2)) + "      Up:" + str(round(moveUp,2)) + "      TurnRight:" + str(round(moveTurnRight,2)))
            print "Azul perseguir"

        elif col == "azul" and InsideCircle == True:

            # Mantenerse estable en el centro
            drone.stop()
            drone.move(0, moveForward, 0, moveTurnRight)
            print("Right: " + str(round(0, 2)) + "      Forward:" + str(round(moveForward, 2)) + "      Up:" + str(round(0,2)) + "      TurnRight:" + str(round(moveTurnRight,2)))
            print "Azul mantenerse"

        else:

            # Mantenerse estable
            drone.stop()
            drone.move(0, 0, 0, 0)
            print("Right: " + str(round(0, 2)) + "      Forward:" + str(round(0, 2)) + "      Up:" + str(round(0,2)) + "      TurnRight:" + str(round(0,2)))
            print "Mantenerse estable"

    #cv2.putText(frame,Direction1,(10,30), cv2.FONT_HERSHEY_SIMPLEX, 1,(255,0,0),2,cv2.LINE_AA)
    #cv2.putText(frame,Direction2,(10,60), cv2.FONT_HERSHEY_SIMPLEX, 1,(255,0,0),2,cv2.LINE_AA)
    #cv2.putText(frame,"Area % Display: " + str(round(area_pct,2)),(10,90), cv2.FONT_HERSHEY_SIMPLEX, 1,(255,0,0),2,cv2.LINE_AA)

    cv2.circle(frame,(320,170),10,(255,0,0),-1)
    cv2.imshow('frame',frame)

    #Exportar resultados

    df.loc[contadorCSV, "altitude"] = str(drone.NavData['demo'][3])
    df.loc[contadorCSV, "theta"] = str(drone.NavData['demo'][2][0])
    df.loc[contadorCSV, "phi"] = str(drone.NavData['demo'][2][1])
    df.loc[contadorCSV, "psi"] = str(drone.NavData['demo'][2][2])

    counter += 1
    contadorCSV += 1 
                                                      
    #Detener el programa pulsando 'esc'.
    if cv2.waitKey(1) & 0xFF == 27: 
            # escape key pressed
            running = False

df.to_csv("output_drone.csv")

cap.release()
cv2.destroyAllWindows()