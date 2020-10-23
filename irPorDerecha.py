from controller import Robot
import math
import struct
import cv2
import numpy as np
import colorsys

class RobotRescate:

    #propiedades de clase - TODOS LOS OBJETOS DE ESTA CLASE TIENEN LOS MISMOS VALORES EN ESTAS VARIABLES
    __timeStep = 32
    __max_velocity = 6.28
    

    # Threshold for the victim being close to the wall
    __victimProximity = 0.09

    # Set RGB colours of the swamp and hole to avoid them 
    # These should be calibrated to match the environment
    __hole_colour = b'\x1e\x1e\x1e\xff'
    __swamp_colour = b'R\x89\xa7\xff'
    __swampV = 222
    __holeV=90

    __victimDetectionTime = 3.2

    #inicializacion. Cuando haga var=RobotRescate(), es decir, cuando cree un objeto de esta clase, se va a ejecutar todo lo que esta en init

    def __init__(self):
        #propiedades de objeto
        self.__robot = Robot()

        #que tenga __ al comienzo hace que la propiedad sea privada, no se puede ver desde afuera, solo
        #desde metodos que esten dentro de la clase
        #en ruedaIzquierda y ruedaDerecha guardamos las ruedas del robot
        #despues voy a poner metodos que las usen, pero sin exponer hacia afuera las ruedas
        #la unica forma de usarlas es con esos metodos publicos que hago despues
        #si en algun momento cambia algo de las ruedas, no voy a afectar nada hacia afuera
        #yo mantengo oculto el lio aca adentro y hacia afuera doy el mismo servicio
        #mientras me manden setVel o algun otro, yo me arreglo aca adentro como hacerlo
        self.__ruedaIzquierda = self.__robot.getMotor("left wheel motor")
        self.__ruedaDerecha = self.__robot.getMotor("right wheel motor")

        #voy a hacer lo mismo con todos los componentes del robot
        #los voy a ocultar aca adentro y hacia afuera voy a brindar metodos para que los puedan usar
        #pero sin saber como los estan usando
        #eso me permite desacoplar a los que usan los robots de como es el robot por dentro
        #imaginen una tele, uds no acceden a los circuitos
        #acceden a ciertos metodos (cambiar de canal, subir el volumen, etc)
        #si la tele cambia por dentro como lo hace, no nos enteramos
        #mientras lo que me ofrezca hacia afuera sea lo mismo y de la misma manera
        self.__camaraI=self.__robot.getCamera("camera_left")
        #self.__camaraI.enable(self.__timeStep)

        self.__camaraC=self.__robot.getCamera("camera_centre")
        #self.__camaraC.enable(self.__timeStep)

        self.__camaraD=self.__robot.getCamera("camera_right")
        self.__camaraD.enable(self.__timeStep)

        self.__colorPiso = self.__robot.getCamera("colour_sensor")
        self.__colorPiso.enable(self.__timeStep)

        self.__emisor = self.__robot.getEmitter("emitter")

        self.__gps = self.__robot.getGPS("gps")
        self.__gps.enable(self.__timeStep)

        self.__tempI = self.__robot.getLightSensor("left_heat_sensor")
        self.__tempD = self.__robot.getLightSensor("right_heat_sensor")

        self.__tempI.enable(self.__timeStep)
        self.__tempD.enable(self.__timeStep)

        self.__distI = []
        self.__distD = []
        self.__distF = []

        self.__distF.append(self.__robot.getDistanceSensor("ps7"))
        self.__distF[0].enable(self.__timeStep)
        self.__distF.append(self.__robot.getDistanceSensor("ps0"))
        self.__distF[1].enable(self.__timeStep)

        self.__distD.append(self.__robot.getDistanceSensor("ps1"))
        self.__distD[0].enable(self.__timeStep)
        self.__distD.append(self.__robot.getDistanceSensor("ps2"))
        self.__distD[1].enable(self.__timeStep)

        self.__distI.append(self.__robot.getDistanceSensor("ps5"))
        self.__distI[0].enable(self.__timeStep)
        self.__distI.append(self.__robot.getDistanceSensor("ps6"))
        self.__distI[1].enable(self.__timeStep)

        self.__ruedaIzquierda.setPosition(float("inf"))
        self.__ruedaDerecha.setPosition(float("inf"))

        self.__vd=0
        self.__vi=0

        self.__messageSent = False
        self.__victimDetected = False
        self.__lastVictimTime=0
        self.__numero=0

    #Metodos de velocidad de las ruedas
    #al armar los metodos de esta manera dejo ocultas las ruedas. Como funcionan las ruedas no les importa a los que usan el robot
    def setVel(self, vi, vd):
        self.setVI(vi)
        self.setVD(vd)

    def setVI(self, vi):
        self.__vi=vi*self.__max_velocity/100
        self.__ruedaIzquierda.setVelocity(self.__vi)

    def setVD(self, vd):
        self.__vd=vd*self.__max_velocity/100
        self.__ruedaDerecha.setVelocity(self.__vd)

    def getVI(self):
        return self.__vi*100/self.__max_velocity

    def getVD(self):
        return self.__vd*100/self.__max_velocity

    #Espera
    def esperar(self, duracion):
        inicio = self.tiempoActual()
        while (self.tiempoActual() - inicio) < duracion:
            self.step()

    def tiempoActual(self):
        return self.__robot.getTime()

    #funciones útiles
    def __mapeo(self, val, min, max):
        return int((val - min) * 100 / (max - min))

    #sensores
    def getColorPiso(self):
        return self.__colorPiso.getImage()

    def getColorPisoHSV(self):
        colour = self.getColorPiso()
        img = np.array(np.frombuffer(colour, np.uint8).reshape((self.__colorPiso.getHeight(), self.__colorPiso.getWidth(), 4)))
        img[:,:,2] = np.zeros([img.shape[0], img.shape[1]])
        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)[0][0]
        return hsv

    def estoyViendoPantano(self):
        hsv=self.getColorPisoHSV()
        if ( hsv[2]> self.__swampV-10 and hsv[2] < self.__swampV+10) :
            return True
        else:
            return False

    def estoyViendoPozo(self):
        hsv=self.getColorPisoHSV()
        if ( hsv[2] < self.__holeV+5 ):
            return True
            #print("pozo")
        else:
            return False
        


    def getCamaraI(self):
        return self.__camaraI.getImage()

    def getCamaraC(self):
        return self.__camaraC.getImage()

    def getCamaraD(self):
        return self.__camaraD.getImage()

    def getGps(self):
        return self.__gps.getValues()

    def getTempI(self):
        return self.__tempI.getValue()        

    def getTempD(self):
        return self.__tempD.getValue()        

    def getDistI(self, num):
        return self.__distI[num].getValue()

    def getDistF(self, num):
        return self.__distF[num].getValue()

    def getDistD(self, num):
        return self.__distD[num].getValue()

    def step(self):
        return (self.__robot.step(self.__timeStep) != -1)

    #mensajes
    def enviarMensaje(self, v1, v2, victimType):
        print(victimType.encode())
        message = struct.pack('i i c', v1, v2, victimType.encode())
        self.__emisor.send(message)

    # Sents a message of the game controller that a victim (of a certain type) has been detected
    def enviarMensajeDeVictima(self, victimType='H'):
        position = self.getGps()

        if not self.__messageSent:
            #robot type, position x cm, position z cm, victim type
            # The victim type is hardcoded as "H", but this should be changed to different victims for your program
            # Harmed = "H"
            # Stable = "S"  
            # Unharmed = "U"
            # Heated (Temperature) = "T"
            self.enviarMensaje(int(position[0] * 100), int(position[2] * 100), victimType)
            self.__messageSent = True

    
    #Deteccion de victimas
    def cercaDeObjeto(self, position):
        return position < self.__victimProximity

    # Detecta victimas visuales (no de temperatura)
    # vamos a mantener privado este metodo, lo usamos solo internamente
    def __deteccionVisualSimple(self, img):
        
        coords_list = []
        bg = (img[:, :, 0] == img[:, :, 1]) & (img[:,:,0]>199) # Azul == Verde y blanco
        #bg2= (img[:, :, 0] == img[:, :, 1]) #Este es para que quede el cuadro blanco sin la letra
        gr = (img[:, :, 1] == img[:, :, 2]) # Verde == Rojo y no pozo
        enBlanco = np.bitwise_and(bg, gr, dtype=np.uint8) * 255
        #apply threshold
        #thresh = cv2.threshold(image_data, 140, 255, cv2.THRESH_BINARY)[1]

        # draw all contours in green and accepted ones in red
        contours, h = cv2.findContours(enBlanco, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        for c in contours:
            if cv2.contourArea(c) > 400:
                coords = list(c[0][0])
                coords_list.append(coords)
                #nombre="imagenes/imagen"+(f'{self.__numero:04}').strip()+".png"
                #cv2.imwrite(nombre, img)
                #self.__numero+=1

        return coords_list
 
    def getVictimasVisibles(self):
        victims= []
        img = np.fromstring(self.getCamaraD(), dtype=np.uint8)
        img = img.reshape((128, 128,4))
        coords = self.__deteccionVisualSimple(img)
        distance_to_wall = self.getDistF(1)

        for coord in coords:
            victims.append([distance_to_wall,coord])

        return victims

    # Get visible victims using either the right or left temperature sensor
    def pararEnVictimasTemp(self):
        #print(self.getTempI(), self.getTempD())
        
        if self.getTempI() > 37 or self.getTempD() > 37:
            self.parar()
            self.enviarMensajeDeVictima('T')
            
            # print("Found heated victim!!")
            # self.__victimDetected = True
            self.esperar(self.__victimDetectionTime)
            #self.girar(60)
            #self.esperar(0.5)
            # self.__victimDetected = False
        else:
            self.__messageSent = False


    # Steer the robot towards the victim
    def girarHaciaVictima(self, victim):
        # [x,y]
        position_on_image = victim[1]

        width = self.__camaraD.getWidth()
        center = width / 2

        victim_x_position = position_on_image[0]
        dx = center - victim_x_position

        if dx < 0:
            self.girarDerechaVictima()
        else:
            self.girarIzquierdaVictima()

    # Return the victim that is closest to you
    # No tiene sentido que sea público
    def __getVictimaMasCercana(self, victims):
        shortestDistance = 999
        closestVictim = []

        for victim in victims:
            dist = victim[0]
            if dist < shortestDistance:
                shortestDistance = dist
                closestVictim = victim

        return closestVictim

    # Stop at a victim once it is detected
    def pararEnVictimaVisible(self):
        #Espero N segundos para detectar otra víctima así no detecto la misma dos veces
        if(self.tiempoActual()-self.__lastVictimTime<2):
            return
        self.__victimDetected=False

        #get all the victims the camera can see
        victims = self.getVictimasVisibles()
        
        foundVictim = False

        if len(victims) != 0:
            closest_victim = self.__getVictimaMasCercana(victims)            
            self.girarHaciaVictima(closest_victim)

        #if we are near a victim, stop and send a message to the supervisor
        for victim in victims:
            
            if self.cercaDeObjeto(victim[0]) and not foundVictim and not self.__victimDetected:
                self.parar()
                #tipoDeVictima=self.decidirTipoVictima()
                tipoDeVictima=self.ReconocimientoVictima()
                if(tipoDeVictima!='R'):
                    self.enviarMensajeDeVictima(tipoDeVictima) # <- Put detected victim type here
                    self.esperar(self.__victimDetectionTime)
                foundVictim = True
                self.__victimDetected = True
                self.__lastVictimTime=self.tiempoActual()
                
        if not foundVictim:
            self.__messageSent = False
            
    def ReconocimientoVictima(self):
        tipoDeVictima='H'

        img_victima=self.RecorteVictima()
        
        arry=np.array(img_victima)
        inv=arry[::-1]
        h=0
        for indx, x in np.ndenumerate(arry):
            if x<20 and (indx[0]>3 and (indx[1]>3)):
                h, w = indx
                break
        #print("Encontro primero en:", h)
        if(h==0):
            return 'R'


        for indx, x in np.ndenumerate(inv):
            if (x < 20) and (indx[0]>3) and (indx[1]>3):
                k, v = indx
                break
        
        h_end = (arry.shape[1])-k
        w_end = int((((arry.shape[0])-v)+w))

        roi = img_victima[h:h_end, w:w_end]
        #cv2.imshow("image2", roi)
        #cv2.moveWindow("image2", 200,200)
        #nombre="imagenes/imagen"+(f'{self.__numero:04}').strip()+".png"
        #cv2.imwrite(nombre, roi)
        #self.__numero+=1

        primera=roi[:,0]
        primeraNegra=np.count_nonzero(primera<20)
        tipoDeVictima='H'
        if(len(primera)!=primeraNegra):
            tipoDeVictima='S'
            return tipoDeVictima
        quinta=roi[:,6]
        quintaNegra=np.count_nonzero(quinta<200)
        print(quinta)
        if(quintaNegra<4):
            tipoDeVictima='U'
            

        #cv2.waitKey(0)
        #cv2.destroyAllWindows()
        
        return tipoDeVictima

    def RecorteVictima(self):
        camara=self.getCamaraD()
        capturaCamara = np.array(np.frombuffer(camara, np.uint8).reshape((128,128, 4)))

        bnw_captura=cv2.cvtColor(capturaCamara, cv2.COLOR_BGR2GRAY)
        ret, thresh1 = cv2.threshold(bnw_captura, 127, 255, cv2.THRESH_BINARY)
        contours, hierarchy = cv2.findContours(thresh1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            x, y, w, h = cv2.boundingRect(cnt)
            #cv2.rectangle(bnw_captura, (x, y), (x+w, y+h), (0, 255, 0), 3)
        victims_aux = []
        img_h, img_w = bnw_captura.shape
        for cnt in contours:
            x, y, w, h = cv2.boundingRect(cnt)
            if (img_w - w) < 20 or (img_h - h) < 20:
                continue
            if w < 20 or h < 20:
                continue
            victims_aux.append((x, y, h, w))
        box=""
        if len(victims_aux) > 0:
            x, y, h, w = victims_aux[0]
            box = bnw_captura[y:y+h, x:x+w]
        else:
            print("Ninguna victima encontrada")
        return box


    # Avoid holes and swamps by looking at the RBG colour of the camera
    def esquivarMosaicos(self):
        colour = self.getColorPiso()

        if colour == self.__hole_colour or colour == self.__swamp_colour:
            self.moverAtras()
            self.esperar(2)

    # avoid tiles uing the HSV decomposition of the colour camera instead of a single value. Requires opencv installation
    def esquivarMosaicosHSV(self):
        hsv = self.getColorPisoHSV()

        # Change the range at which the robot detects the swamps and holes
        #       SWAMP                           HOLE
        if (self.estoyViendoPozo()):
            self.moverAtras()
            self.esperar(2)
            

    #Metodos de movimiento del robot (son de mas alto nivel que los de las ruedas)
    def girarDerechaVictima(self):
        self.setVel(100,40)

    def girarIzquierdaVictima(self):
        self.setVel(40,100)

    def moverAtras(self):
        self.setVel(-50, -70)

    def parar(self):
        self.setVel(0, 0)

    def doblarDerecha(self):
        self.setVel(60, -20)

    def doblarIzquierda(self):
        self.setVel(-20, 60)

    def girar(self, vel):
        self.setVel(vel, -vel)

    def girarI(self, vel):
        self.setVel(vel-60, vel+46.5)
    def girarD(self, vel):
        self.setVel(vel+46.5, vel-60)

    def recto(self, vel):
        self.setVel(vel, vel)


# Threshold for detecting the wall 
sensor_value = 0.05
robot = RobotRescate()

posInicial=None
tiempoInicialUltimaSalida=None

def girar90I():
    #print("girar90I")
    robot.girar(-33)
    robot.esperar(0.5)
    robot.pararEnVictimasTemp()
    robot.girar(-33)
    robot.esperar(0.5)
    robot.recto(100)
    robot.esperar(0.07)

def girar90D():
    #print("girar90D")
    robot.girar(33)
    robot.esperar(0.5)
    robot.pararEnVictimasTemp()
    robot.girar(33)
    robot.esperar(0.5)
    robot.recto(100)
    robot.esperar(0.07)

def navegar():
    sd0=robot.getDistD(0)
    sd1=robot.getDistD(1)
    sf0=robot.getDistF(0)
    sf1=robot.getDistF(1)

    robot.recto(75)

    #Si hay pared muy cerca, se aleja
    if(sd1<0.02):
        robot.girar(-15)
        #print(sd1)
        #print("I")
    
    #Si no hay pared, gira a la derecha
    if(sd1>0.082):
        robot.girarD(50)

    #Pared enfrente, gira
    if(sf0<0.035 or sf1<0.035):
        girar90I()

    #idem, pero pozo y avanza en acro
    if(robot.estoyViendoPozo()):
        girar90I()
        robot.recto(100)
        robot.esperar(0.01)
        robot.setVel(80, -20)
        robot.esperar(0.25)

def cerca(pos1, pos2, limite):
    return (distancia(pos1, pos2)<limite)

def distancia(pos1, pos2):
    x1=pos1[0]
    z1=pos1[2]
    x2=pos2[0]
    z2=pos2[2]
    return math.sqrt((x2-x1)*(x2-x1)+(z2-z1)*(z2-z1))

def llegueAlInicio():
    global posInicial, tiempoInicialUltimaSalida
    posActual=robot.getGps()
    if(robot.tiempoActual()-tiempoInicialUltimaSalida<20):
        #print("no paso tiempo")
        return
    if(cerca(posActual, posInicial, 0.003)):
        #print("volvi por relocate o lop")
        tiempoInicialUltimaSalida=robot.tiempoActual()
        return
    if(cerca(posActual, posInicial, 0.04)):
        #print("volvi al inicio")
        robot.enviarMensaje(0,0,'E')
        return
    #print("no pasa nada")

while robot.step():
    #reconocer cuando volvimos al principio
    if(posInicial==None):
        posInicial=robot.getGps()
        tiempoInicialUltimaSalida=robot.tiempoActual()
        robot.moverAtras()
        robot.esperar(0.01)
    
    llegueAlInicio()  
    navegar()
    robot.pararEnVictimaVisible()
    robot.pararEnVictimasTemp()

