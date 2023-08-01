from PyQt5.QtWidgets import QMainWindow, QApplication
from PyQt5.uic import loadUi
from PyQt5.QtSerialPort import QSerialPort, QSerialPortInfo
from PyQt5.QtCore import QIODevice, QPoint
from PyQt5 import QtCore, QtWidgets
import pyqtgraph as pg
from pyqtgraph import PlotWidget
import numpy as np
import sys
import gc
import csv

# HMI class for the GUI
class HMI(QMainWindow):
    # Constructor for the HMI class
    def __init__(self):
        super(HMI, self).__init__() # Call the inherited classes __init__ method
        loadUi('HMI.ui', self) # Load the .ui file
        self.click_position = QPoint()  # Create a QPoint object to store the click position
        self.setWindowTitle('Automatic Mast System HMI') # Set the window title
        self.setMinimumSize(1366, 768) # Set the minimum window size

        # Create the serial object and connect the signals to their respective functions
        self.serial = QSerialPort() # Create a QSerialPort object
        self.bt_conectar_serial.clicked.connect(self.conectar_puerto) # Connect the conectar_serial button to the conectar_puerto function
        self.bt_actualizar_serial.clicked.connect(self.leer_puertos) # Connect the actualizar_serial button to the actualizar_puertos function
        self.bt_desconectar_serial.clicked.connect(self.desconectar_puerto) # Connect the desconectar_serial button to the desconectar_puerto function
        self.serial.readyRead.connect(self.leer_serial) # Connect the serial.readyRead signal to the leer_serial function

        # Connect the buttons from the Lidar Frame to their respective functions
        self.bt_iniciar_lidar.clicked.connect(lambda: self.enviar_serial("Enviando instrucción: Iniciar Lidar", 1)) # Connect the iniciar_lidar button to the iniciar_lidar function
        self.bt_detener_lidar.clicked.connect(self.detener_lidar) # Connect the pausar_lidar button to the pausar_lidar function
        self.bt_guardar_lidar.clicked.connect(self.guardar_lidar) # Connect the guardar_lidar button to the guardar_lidar function
        self.lectura_lidar = [] # Create a list to store the Lidar data

        # Connect the Mode Selection buttons to their respective functions
        self.modo_manual.clicked.connect(lambda: self.seleccionar_modo('manual')) # Connect the modo_manual button to the modo_manual_func function
        self.modo_auto.clicked.connect(lambda: self.seleccionar_modo('auto')) # Connect the modo_automatico button to the modo_automatico_func function
        
        # Connect the buttons from the Modo Manual Frame to their respective functions
        self.bt_enviar_manual.clicked.connect(self.enviar_manual_data) # Connect the enviar_manual button to the enviar_manual_func function
        self.bt_home_tilt.clicked.connect(self.home) # Connect the home_tilt button to the home_tilt function
        self.bt_ascenso.clicked.connect(lambda: self.enviar_serial("Enviando instrucción: Ascenso:",7)) # Connect the ascenso button to the ascenso function
        self.bt_descenso.clicked.connect(lambda: self.enviar_serial("Enviando instrucción: Descenso:",8)) # Connect the descenso button to the descenso function

        # Connect the buttons from the Modo Automatico Frame to their respective functions
        self.bt_iniciar_rutina.clicked.connect(self.iniciar_rutina) # Connect the iniciar_rutina button to the iniciar_rutina function

        # Connect the signals from the Modo Automatico Range Selection to the PlotWidget
        self.entry_inf_pan.valueChanged.connect(self.actualizar_grafica) # Connect the entry_inf_pan signal to the update_plot function
        self.entry_sup_pan.valueChanged.connect(self.actualizar_grafica) # Connect the entry_sup_pan signal to the update_plot function

        self.leer_puertos() # Call the leer_puertos function to obtain the available ports
        self.inicializar_grafica() # Call the update_plot function to initialize the plot
        self.seleccionar_modo('manual') # Call the seleccionar_modo function to initialize the mode to manual

    def leer_puertos(self):
        self.baudrates = ['4800','9600', '19200', '38400', '57600', '115200','230400','460800'] # List of baudrates
        puertos_disponibles = []
        puertos = QSerialPortInfo.availablePorts() # Get the available ports
        for puerto in puertos:
            puertos_disponibles.append(puerto.portName())
            
        self.lista_puertos.clear() # Clear the list of ports
        self.lista_baud.clear() # Clear the list of baudrates
        self.lista_puertos.addItems(puertos_disponibles) # Add the available ports to the list
        self.lista_baud.addItems(self.baudrates) # Add the baudrates to the list
        self.lista_baud.setCurrentIndex(7) # Set the default baudrate to 460800
        
    def conectar_puerto(self): # Function to connect to the serial port
        try:
            self.serial.waitForReadyRead(100) # Wait for 100 ms
            self.port = self.lista_puertos.currentText() # Get the selected port
            self.baud = self.lista_baud.currentText() # Get the selected baudrate
            self.serial.setPortName(self.port) # Set the port name
            self.serial.setBaudRate(int(self.baud)) # Set the baudrate
            self.serial.open(QIODevice.ReadWrite) # Open the port
            print('Conectado al puerto: ', self.port)
        except:
            print('Error al abrir el puerto serial')
    
    def desconectar_puerto(self): # Function to disconnect from the serial port
        try:
            self.serial.close() # Close the port
            print('Desconectado del puerto: ', self.port)
        except:
            print('Error al cerrar el puerto serial')

    def leer_serial(self): # Function to read the serial port
        try:
            linea = self.serial.readLine()
            linea_decoded = str(linea, 'utf-8').strip()
            linea_decoded.strip('\n')
            linea_decoded.replace(',','')
            lista_datos = [valor for valor in linea_decoded.split(',') if valor.strip()]
            self.lectura_lidar.append(lista_datos)
        except:
            print('Error al leer el puerto serial')

    def enviar_serial(self, mensaje, data): # Function to send data to the serial port
        try:
            data = bytes([data])
            print(mensaje,data)
            if self.serial.isOpen():
                self.serial.write(data)
        except:
            print('Error al enviar datos por el puerto serial')

    def detener_lidar(self): # Function to stop the Lidar
        self.enviar_serial("Enviando instrucción: Detener Lidar", 2)
        self.display_lidar.setText(str(len(self.lectura_lidar)))
        

    def guardar_lidar(self): # Function to save the Lidar data
        ruta_archivo = "datosLidar.csv"
        # Escribir la lista de datos en el archivo CSV
        with open(ruta_archivo, "w", newline="") as archivo_csv:
            writer = csv.writer(archivo_csv)
            writer.writerows(self.lectura_lidar)
        self.lectura_lidar = []
        self.display_lidar.setText(str(len(self.lectura_lidar)))
    
    def seleccionar_modo(self, seleccion): # Function to select the mode of the system, it will habilitate or disable the respective frames buttons
        if(seleccion == 'manual'):
            self.modo = 'M'
        elif(seleccion == 'auto'):
            self.modo = 'A'
    
    def actualizar_display(self):
        self.disp_tilt.display(self.tilt)
        self.disp_pan.display(self.pan)

    def home(self): # Function to send the home signal to the serial port
        self.enviar_serial("Enviando instrucción: Posición Home Cabezal",6)
        self.tilt = 0
        self.pan = 0
        self.actualizar_display()
        
    def enviar_manual_data(self): # Function to send the manual data to the serial port
        if(self.modo == 'M'):
            self.enviar_serial("Enviando instrucción: Enviar Pos. Manual", 5)
            self.tilt = self.entry_tilt_manual.value()
            self.pan = self.entry_pan_manual.value()
            self.actualizar_display()
            data = str(self.tilt)+','+ str(self.pan)
            data = data.encode('utf-8')
            print(data)
            if self.serial.isOpen():
                self.serial.write(data)
        else:
            print('Te encuentras en modo automatico, cambia a modo manual para enviar datos')

    def iniciar_rutina(self): # Function to send the initiate signal for the scanning routine to the serial port
        if(self.modo == 'A'):
            self.enviar_serial("Enviando instrucción: Iniciar Rutina", 4)
            tilt = self.entry_tilt_auto.value()
            inf_pan = self.entry_inf_pan.value()
            sup_pan = self.entry_sup_pan.value()
            data = str(tilt)+','+str(inf_pan)+','+str(sup_pan)
            data = data.encode('utf-8')
            print(data)
            if self.serial.isOpen():
                self.serial.write(data)  
        else:
            print('Te encuentras en modo manual, cambia a modo automatico para iniciar la rutina')

    def inicializar_grafica(self):
        self.range_plot.setAspectLocked() # Lock the aspect ratio of the plot
        self.range_plot.hideAxis('bottom') # Hide the bottom axis
        self.range_plot.hideAxis('left') # Hide the left axis
        self.range_plot.setRange(xRange=[-150,150],yRange=[10,250]) # Set the range of the plot
        self.range_plot.addLine(x=0, pen=0.2) # Add the x axis
        self.range_plot.addLine(y=0, pen=0.2) # Add the y axis
        
        for p in range(15,175,15): # Add the polar axis
            polar_axis = pg.InfiniteLine((0,0),angle=p,pen=0.2)
            self.range_plot.addItem(polar_axis)

        for r in range(1,600,100): # Create the polar grid
            circle = pg.CircleROI((-r/2,-r/2),r, pen = pg.mkPen(0.2))
            self.range_plot.addItem(circle)
            circle.removeHandle(0)

        # Create the polar label points every 15 degrees or pi/12 radians
        theta = np.linspace(0, np.pi, 13)
        radius = 250

        # Transform to cartesian and plot the labels
        x = radius * np.cos(theta)
        y = radius * np.sin(theta)

        grid_labels = ['90°','75°','60°','45°','30°','15°','0°','-15°','-30°','-45°','-60°','-75','-90']
        for i in range(len(grid_labels)):
            label = pg.TextItem(grid_labels[i], anchor=(0.5,0.5))
            label.setPos(x[i],y[i])
            self.range_plot.addItem(label)

        mast_point = (0,0)
        self.range_plot.plot(mast_point,mast_point, pen=None, symbol='o', symbolSize=25, symbolBrush='y') # Add the mast point
        
    def actualizar_grafica(self):
        self.range_plot.clear() # Clear the plot
        self.inicializar_grafica() # Initialize the plot
        radius = 250 # Set the radius of the plot
        
        # Create the pan cartesian coordinates
        inf_pan = self.entry_inf_pan.value()
        sup_pan = self.entry_sup_pan.value()

        # Modify the pan angles to consider 90 degrees as the 0 degrees, so negative angles will be located in the 2nd quadrant and positive angles in the 1th quadrant
        inf_pan = 90 + abs(inf_pan)
        sup_pan = 90 - sup_pan

        cartesian_inf_pan_x = radius * np.cos((2*np.pi*inf_pan)/360) # Transform the pan angles to cartesian coordinates, the angles are in degrees so we need to convert them to radians
        cartesian_inf_pan_y = radius * np.sin((2*np.pi*inf_pan)/360)
        cartesian_sup_pan_x = radius * np.cos((2*np.pi*sup_pan)/360)
        cartesian_sup_pan_y = radius * np.sin((2*np.pi*sup_pan)/360)

        self.range_plot.plot((cartesian_inf_pan_x,cartesian_sup_pan_x),(cartesian_inf_pan_y,cartesian_sup_pan_y), pen=None, symbol='o', symbolSize=7, symbolBrush='r')

        # Create the field of view of the pan range
        pov_inf = pg.InfiniteLine((0,0),angle=inf_pan,pen=1)
        self.range_plot.addItem(pov_inf)
        pov_sup = pg.InfiniteLine((0,0),angle=sup_pan,pen=1)
        self.range_plot.addItem(pov_sup)
    
    
    

        
    

if __name__ == '__main__': # If we're running file directly and not importing it
    app = QApplication(sys.argv) # Create an instance of QApplication
    window = HMI() # Create an instance of the HMI class
    window.show() # Show the window
    gc.collect() # Collect the garbage
    sys.exit(app.exec_()) # Start the event loop