from PyQt5.QtWidgets import QMainWindow, QApplication
from PyQt5.uic import loadUi
from PyQt5.QtSerialPort import QSerialPort, QSerialPortInfo
from PyQt5.QtCore import QIODevice, QPoint
from PyQt5 import QtCore, QtWidgets
from pyqtgraph import PlotWidget
import numpy as np
import sys
import time
import threading

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
        self.bt_desconectar_serial.clicked.connect(lambda: self.serial.close()) # Connect the desconectar_serial button to the desconectar_puerto function
        self.serial.readyRead.connect(self.leer_serial) # Connect the serial.readyRead signal to the leer_serial function

        # Connect the buttons from the Lidar Frame to their respective functions
        self.bt_iniciar_lidar.clicked.connect(self.iniciar_lidar) # Connect the iniciar_lidar button to the iniciar_lidar function
        self.bt_pausar_lidar.clicked.connect(self.pausar_lidar) # Connect the pausar_lidar button to the pausar_lidar function
        self.bt_guardar_lidar.clicked.connect(self.guardar_lidar) # Connect the guardar_lidar button to the guardar_lidar function

        # Connect the Mode Selection buttons to their respective functions
        self.modo_manual.clicked.connect(self.modo_select('manual')) # Connect the modo_manual button to the modo_manual_func function
        self.modo_automatico.clicked.connect(self.modo_select('auto')) # Connect the modo_automatico button to the modo_automatico_func function
        
        # Connect the buttons from the Modo Manual Frame to their respective functions
        self.bt_enviar_manual.clicked.connect(self.enviar_manual_data) # Connect the enviar_manual button to the enviar_manual_func function
        self.bt_home_tilt.clicked.connect(self.home_tilt) # Connect the home_tilt button to the home_tilt function
        self.bt_ascenso.clicked.connect(self.ascenso) # Connect the ascenso button to the ascenso function
        self.bt_descenso.clicked.connect(self.descenso) # Connect the descenso button to the descenso function

        # Connect the buttons from the Modo Automatico Frame to their respective functions
        self.bt_iniciar_rutina.clicked.connect(self.iniciar_rutina) # Connect the iniciar_rutina button to the iniciar_rutina function

        # Connect the signals from the Modo Automatico Range Selection to the PlotWidget
        self.entry_inf_tilt.valueModified.connect(self.update_plot) # Connect the entry_inf_tilt signal to the update_plot function
        self.entry_sup_tilt.valueModified.connect(self.update_plot) # Connect the entry_sup_tilt signal to the update_plot function
        self.entry_inf_pan.valueModified.connect(self.update_plot) # Connect the entry_inf_pan signal to the update_plot function
        self.entry_sup_pan.valueModified.connect(self.update_plot) # Connect the entry_sup_pan signal to the update_plot function

        self.leer_puertos() # Call the leer_puertos function to obtain the available ports

        def leer_puertos(self):
            self.baudrates = ['4800','9600', '19200', '38400', '57600', '115200','256000'] # List of baudrates
            puertos_disponibles = []
            puertos = QSerialPortInfo.availablePorts() # Get the available ports
            for puerto in puertos:
                puertos_disponibles.append(puerto.portName())
            
            self.lista_puertos.clear() # Clear the list of ports
            self.lista_baud.clear() # Clear the list of baudrates
            self.lista_puertos.addItems(puertos_disponibles) # Add the available ports to the list
            self.lista_baud.addItems(self.baudrates) # Add the baudrates to the list
            self.lista_baud.setCurrentIndex(6) # Set the default baudrate to 256000
        
        def conectar_puerto(self):
            self.serial.waitForReadyRead(100) # Wait for 100 ms
            self.port = self.lista_puertos.currentText() # Get the selected port
            self.baud = self.lista_baud.currentText() # Get the selected baudrate
            self.serial.setPortName(self.port) # Set the port name
            self.serial.setBaudRate(int(self.baud)) # Set the baudrate
            self.serial.open(QIODevice.ReadWrite) # Open the port

        def leer_serial(self):
            if not self.serial.canReadLine(): return
            try:
                linea = self.serial.readLine()
                linea_decoded = str(linea, 'utf-8').strip()
                print(linea_decoded)
            except:
                print('Error al leer el puerto serial')

        def enviar_serial(self, data):
            try:
                data = data + '\n'
                print("Enviando:",data)
                if self.serial.isOpen():
                    self.serial.write(data.encode())
            except:
                print('Error al enviar datos por el puerto serial')

        def iniciar_lidar(self):
            print('Iniciando Lidar')
            self.enviar_serial()



    



if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = HMI()
    window.show()
    sys.exit(app.exec_())