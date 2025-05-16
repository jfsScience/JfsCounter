from PyQt6 import QtCore, QtWidgets, QtGui
from PyQt6.QtWidgets import QMessageBox
from PyQt6.QtWebEngineWidgets import QWebEngineView
import sys, time, os
import serial
from serial.tools.list_ports import comports
import pyqtgraph as pg
import traceback
import gui

class MainWindow(QtWidgets.QMainWindow, gui.Ui_MainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()
        self.ui = gui.Ui_MainWindow()
        self.ui.setupUi(self)
        self.setWindowIcon(QtGui.QIcon('jfspc.ico'))
        self.setWindowTitle('PulseCounterESP')
        
        ## Plotwindow
        self.x = []
        self.y = []
        self.grafwd = pg.PlotWidget()
        self.ui.gridLayout_3.addWidget(self.grafwd, 0, 1, 1, 1)
        self.data_line = self.grafwd.plot(self.x, self.y, pen=(0, 255, 0))
        
        ## Threads
        self.timebase = 100
        self.thdx = {}
        self.threadpool = QtCore.QThreadPool()
        
        ## serial
        self.populate_ports()
        self.esp32_serial = None
        
        ## gui
        self.ui.pushButton.pressed.connect(self.connect2board)
        self.ui.pushButton_2.pressed.connect(self.disconnect2board)
        self.ui.pushButton_3.pressed.connect(self.startesp32)
        self.ui.pushButton_4.pressed.connect(self.stopesp32)
        self.ui.pushButton_5.pressed.connect(self.clearPlot)
        self.ui.pushButton_6.pressed.connect(self.setFrequency)
        
        times = ['1', '5', '10', '50', '100', '500', '1000', '5000']
        self.ui.comboBox_2.addItems(times)
        self.ui.comboBox_2.setCurrentText(str(self.timebase))
        self.ui.comboBox_2.currentTextChanged.connect(self.setTimebase)
        self.ui.checkBox.stateChanged.connect(self.testLauf)

        self.closeButtons()
        
        ### Link zur Doku
        file_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "info.html"))
        file_path = file_path.replace("\\", '/')
        file_path = 'file:///' + file_path
        self.ui.webEngineView.setUrl(QtCore.QUrl(file_path))

    def closeEvent(self, event):
        reply = QMessageBox.question(
            self, 
            'Window Close', 
            'Are you sure you want to close the window?',
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No, 
            QMessageBox.StandardButton.No
        )
        if reply == QMessageBox.StandardButton.Yes:
            tnr = self.threadpool.activeThreadCount()
            if tnr == 2:
                self.thdx[1].stop()
                self.thdx[0].stop()
            if tnr == 1:
                self.thdx[0].stop()
            if self.esp32_serial is not None and self.esp32_serial.is_open:
                self.esp32_serial.close()
            event.accept()
            print('Window closed')
        else:
            event.ignore()
                        
    # Populate the available ports
    def populate_ports(self):
        result = []
        for port in comports():
            result.append(port.name)
        self.ui.comboBox.addItems(result)
        self.globports = result
        print("Ports have been populated.",self.globports)
        
    def closeIt(self):
        print("closed")

    def setTimebase(self):
        self.timebase=int(self.ui.comboBox_2.currentText())

    def closeButtons(self):
        self.ui.pushButton.setEnabled(True)
        self.ui.pushButton_2.setEnabled(False)
        self.ui.pushButton_3.setEnabled(False)
        self.ui.pushButton_4.setEnabled(False)
        self.ui.pushButton_5.setEnabled(False)
        self.ui.pushButton_6.setEnabled(False)
        self.ui.checkBox.setEnabled(False)
    
    def openButtons(self):
        self.ui.pushButton.setEnabled(False)
        self.ui.pushButton_2.setEnabled(True)
        self.ui.pushButton_3.setEnabled(True)
        #self.ui.pushButton_4.setEnabled(True)
        self.ui.pushButton_5.setEnabled(True)
        self.ui.pushButton_6.setEnabled(True)
        #self.ui.checkBox.setEnabled(True)

    def connect2board(self):
        self.esp32_port = self.ui.comboBox.currentText()
        self.statusBar().showMessage("You clicked CONNECT TO Esp32")
        try:
            port_declared = self.esp32_port in vars()
            try:
                print("connect to .. ",self.esp32_port)
                self.esp32_serial = serial.Serial()
                self.esp32_serial.port = self.esp32_port
                self.esp32_serial.baudrate = 115200
                self.esp32_serial.parity = serial.PARITY_NONE
                self.esp32_serial.stopbits = serial.STOPBITS_ONE
                self.esp32_serial.bytesize = serial.EIGHTBITS
                self.esp32_serial.timeout = 1
                self.esp32_serial.open()
                self.openButtons()
                time.sleep(0.1)
                self.statusBar().showMessage("Successfully connected to esp32 board.")               
            except:
                self.statusBar().showMessage("Cannot connect to board. Try again..")
        except AttributeError:
            self.statusBar().showMessage("Please plug in the board and select a proper port, then press connect.")

    def disconnect2board(self):
        self.statusBar().showMessage("You clicked DISCONNECT FROM ESP32")
        print("Disconnecting from board..")
        time.sleep(0.1)
        self.esp32_serial.close()
        self.closeButtons()
        print("Board has been disconnected")

    def startesp32(self):
        self.clearPlot()
        self.thdx[0]=  ThC(index=1,port = self.esp32_serial,base=self.timebase)
        self.threadpool.start(self.thdx[0])
        self.thdx[0].signals.datasignal.connect(self.updateData)
        self.ui.pushButton_3.setEnabled(False)
        self.ui.pushButton_4.setEnabled(True)
        self.ui.comboBox_2.setEnabled(False)
        self.ui.checkBox.setEnabled(True)
       
    def stopesp32(self):
        self.thdx[0].stop()
        if self.threadpool.activeThreadCount() == 2 :
            self.thdx[0].stop()
            self.thdx[1].stop()
        if self.threadpool.activeThreadCount() == 1 :
            self.thdx[0].stop()
        self.ui.checkBox.setChecked(False)
        self.ui.pushButton_3.setEnabled(True)
        self.ui.pushButton_4.setEnabled(False)
        self.ui.comboBox_2.setEnabled(True)
        self.ui.checkBox.setEnabled(False)
          
    def updateData(self,x,y):
        self.x.append(x)
        self.y.append(y)
        self.data_line.setData(self.x,self.y)
        
    def clearPlot(self):
        self.x.clear()
        self.y.clear()
        self.data_line.setData(self.x,self.y)
 
    def setFrequency(self):
        s = self.ui.lineEdit.text()
        cmd='o'+str(s)+'\n'
        self.statusBar().showMessage(cmd)
        thread = Thread(self.sendToEsp32, cmd)
        thread.signals.finished.connect(lambda:self.thread_finished(thread))
        self.threadpool.start(thread)

    def sendToEsp32(self, sendStr):
        print(f" sending {sendStr}")
        self.esp32_serial.write(sendStr.encode('utf-8'))
        self.esp32_serial.flushInput()

    def testLauf(self,state):
        if state == QtCore.Qt.Checked:
            self.thdx[1] = ThC_2(port = self.esp32_serial)
            self.threadpool.start(self.thdx[1])
        else:
            self.thdx[1].stop()

    def thread_finished(self, th):
        print("Your thread has completed. Now terminating..")
        th.stop()
        print("Thread has been terminated.")
        print("=============================\n\n")

 
###############################
# MULTITHREADING : SIGNALS CLASS
# ##############################
class WorkerSignals(QtCore.QObject):
    finished = QtCore.pyqtSignal()
    error = QtCore.pyqtSignal(tuple)
    result = QtCore.pyqtSignal(object)
    progress = QtCore.pyqtSignal(int)
    datasignal = QtCore.pyqtSignal(int, float)
class Thread(QtCore.QRunnable):
    def __init__(self, fn, *args, **kwargs):
        super().__init__()
        self.runs = True
        self.fn = fn
        self.args = args
        self.kwargs = kwargs
        self.signals = WorkerSignals()
    def run(self):
        try:
            result = self.fn(*self.args, **self.kwargs)
        except:
            traceback.print_exc()
            exctype, value = sys.exc_info()[:2]
            self.signals.error.emit((exctype, value, traceback.format_exc()))
        else:
            self.signals.result.emit(result)  # Return the result of the processing
        finally:
            self.signals.finished.emit()
            self.stop()
    def stop(self):
        self.runs = False

class ThC(QtCore.QRunnable):
    #datasignal = QtCore.pyqtSignal(int,float)
    def __init__(self,port=None,index=0,base=100):
        super().__init__()
        self.index = index
        self.is_running = True
        self.port = port
        self.base = base
        self.signals = WorkerSignals()
    def run(self):
        print(f"Thread {self.index} started")
        self.port.flushOutput()
        self.port.flushInput()
        i = self.base*1000
        cmd = 's'+str(i)+'\n'
        self.port.write(cmd.encode('utf-8'))
        cnt = 0;
        while True:
            cnt+=1
            inp = self.port.readline()
            try:
                if inp[0] == 62 and inp[-3] == 60 :
                    inp = inp[1:-3]    
                    try:     
                        y = float(inp.decode('utf-8'))
                        self.signals.datasignal.emit(cnt,y)
                    except ValueError:
                        pass
            except IndexError:
                pass
            time.sleep(0.001) 
            if self.is_running==False:
                break
    def stop(self):
        self.is_running=False
        print(f"Thread {self.index} stopped")
        
class ThC_2(QtCore.QRunnable):
    def __init__(self,port=None):
        super().__init__()
        self.is_running = True
        self.port=port
    def run(self):
        cnt=0
        while True:
            cnt+= 1
            if cnt >= 100 : cnt = 1
            cmd = 'o'+str(cnt*10000)+'\n'
            print(cmd)
            self.port.write(cmd.encode('utf-8'))
            time.sleep(1)
            if self.is_running==False:
                break
    def stop(self):
        self.is_running=False
        print(f'Thread ThC-2 stopped')

if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())