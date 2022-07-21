import sys, os
from PyQt5.QtWidgets import *
from PyQt5 import uic
from PyQt5.QtCore import *
from PyQt5.QtGui import QPixmap
from PyQt5.QtGui import QIcon

from datetime import datetime
import threading
import time
import serial
NO_CRC_CHECK = False

def resource_path(relative_path):
    base_path = getattr(sys, '_MEIPASS', os.path.dirname(os.path.abspath(__file__)))
    print('L14 base_path', base_path, 'rel_path', os.path.join(base_path, relative_path))
    return os.path.join(base_path, relative_path)

form = resource_path("PPbootloader_v1.ui")
form_class = uic.loadUiType(form)[0]

__platform__ = sys.platform


class MainWindow(QMainWindow, form_class):
    def __init__(self):
        super().__init__()
        self.setupUi(self)

        self.PBConnect.clicked.connect(self.slot_clicked_connect_button)

        BAUDRATES = (9600, 115200)
        self.serial_comm = SerialComm(self)
        self.CBPort.insertItems(0, self.serial_comm.get_available_port())
        self.CBBaud.insertItems(0, [str(x) for x in BAUDRATES])

    def slot_clicked_connect_button(self):
        global bconnect_state,  ser
        port = self.CBoxPort.currentText()
        baudrate = self.CBoxBaud.currentText()
        print('main L421', [port, baudrate])
        try:
            ser = self.serial_comm.open_serial_port(port, baudrate)
            ser.reset_input_buffer()
            print(['main L411', ser])
            self.uart_synch(1)
            self.init_button_status()
        except:
            print(['main L415 serial port is not available and not opened'])
            pass
        if bconnect_state == 0:
            bconnect_state = 1
        else:
            bconnect_state = 0
            if ser.is_open:
                print(['L406 serial port closed', ser])
                self.uart_synch(0)
                time.sleep(0.5)
                ser.close()
            else:
                print('385 port is not closed')

        self.serial_comm.serial_event_thread(bconnect_state, ser)
        self.serial_comm.request_serial_data_event(bconnect_state, ser)# call self.updateobject()
        self.pushButton_Connect.setText({0: 'Connect', 1: 'Disconnect'}[bconnect_state])
        _refresh_count = 0
        self.refreshscreen(_refresh_count, ser)


class SerialComm:
    def __init__(self, parent):
        self.parent = parent

        super().__init__()

    def get_available_port(self):
        available_port = list()
        port_path = {"linux": '/dev/ttyS', "win32": 'COM'}[__platform__]

        for number in range(255):
            port_name = port_path + str(number)
            try:
                ser = serial.Serial(str(port_name))
                available_port.append(port_name)
                print('serialcomm L32 available port:', port_name)
            except:
                pass
                #print('serialcomm L34 serial port not available', number)

        return available_port

    def open_serial_port(self, port, baud):
        self.port = port
        self.baud = baud

        ser = serial.Serial(self.port, self.baud, inter_byte_timeout=0.0)
        print('serialcomm L44')
        return ser

    def check_serial_event(self, state, ser):
        print('serialcomm L40 serial thread event')
        while ser.is_open:
            if ser.in_waiting:
                #read constant_def.UART_BUFFER_SIZE bytes, returns bytes
                packets = ser.read(16)
        if not state:
            ser.close()

    def serial_event_thread(self, state, ser):
        print('serialcomm L58 thread start')
        serial_thread = threading.Thread(target=self.check_serial_event, args=[state, ser])
        serial_thread.start()

    def tohex(val, nbits):
        return hex((val + (1 << nbits)) % (1 << nbits))

    def convertvalue(self, addr, param):
        value = int(param, 10)

        x = hex((int(param) + (1 << 32)) % (1 << 32))
        y = int(x, 16)
        values=[addr, (y & 0xFF000000) >> 24, (y & 0x00FF0000) >> 16, (y & 0x0000FF00) >> 8, (y & 0x000000FF)]
        print("addr param values", addr, param, values)
        return values

    def request_serial_data_event(self, state, ser):
        self.serial_thread_data_req = threading.Timer(0.1, self.request_serial_data_event, args=[state, ser])
        self.serial_thread_data_req.start()

        if ser.is_open:
            self.parent.updateobject()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    myWindow = MainWindow()
    myWindow.show()
    sys.exit(app.exec_())

