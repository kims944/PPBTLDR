import threading
import sys
import serial

import numpy as np
import time


'''from main import dicACK, dicDatalog, dicConfiguration, dicParameter, dicAlarm, dicWarning, dicOperationLimit, serial_number, \
    dicState'''

__platform__ = sys.platform
flag = 0


class SerialComm:
    def __init__(self, parent, packet_buf):
        self.parent = parent
        self.packet_buf = packet_buf
        super().__init__()
        self.packet = packet.Packet(self, dicDatalog, dicConfiguration, dicParameter, dicACK, dicAlarm, dicWarning,
                                    dicOperationLimit, serial_number, dicState)

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
                packets = ser.read(constant_def.UART_BUFFER_SIZE)
                #print('L45 serialcomm', str(packets))
                #print('L46 serialcomm decode busy', self.packet_buf.check_decode_busy())
                #if self.packet_buf.check_decode_busy() == False:
                self.packet_buf.packetin(packets)
                #ser.flush()
                #ser.reset_input_buffer()
                #print('L49 serialcomm', str(packets))
        if not state:
            ser.close()
            #print('L51 stop event')

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

    def transmitpacket(self, ser, packet):
        temppacket = [constant_def.HDLC_FLAG]
        for val in packet:
            temppacket.append(val & 0xFF)
        [cksA, cksB] = self.packet.checksumAB(temppacket[1:])
        #print('L77', [cksA, cksB], temppacket)

        hdlcpacket = [constant_def.HDLC_FLAG]
        for val in packet:
            if val == constant_def.HDLC_FLAG or val == constant_def.ESC_FLAG:
                hdlcpacket.append(constant_def.ESC_FLAG)
                hdlcpacket.append(val ^ constant_def.ESC_CHAR)
            else:
                hdlcpacket.append(val & 0xFF)

        #[cksA, cksB] = self.packet.checksumAB(hdlcpacket[1:])
        #print('L126 checksum', [cksA, cksB], hdlcpacket)

        if cksA == constant_def.HDLC_FLAG or cksA == constant_def.ESC_FLAG:
            hdlcpacket.append(constant_def.ESC_FLAG)
            hdlcpacket.append(cksA ^ constant_def.ESC_CHAR)
        else:
            hdlcpacket.append(cksA)

        if cksB == constant_def.HDLC_FLAG or cksB == constant_def.ESC_FLAG:
            hdlcpacket.append(constant_def.ESC_FLAG)
            hdlcpacket.append(cksB ^ constant_def.ESC_CHAR)
        else:
            hdlcpacket.append(cksB)

        hdlcpacket.append(constant_def.HDLC_FLAG)

        ##############################################
        lenp = round(np.ceil(len(hdlcpacket) / constant_def.UART_TX_PACKET_LENGTH))
        #print('L101', len(hdlcpacket), lenp, hdlcpacket)

        if len(hdlcpacket) < constant_def.UART_TX_PACKET_LENGTH * lenp:
            temp = constant_def.UART_TX_PACKET_LENGTH * lenp - len(hdlcpacket)
            while temp > 0:
                hdlcpacket.append(0)
                temp = temp - 1

        for n in range(lenp):
            ser.write(
                bytes(hdlcpacket[constant_def.UART_TX_PACKET_LENGTH * n:constant_def.UART_TX_PACKET_LENGTH * (n + 1)]))
            #print('L112', n, hdlcpacket[constant_def.UART_TX_PACKET_LENGTH * n:constant_def.UART_TX_PACKET_LENGTH * (n + 1)])
            if int(lenp) > 2:
                time.sleep(0.1)

        ##############################################

    def request_serial_data_event(self, state, ser):
        self.serial_thread_data_req = threading.Timer(0.1, self.request_serial_data_event, args=[state, ser])
        self.serial_thread_data_req.start()

        if ser.is_open:
            self.parent.updateobject()