import socket
import multiprocessing as mp
import ctypes as c
import struct
from time import sleep
from enum import Enum


class IO():

    # UPD command
    WRITE_ALL_OUT = 0x01  # write whole coil register at once
    WRITE_ONE_OUT = 0x02  # write single coil
    HEART_BEAT = 0x03  # watchdog reset

    class Status(Enum):
        OK = 0
        NO_RESPONSE = 1
        INCORRECT_RESPONSE = 2
        CONN_CLOSED = 3

    def __init__(self):
        self.UDP_PARAM = ("192.168.0.251", 2011)  # plc connection settings

        self.l = mp.Lock()  # mutex for UDP send/receive commands

        # socket settings
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.s.setblocking(0)

        # mp values
        self.status = mp.Value(c.c_int, IO.Status.OK.value)
        self.mode = mp.Value(c.c_int, IO.HEART_BEAT)
        self.stop_flag = mp.Value(c.c_bool, False)

        # heartbeat process
        self.p = mp.Process(target=self.watchdog_thread)
        self.p.start()

    def watchdog_thread(self):
        arr = bytearray([IO.HEART_BEAT])
        while(not self.stop_flag.value):
            self.l.acquire()  # acquire lock, we dont want simultaneous commands to be sent from watchdog and main threads
            self.s.sendto(arr, self.UDP_PARAM)  # send heartbeat command
            sleep(0.01)
            try:
                data, addr = self.s.recvfrom(1023)
                # check if we have correct response from PLC
                self.check_status(data, arr)
            except:
                self.status.value = IO.Status.NO_RESPONSE.value  # communication error
            self.l.release()  # release lock
            # PLC resets its outputs after 3 seconds without communication, we need to run it at least that frequency
            sleep(1)

    def check_status(self, res, arr):
        # check if PLC echo'd UDP command properly
        answer = struct.unpack('!b', res[0])
        if(answer[0] != arr[0]):
            self.status.value = IO.Status.INCORRECT_RESPONSE.value
        else:
            self.status.value = IO.Status.OK.value

    def close(self):
        arr = bytearray([IO.WRITE_ALL_OUT, 0])
        self.send_cmd(arr)
        self.stop_flag.value = True
        self.s.close()
        self.status = IO.Status.CONN_CLOSED
        self.p.join()

    def send_cmd(self, arr):
        self.l.acquire()
        self.s.sendto(arr, self.UDP_PARAM)
        sleep(0.01)
        try:
            data, addr = self.s.recvfrom(1023)
            self.check_status(data, arr)
        except:
            self.status.value = IO.Status.NO_RESPONSE.value
        self.l.release()

    def grab(self):
        arr = bytearray([IO.WRITE_ONE_OUT, 1, 1])
        self.send_cmd(arr)

    def release(self):
        arr = bytearray([IO.WRITE_ONE_OUT, 1, 0])
        self.send_cmd(arr)

    def cutter_on(self):
        arr = bytearray([IO.WRITE_ONE_OUT, 0, 1])
        self.send_cmd(arr)

    def cutter_off(self):
        arr = bytearray([IO.WRITE_ONE_OUT, 0, 0])
        self.send_cmd(arr)


if __name__ == '__main__':
    io = IO()

    io.release()
    io.cutter_on()
    print("IO status " + io.Status(io.status.value).name)
    sleep(1)
    print("closing connection")
    print("IO status " + io.Status(io.status.value).name)
    io.close()
    print("connection closed")
    print("IO status " + io.Status(io.status.value).name)
    sleep(5)
    print("killing script")
