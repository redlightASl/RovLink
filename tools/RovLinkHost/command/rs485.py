import serial


class Rs485(object):
    def __init__(self, read_port="COM31", send_port="COM9", baudrate="115200", debug=False) -> None:
        self._read_port = read_port
        self._send_port = send_port
        self._baudrate = baudrate
        self._debug = debug
        self._bytesize = 8
        self._parity = "N"
        self._stopbits = 1
        self._timeout = 0.1

        # self._rx_comm = serial.Serial(port=self._read_port,
        #                          baudrate=self._baudrate,
        #                          bytesize=self._bytesize,
        #                          parity=self._parity,
        #                          stopbits=self._stopbits,
        #                          timeout=self._timeout)

        self._tx_comm = serial.Serial(port=self._send_port,
                                      baudrate=self._baudrate,
                                      bytesize=self._bytesize,
                                      parity=self._parity,
                                      stopbits=self._stopbits,
                                      timeout=self._timeout)

    def transmit(self, msg, dbg_print=False) -> None:
        if dbg_print:
            print(msg)
        self._tx_comm.write(msg)

    # def receive(self):
    #     data = self._rx_comm.read_all()
    #     return data

    def release(self) -> None:
        # self._rx_comm.close()
        self._tx_comm.close()


if __name__ == '__main__':
    comm = Rs485(debug=True)
    comm.transmit(b"Hello World!", dbg_print=True)
    comm.release()
