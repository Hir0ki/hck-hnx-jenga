from tinkerforge.ip_connection import IPConnection
from tinkerforge.bricklet_load_cell_v2 import BrickletLoadCellV2


class PressureSensor():

    def __init__(self, uid, host):
        
        self._host = host
        self._port = 4223
        self._uid = uid # Change XYZ to the UID of your Load Cell Bricklet 2.0

        self._connect()

    def _connect(self):
        print(self._host)
        print(self._port)
        print(self._uid)
        
        self._ipcon = IPConnection() # Create IP connection
        self._lc = BrickletLoadCellV2(self._uid, self._ipcon) # Create device object

        self._ipcon.connect(self._host, self._port) # Connect to brickd

    def disconnect(self):

        self._ipcon.disconnect()

    def get(self):
        return self._lc.get_weight()
