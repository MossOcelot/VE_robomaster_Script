import socket
from robomaster import logger

class SimConnection:
    def __init__(self ):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    def request_connection(self, host, port, proto_type='udp'):
        if host is None or port is None:
            logger.error("SimConnection: request_connection, host or port is None.")
            return False, None
        
        self._sock.settimeout(3)
        self._sock.sendto(b'hello', (host, port))
        self._sock.settimeout(3)

        data, address = self._sock.recvfrom(1024)
        print(data, address)
        logger.debug("SimConnection: request_connection, data:{0}, address:{1}".format(data, address))