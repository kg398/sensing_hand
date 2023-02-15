import socket


class matlab_comms():
    def __init__(self,port=10000):
        # Create a TCP/IP socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # Bind the socket to the port
        server_address = ('127.0.0.1', 10000)
        print('starting up on %s port %s' % server_address)
        sock.bind(server_address)

        # Listen for incoming connections
        sock.listen(1)

        # Wait for a connection
        print('waiting for a connection')
        self.connection, client_address = sock.accept()

        print('connection from', client_address)
        return

    def read(self):
        # Receive the data in small chunks
        data = bytes.decode(self.connection.recv(16))
        #print('received "%s"' % data)
        return data

    def read_raw(self):
        # Receive the data in small chunks
        data = self.connection.recv(16)
        #print('received "%s"' % data)
        return data

    def write(self,data):
        # Receive the data in small chunks and retransmit it
        #print('sending data back to the client')
        self.connection.sendall(str.encode(data+'\n'))
        return
            
    def close(self):
        self.connection.close()
        return