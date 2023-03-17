from socket import *

host = '127.0.0.1' # or 'localhost'
port = 21567
bufsiz = 1024
addr = (host, port)

tcpCliSock = socket(AF_INET, SOCK_SEQPACKET)
tcpCliSock.connect(addr)

while True:
    data = input('> ')
    if not data:
        break
    print(data.decode('utf-8'))

tcpCliSock.close()
