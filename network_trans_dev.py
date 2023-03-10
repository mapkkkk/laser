from socket import *
from time import ctime

host = ''
port = 21567
bufsiz = 1024
addr = (host, port)

tcpSerSock = socket(AF_INET, SOCK_STREAM)
tcpSerSock.bind(addr)
tcpSerSock.listen(5)

while True:
    print('waiting for connection...')
    tcpCliSock, addr = tcpSerSock.accept()
    print('...connected from:', addr)

    while True:
        data = tcpCliSock.recv(bufsiz)
        if not data:
            break
        tcpCliSock.send('[%s] %s' % (
            bytes(ctime(), 'utf-8'), data))
        
    tcpCliSock.close()
    
tcpSerSock.close()
