#!/usr/bin/env python2
import collections
import socket
import threading
import sys

if __name__ == "__main__":
    data = collections.deque(maxlen=int(sys.argv[1]))
    lock = threading.Lock()

    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    sock.bind(sys.argv[2])
    sock.listen(1)

    def serve():
        while True:
            connection, _ = sock.accept()
            try:
                with lock:
                    for l in  reversed(data):
                        connection.sendall(l)
            except:
                pass
            finally:
                connection.close()

    server = threading.Thread(target=serve)
    server.setDaemon(True)
    server.start()

    while True:
        l = sys.stdin.readline()
        with lock:
            data.append(l)
 