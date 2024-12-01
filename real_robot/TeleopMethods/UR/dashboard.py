import socket

class rtde_dashboard:
    def __init__(self, ip_address):
        self.ip = ip_address
        self.port = 29999
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.settimeout(1.0)
        self.socket.connect((self.ip, self.port))
    
    def send(self, message):
        self.socket.send(message.encode())
    
    def receive(self):
        return self.socket.recv(1024).decode()
    
    def __del__(self):
        self.socket.close()

    def unlockProtectiveStop(self):
        self.send("unlock protective stop\n")
        return self.receive()
    
    def close_popup(self):
        self.send("close popup\n")
        return self.receive()
    
    def stop(self):
        self.send("stop\n")
        return self.receive()