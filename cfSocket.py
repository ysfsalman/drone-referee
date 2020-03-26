import socket  
import threading

class cfSocket():
  # init method or constructor
  def __init__(self, address='localhost', port=30000):
    self.s =  socket.socket()
    self.bytepose = bytes()
    self.dronepose = []
    self.ballpos = []
    while True:
      try:
        self.s.connect((address, port))
        break
      except:
        port += 1
    thread = threading.Thread(target=self.getBallPos, args=())
    thread.start()                             
        

  def sendDronePose(self, pose = []):
    for i in pose:
      self.bytepose += (i).to_bytes(2,'big',signed=True)
    self.s.send(self.bytepose)
    self.bytepose = b''
  
  def ballpos():
    return self.ballpos
  
  def getBallPos(self):
    x = 1
  '''
    while True:
      self.ballpos =  self.s.recv() # change depends on source
  '''
  


