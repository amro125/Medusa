from queue import Queue
from threading import Thread

q = Queue()
def receiveq():
    # q.get(timeout=2)
    while q.empty() == True:
        y = q.get()
        print(y)


xArm0 = Thread(target=receiveq, args=())
input("press enter to start")
xArm0.start()
while True:
    x = int(input("add to q"))
    q.put(x)