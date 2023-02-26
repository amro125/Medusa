from xarm_controller import XArmController
from queue import Queue
from pythonosc import dispatcher, osc_server
from threading import Thread, Event
import atexit

server_ip = "0.0.0.0"
server_port = 12346


def handler(address, *args):
    print("Received OSC message:", address, args)
    if address == "/gesture":
        if args[0] == "wave_hello":
            q0.put("wave_hello")
        elif args[0] == "wave_bye":
            q0.put("wave_bye")
        elif args[0] == "twirl":
            q0.put("twirl")
        print(args[0])


def server(ip, port, msg_dispatcher):
    msg_server = osc_server.ThreadingOSCUDPServer((ip, port), msg_dispatcher)
    print("Serving on {}".format(msg_server.server_address))
    msg_server.serve_forever()
    atexit.register(msg_server.server_close())


if __name__ == '__main__':
    ROBOT = "xArms"
    PORT = 5004
    xarms = XArmController()
    input("Press enter to continue setup arms...\n")
    xarms.setup()
    input("Press enter to continue setup position...\n")
    xarms.setup_pos()
    for a in xarms.arms:
        a.set_mode(1)
        a.set_state(0)

    q0 = Queue()
    q1 = Queue()
    q2 = Queue()
    q3 = Queue()
    q4 = Queue()
    dq0 = Queue()
    dq1 = Queue()

    t0 = Thread(target=xarms.strummer, args=(q0, 0,))
    t1 = Thread(target=xarms.strummer, args=(q1, 1,))
    t2 = Thread(target=xarms.strummer, args=(q2, 2,))
    t3 = Thread(target=xarms.strummer, args=(q3, 3,))
    t4 = Thread(target=xarms.strummer, args=(q4, 4,))
    dt0 = Thread(target=xarms.drummer, args=(dq0, 5,))
    dt1 = Thread(target=xarms.drummer, args=(dq1, 6,))

    input("Press enter to start enable arms queue...\n")
    t0.start()
    t1.start()
    t2.start()
    t3.start()
    t4.start()
    dt0.start()
    dt1.start()

    dispatcher = dispatcher.Dispatcher()
    dispatcher.map("/gesture", handler)
    serve = Thread(target=server, args=(server_ip, server_port, dispatcher,))

    input("Press enter to start listening...\n")
    serve.start()
