### sends a continuous pulse
from SimpleXMLRPCServer import SimpleXMLRPCServer
import xmlrpclib
import sys

server = xmlrpclib.ServerProxy("http://localhost:5030")

def safe_exit(sc,e):
    print "Exit safely"
    print e
    sc.stop()
    sys.exit()

if __name__=="__main__":
    pulse_width = int(sys.argv[1])   # 1-16383
    pulse_delay = float(sys.argv[2]) # time between subsequent pulses (1/rate) [ms]
    pulse_number = int(sys.argv[3])  # how many pulses?
    channel = int(sys.argv[4])       # channel
    trigger_delay = 0
    fibre_delay = 0
    pulse_height = 16383

    server.init_channel(channel, pulse_number, pulse_delay, trigger_delay, pulse_width, pulse_height, fibre_delay)
    try:
        server.trigger_averaged()
    except Exception,e:
        safe_exit(server,e)

    mean = None
    try:
        print "Waiting for sequence to finish..."
        while (mean == None):
            mean, rms, chan = server.read_pin_sequence()
    except Exception,e:
        safe_exit(server,e)
    except KeyboardInterrupt:
        safe_exit(server, "keyboard interrupt")

    print "\nPin: %s \nrms: %s\n" % (mean, rms)
