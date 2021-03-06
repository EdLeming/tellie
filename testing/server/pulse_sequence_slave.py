### sends a continuous pulse
from SimpleXMLRPCServer import SimpleXMLRPCServer
import xmlrpclib
import sys
from common import parameters as p

server = xmlrpclib.ServerProxy("http://localhost:5030")

def safe_exit(sc,e):
    print "Exit safely"
    print e
    sc.stop()
    sys.exit()

if __name__=="__main__":
    pulse_width = int(sys.argv[1])   # 1-16383
    #pulse_delay = float(sys.argv[2]) # time between subsequent pulses (1/rate) [ms]
    pulse_delay = 0                  # external trigger rate
    pulse_number = int(sys.argv[2])  # how many pulses?
    channel = int(sys.argv[3])       # channel
    trigger_delay = 0
    fibre_delay = 0
    pulse_height = p._max_pulse_height

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

    print "\nPIN: %s \nRMS: %s\n" % (mean, rms)
