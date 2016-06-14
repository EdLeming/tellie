#!/usr/bin/env python
#
# tellie_server
#
# classes: SerialCommand
#          TellieException
#          TellieSerialException
#          ThreadException
#
# Launches a xmlrpc server containing the 
# SerialCommand class used to talk to the 
# tellie control box. 
#
# Author: EdLeming
#         <e.leming@sussex.ac.uk>
#
###########################################
###########################################
from SimpleXMLRPCServer import SimpleXMLRPCServer
import serial
import tellie_exception
import re
import sys
import time
import math
_snotDaqLog = False
try:
    from snotdaq import logger
    _snotDaqLog = True
except ImportError:
    from common import tellie_logger

class TellieException(Exception):
    """General exception for the Tellie command modules"""

    def __init__(self, error):
        Exception.__init__(self, error)


class TellieSerialException(Exception):
    """Exception when communicating with the Serial Port"""

    def __init__(self, error):
        Exception.__init__(self, error)


class ThreadException(Exception):
    """Exception raised specific to threading issues"""

    def __init__(self, error):
        Exception.__init__(self, error)

# Global max values
_max_pulse_height = 16383
_max_pulse_width = 16383
_max_lo = 255.
_max_pulse_delay = 256.020
_min_pulse_delay = 0.1
_max_trigger_delay = 1275
_max_fibre_delay = 127.5
_max_pulse_number = 65025
_max_pulse_number_upper = 255
_max_pulse_number_lower = 255
_max_temp_probe = 64.
_max_fibre_delay = 63.75 #ns
_max_trigger_delay = 1275 #ns

class ParametersClass(object):
    """Class to emulate common/parameters """

    def __init__(self):
        # Some init stuff
        self._something = 0;

    def pulse_number(self, number):
        adjusted = False
        if type(number)!=int:
            raise Exception("PN must be an integer")
        if number > _max_pulse_number:
            raise Exception("PN must be < %d.  You set %d" % (65025, number))
        #number = max_pulse_number
        #adjusted = True
        hi = -1
        lo = -1
        diff = 100000 # bigger than max pn
        for i in range(1, 256):
            # assume hi is i
            lo_check = number/i
            if lo_check > 255:
                lo_check = 255
            check = i * lo_check
            if math.fabs(check - number) < diff:
                diff = math.fabs(check - number)
                hi = i
                lo = lo_check
            if check == number:
                break
        actual_par = hi * lo
        if actual_par != number:
            adjusted = True
        return adjusted, actual_par, hi, lo

    def trigger_delay(self, delay):
        adjusted = False
        delay = float(delay)
        if delay > _max_trigger_delay or delay < 0:
            raise Exception("TD must be >%s and <%s" % (0, _max_trigger_delay))
        parameter = int(round(delay)/5)
        adj_delay = parameter * 5
        if delay != adj_delay:
            adjusted = True
        return adjusted, adj_delay, parameter

    def fibre_delay(self, delay):
        adjusted = False
        delay = float(delay)
        if delay > _max_fibre_delay or delay < 0:
            raise Exception("FD must be >%s and <%s" % (0, _max_fibre_delay))
        parameter = int(round(delay * 4.))
        adj_delay = float(parameter) / 4.
        if delay != adj_delay:
            adjusted = True
        return adjusted, adj_delay, parameter

# Global char commands to call functions on the ELLIE pic.
_cmd_fire_continuous = "a"
_cmd_read_single_lower = "r"
_cmd_read_single_upper = "m"
_cmd_fire_average_lower = "s"
_cmd_fire_average_upper = "U"
_cmd_fire_series = "g"
_buffer_end_sequence = "K"
_cmd_stop = "X"
_cmd_channel_clear = "C"
_cmd_channel_select_single_start = "I"
_cmd_channel_select_single_end = "N"
_cmd_channel_select_many_start = "J"
_cmd_channel_select_many_end = "E"
_cmd_pulse_height_hi = "L"
_cmd_pulse_height_lo = "M"
_cmd_pulse_height_end = "P"
_cmd_pulse_width_hi = "Q"
_cmd_pulse_width_lo = "R"
_cmd_pulse_width_end = "S"
_cmd_pulse_number_hi = "H"
_cmd_pulse_number_lo = "G"
_cmd_pulse_delay = "u"
_cmd_trigger_delay = "d"
_cmd_fibre_delay = "e"
_cmd_temp_select_lower = "n"
_cmd_temp_read_lower = "T"
_cmd_temp_select_upper = "f"
_cmd_temp_read_upper = "k"
_cmd_distable_trig_in = "B"


class SerialCommand(object):
    """Contains a serial command object.
    """

    def __init__(self, port_name = "/dev/tty.usbserial-FTGA2OCZ", server_port = 5030, logger_port = 4001,
                 port_timeout = 0.3):
        '''Initialise function: open serial connection.
        '''
        self._port_name = port_name
        self._port_timeout = port_timeout
        self._logger_port = logger_port

        # Set up logger stuff.
        if _snotDaqLog:
            self.logger = logger.Logger()
            try:
                self.logger.connect('tellie', 'minard', self._logger_port)
            except Exception as e:
                self.logger.warn("unable to connect to log server: %s" % str(e))

        else:
            self.logger = tellie_logger.TellieLogger.get_instance()
            self.logger.set_debug_mode(True)
        self.logger.notice("Tellie connected to log server!")

        # Set up serial connection to tellie
        self._serial = None
        try:
            self._serial = serial.Serial(port=self._port_name, timeout=self._port_timeout)
            self.logger.debug("Serial connection open: %s" % self._serial)
        except serial.SerialException, e:
            raise tellie_exception.TellieSerialException(e)

        # Cache current settings - remove need to re-command where possible
        # Channel specific settings
        self._channel = [] #always a list
        self._current_pulse_width = [-999]*96
        self._current_pulse_height = [-999]*96
        self._current_fibre_delay = [-999]*96

        # Global settings
        self._current_pulse_number = None
        self._current_pulse_delay = None
        self._current_trigger_delay = None

        # Information on whether the channel is being fired
        self._firing = 0 #must wait for firing to complete
        self._reading = 0 #once a read command has been sent, dont send again!

        # Temperature settings
        self._current_temp_probe = None

        # If a new channel is selected should force setting all new parameters
        # restriction only lifted once a fire command has been called
        self._force_setting = False

        # Send a reset, to ensure the RTS is set to false
        self.reset()

        # Send a clear channel command, just in case
        self._clear_buffer()
        self.clear_channel()

    def __del__(self):
        """Deletion function"""
        if self._serial:
            self._serial.close()
        self.logger.warn("tellie server dropped out")

    def test(self):
        self.logger.notice("Tellie server responding")

    def _clear_buffer(self):
        """Clear any chars left in the buffer"""
        buffer_read = self._serial.read(100)
        if buffer_read != "":
            self.logger.debug("Buffer was not clear: %s" % buffer_read)

    def _check_clear_buffer(self):
        """Many commands expect an empty buffer, fail if they are not!
        """
        buffer_read = self._serial.read(100)
        if buffer_read != "":
            self.logger.warn("Buffer not clear: %s" % (buffer_read))

    def _send_command(self, command, readout=True, buffer_check=None, sleep_after_command=0.1):
        """Send a command to the serial port.
        Command can be a chr/str (single write) or a list.
        Lists are used for e.g. a high/low bit command where
        the high bit could finish with an endline (i.e. endstream)

        sleep_after_command is the default time to sleep between each write command"""
        self.logger.debug("_send_command:%s" % command)

        if type(command) is str:
            command = [command]
        if type(command) is not list:
            raise TellieException("Command is not a list: %s %s" % (command, type(command)))
        try:
            for c in command:
                self._serial.write(c)
                time.sleep(sleep_after_command)
        except:
            raise TellieException("Lost connection with TELLIE control!")

        if not buffer_check: # assume returns same as input
            buffer_check = ''
            for c in command:
                buffer_check += c

        if readout is True:
            # One read command (with default timeout of 0.1s) should be
            # enough to get all the chars from the readout.
            buffer_read = self._serial.read(len(buffer_check))
            attempt = 0
            self.logger.debug("READ: %s\tCHECK: %s" % (buffer_read, buffer_check))
            while (len(buffer_read) != len(buffer_check)) and attempt<10:
                self.logger.debug("Didn't read correct no of chars, read again")
                # First, try reading again
                time.sleep(0.1)
                buffer_read += self._serial.read(len(buffer_check))
                attempt += 1

            if str(buffer_read)!=str(buffer_check):
                self.logger.debug("problem reading buffer, send %s, read %s" % (command, buffer_read))
                #clear anything else that might be in there
                time.sleep(0.1)
                remainder = self._serial.read(100)
                self._serial.write("X") # send a stop
                time.sleep(0.1)
                self._serial.write("C") # send a clear
                time.sleep(0.1)
                self._serial.read(100)
                if buffer_read == '\x00':
                    self.logger.warn("Looks like power was lost to tellie...It may still be off?")
                    # Re-run 
                    self._send_command(command, readout, buffer_check, sleep_after_command)
                message = "Unexpected buffer output:\nsaw: %s, remainder %s\nexpected: %s\n" % (buffer_read, remainder, buffer_check)
                self.logger.warn(message)
                raise TellieException(message)
            else:
                self.logger.debug("success reading buffer: %s" % buffer_read)
        else:
            self.logger.debug("not a readout command")

    def _send_setting_command(self, command, buffer_check=None, while_fire=False):
        """Send non-firing command.
        All of these should have a clear buffer before being used.  Can set
        while_fire to True to allow a non-fire command to be sent while firing
        (will cause PIN readout to be flushed to buffer).
        """
        self.logger.debug("Send non-firing command")
        if self._firing is True:
            if while_fire is False:
                raise TellieException("Cannot run command, in firing mode")
            else:
                #Assume that we CANNOT readout the buffer here!
                self._send_command(command=command, readout=False)
        else:
            self._check_clear_buffer()
            self._send_command(command=command, buffer_check=buffer_check)

    def _send_global_setting_command(self, command, buffer_check=None, while_fire=False):
        """Send non-firing command that affects all channels.
        Can set while_fire to True to allow a non-fire command to be sent
        while firing (will cause PIN readout to be flushed to buffer).
        """
        self.logger.debug("Send global setting command %s" % (command))
        self._send_setting_command(command=command, buffer_check=buffer_check, while_fire=while_fire)

    def _send_channel_setting_command(self, command, buffer_check=None, while_fire=False):
        """Send non-firing command for specific channel.
        Can set while_fire to True to allow a non-fire command to be sent while
        firing (will cause PIN readout to be flushed to buffer).
        """
        self.logger.debug("Send channel setting command %s" % (command))

        if not self._channel or self._channel == []:
            raise TellieException("Cannot run channel command, no channel selected")
        if len(self._channel)!=1:
            raise TellieException("Cannot run channel command, must have single channel selected: %s" % (self._channel))
        self._send_setting_command(command=command, buffer_check=buffer_check, while_fire=while_fire)

    def reset(self):
        """Send a reset command!

        Assumes that the port is open (which it is by default)
        """
        self.logger.debug("Reset!")            

        self._serial.setRTS(True)
        # sleep, just in case
        time.sleep(1.0)
        self._serial.setRTS(False)
        # close the port and reopen?
        time.sleep(1.0)

    def fire(self, while_fire=False):
        """Fire tellie, place class into firing mode.
        Can send a fire command while already in fire mode if required."""
        self.logger.debug("Fire!")

        if self._firing is True and while_fire is False:
            raise TellieException("Cannot fire, already in firing mode")
        self.check_ready()
        # Set readout to false when firing (must read
        # averaged pin at some later time).
        cmd = None
        buffer_check = _cmd_fire_series
        #if the series is less than 0.5 seconds, also check for the end of sequence
        if (self._current_pulse_number * self._current_pulse_delay) < 500:
            buffer_check += _buffer_end_sequence
            self._send_command(_cmd_fire_series, buffer_check=buffer_check)
        else:
            self._send_command(_cmd_fire_series, buffer_check=buffer_check)
            self._firing = True #still firing
        self._force_setting = False

    def fire_sequence(self, while_fire=False):
        """Fire in sequence mode, can only be done for a single channel.
        """
        self.logger.notice("Fire sequence on channel %s!" % self._channel)
        if len(self._channel)!=1:
            raise TellieException("Cannot fire with >1 channel")
        if self._current_pulse_number == 0:
            self.logger.warn("Requested to fire 0 pulses!")
            return 0
        self.check_ready()
        cmd = None
        if self._channel[0] <= 56: #up to box 7
            cmd = _cmd_fire_average_lower
        else:
            cmd = _cmd_fire_average_upper
        self._send_command(cmd, False)
        self._firing = True
        self._force_setting = False
            
    def fire_single(self):
        """Fire single pulse
        """
        if self._firing is True:
            raise TellieException("Cannot fire, already in firing mode")
        if self._channel <= 56: #up to box 7
            cmd = _cmd_fire_single_lower
        else:
            cmd = _cmd_fire_single_upper
        self._send_command(cmd, False)
        self._firing = True
        pin = self.read_pin(self._channel[0])
        while not pin:
            pin = self.read_pin(self._channel[0])
        return pin

    def fire_continuous(self):
        """Fire Tellie in continous mode.
        """
        if self._firing is True:
            raise TellieException("Cannot fire, already in firing mode")
        self._send_command(_cmd_fire_continuous, False)
        self._firing = True
        self._force_setting = False

    def read_buffer(self, n=100):
        return self._serial.read(n)

    def stop(self):
        """Stop firing tellie"""
        self.logger.debug("Stop firing!")
        self._send_command(_cmd_stop, False)
        buffer_contents = self._serial.read(100)
        self._firing = False
        return buffer_contents

    def read_pin(self, channel=None, timeout=2.0, final=True):
        """Read the pin diode output, should always follow a fire command,
        Provide channel number to select specific channel, otherwise, receive dict of all channels"""
        self.logger.debug("Read PINOUT")
        #if in firing mode, check the buffer shows the sequence has ended
        if self._firing:
            if self._serial.read(100) == _buffer_end_sequence:
                print "K in buffer"
                self._firing = False
            else:
                print "No K in buffer"
                return None, None
        if channel:
            if self._reading is True:
                if channel != self._channel[0]:
                    raise TellieException("Cannot read pin for channel %s, already trying to read channel %s" % (channel, self._channel[0]))
            else:
                self.select_channel(channel)
            if self._channel[0] <= 56: #up to box 7
                #cmd = _cmd_read_average_lower
                print "read!"
                cmd = _cmd_read_single_lower
            else:
                #cmd = _cmd_read_average_upper
                print "read!"
                cmd = _cmd_read_single_upper
            if not self._reading:
                self._send_command(cmd, False)
            pattern = re.compile(r"""\d+""")
            start = time.time()
            pin = []
            while (time.time()-start)<timeout:
                output = self._serial.read(100)
                pin = pattern.findall(output)
                if len(pin):
                    break
                time.sleep(0.1)
            if len(pin)>1:
                self._firing = False
                self._reading = False
                raise TellieException("Bad number of PIN readouts: %s %s" % (len(pin), pin))
            elif len(pin) == 0:
                self._reading = True
                return None, None
            self._reading = False
            if final is True:
                self._firing = False
            return pin[0], channel
        else:
            #check all PINs from the last firing sequence
            #need to store a copy of which pins were read
            channel_list = self._channel
            channel_dict = {}
            final_read = False
            for i, channel in enumerate(channel_list):
                if i == len(channel_list)-1:
                    final_read = True
                pin, _ = self.read_pin(channel, final=final_read)
                channel_dict[channel] = pin
            return channel_dict, channel_list

    def read_pin_sequence(self):
        """Read a pin from the sequence firing mode only.
        """
        self.logger.debug("Read PINOUT sequence")            
        if self._firing is not True:
            raise TellieException("Cannot read pin, not in firing mode")
        pattern = re.compile(r"""\d+""")
        output = self._serial.read(100)
        if _snotDaqLog == True:
            self.logger.log(logger.DEBUG, "BUFFER: %s" % output)
        else:
            self.logger.debug("BUFFER: %s" % output)
        numbers = pattern.findall(output)
        if len(numbers) == 0:
            self.logger.debug("Sequence doesn't appear to have finished..")
            return None
        if len(numbers) == 1:
            pin, rms = numbers[0], 0.
        elif len(numbers) == 3:
            pin, rms = numbers[0], "%s.%s" % (numbers[1], numbers[2])
        else:
            self.logger.warn("Bad number of PIN readouts: %s %s" % (len(numbers), numbers))
            return 0, 0, 0
        self._firing = False
        value_dict = {self._channel[0]: pin}
        rms_dict = {self._channel[0]: rms}
        return pin, rms, self._channel
        #return value_dict, rms_dict, self._channel

    def check_ready(self):
        """Check that all settings have been set"""
        not_set = []
        for channel in self._channel:
            if self._current_pulse_width[channel-1] is None:
                not_set += ["Pulse width"]
            if self._current_pulse_height[channel-1] is None:
                not_set += ["Pulse height"]
            if self._current_fibre_delay[channel-1] is None:
                not_set += ["Fibre delay"]
        if self._current_pulse_number is None:
            not_set += ["Pulse number"]
        if self._current_pulse_delay is None:
            not_set += ["Pulse delay"]
        if self._current_trigger_delay is None:
            not_set += ["Trigger delay"]
        self.logger.debug("The following parameters have not been set: %s" % not_set)

    def clear_channel(self):
        """Unselect the channel"""
        self.logger.debug("Clear channel")
        self._send_command(_cmd_channel_clear)
        self._channel = []

    def clear_channel_settings(self, channel):
        """Clear settings for a specific channel"""
        self._current_pulse_width[channel-1] = None
        self._current_pulse_height[channel-1] = None
        self._current_fibre_delay[channel-1] = None

    def clear_global_settings(self):
        """Clear settings that affect all channels"""
        self._current_pulse_number = None
        self._current_pulse_delay = None
        self._current_trigger_delay = None

    def select_channel(self, channel):
        """Select a channel"""
        if type(channel) is not int:
            channel = int(channel)
        if self._channel != []:
            if self._channel == [channel]:
                #channel already selected
                self.logger.debug("Channel already selected")
                return 0
        self.logger.notice("Select channel %s %s" % (channel, type(channel)))
        command, buffer_check = command_select_channel(channel)
        self._send_command(command=command, buffer_check=buffer_check)
        self._channel = [channel]
        self.logger.debug("About to return")
        return 0 # OK status

    def select_channels(self, channels):
        """Select multiple channels, expects list for channels"""
        self.logger.notice("Select channels %s %s" % (channels, type(channels)))
        self.clear_channel()
        command = _cmd_channel_select_many_start
        for channel in channels:
            self.logger.debug(channel)
            command += chr(channel)
        command += _cmd_channel_select_many_end
        buffer_check = "B"+str((int(channels[0])-1)/8+1)+_cmd_channel_select_many_end
        self.logger.debug("SEND CHANNELS", "CMD", command, "BUF", buffer_check)
        self._send_command(command=command, buffer_check=buffer_check)
        self._channel = channels

    def init_channel(self, channel, pulse_number, pulse_delay, trigger_delay,
                     pulse_width, pulse_height, fibre_delay):
        """Select and setup all channel settings.
        """
        self.logger.debug("inside init_channel")
        if self._firing:
            self.logger.debug("Currently in firing mode. Wait until firing has stopped before retrying channel init.")
            return 1
        self._force_setting = True
        self.select_channel(int(channel))
        self.set_pulse_number(int(pulse_number))
        self.set_pulse_delay(float(pulse_delay))
        self.set_trigger_delay(int(trigger_delay))
        self.set_pulse_width(int(pulse_width))
        self.set_pulse_height(int(pulse_height))
        self.set_fibre_delay(float(fibre_delay))

        # Return a dump of the settings
        settings = {"channels": self._channel,
                    "pulse_number": self._current_pulse_number,
                    "pulse_delay": self._current_pulse_delay,
                    "trigger_delay": self._current_trigger_delay,
                    "channel_settings": {}}
        for c in self._channel:
            self.logger.debug("Channel %i is set in init_channel" % c)
            settings["channel_settings"][str(c)] = {"pulse_width": self._current_pulse_width[c],
                                                    "pulse_height": self._current_pulse_height[c],
                                                    "fibre_delay": self._current_fibre_delay[c]}
        return settings

    def set_pulse_height(self, par):
        """Set the pulse height for the selected channel"""
        if len(self._channel) != 1:
            raise TellieException("Cannot set parameter with channels set as %s" % (self._channel))
        if par == self._current_pulse_height[self._channel[0]] and not self._force_setting:
            pass #same as current setting
        else:
            self.logger.notice("Set pulse height %s %s" % (par, type(par)))
            command, buffer_check = command_pulse_height(par)
            self._send_channel_setting_command(command=command, buffer_check=buffer_check)
            self._current_pulse_height[self._channel[0]] = par
        return 0

    def set_pulse_width(self, par):
        """Set the pulse width for the selected channel.
        This is the only setting that can be modified while in firing mode."""
        if len(self._channel) != 1:
            raise TellieException("Cannot set parameter with channels set as %s" % (self._channel))
        if par == self._current_pulse_width[self._channel[0]] and not self._force_setting:
            pass #same as current setting
        else:
            self.logger.notice("Set pulse width %s %s" % (par, type(par)))                
            command, buffer_check = command_pulse_width(par)
            self._send_channel_setting_command(command=command, buffer_check=buffer_check)
            self._current_pulse_width[self._channel[0]] = par
        return 0

    def set_fibre_delay(self, par):
        """Set the fibre (channel) delay for the selected channel"""
        if len(self._channel) != 1:
            raise TellieException("Cannot set parameter with channels set as %s" % (self._channel))
        if par == self._current_fibre_delay[self._channel[0]] and not self._force_setting:
            pass
        else:
            self.logger.notice("Set Fibre delay %s %s" % (par, type(par)))
            command, buffer_check = command_fibre_delay(par)
            self._send_channel_setting_command(command=command, buffer_check=buffer_check)
            self._current_fibre_delay[self._channel[0]] = par
        return 0

    def set_pulse_number(self, par):
        """Set the number of pulses to fire (global setting)"""
        if par == self._current_pulse_number and not self._force_setting:
            pass
        else:
            self.logger.notice("Set pulse number %s %s" % (par, type(par)))
            command, buffer_check = command_pulse_number(par)
            self._send_global_setting_command(command=command, buffer_check=buffer_check)
            self._current_pulse_number = par
        return 0

    def set_pulse_delay(self, par):
        """Set the delay between pulses (global setting)"""
        if par == self._current_pulse_delay and not self._force_setting:
            pass
        else:
            self.logger.notice("Set pulse delay %s %s" % (par, type(par)))
            command, buffer_check = command_pulse_delay(par)
            self._send_global_setting_command(command=command, buffer_check=buffer_check)
            self._current_pulse_delay = par
        return 0

    def set_trigger_delay(self, par):
        """Set the trigger delay (global setting)"""
        if par == self._current_trigger_delay and not self._force_setting:
            pass
        else:
            self.logger.notice("Set trigger delay %s %s" % (par, type(par)))
            command, buffer_check = command_trigger_delay(par)
            self._send_global_setting_command(command=command, buffer_check=buffer_check)
            self._current_trigger_delay = par
        return 0

    def select_temp_probe(self, par):
        """Select the temperature probe to read"""
        if par == self._current_temp_probe and not self._force_setting:
            pass
        else:
            self.logger.notice("Select temperature probe %s %s" % (par, type(par)))
            command, buffer_check = command_select_temp(par)
            self._send_command(command=command, readout=False)
            self._current_temp_probe = par
            #read the temperature twice
            #first reading is always junk
            #second reading is sometimes junk
            self.read_temp()
            self.read_temp()
        return 0

    def read_temp(self, timeout=1.0):
        """Read the temperature"""
        if not self._current_temp_probe:
            raise TellieException("Cannot read temp: no probe selected")
        cmd = ""
        if self._current_temp_probe < 33 and self._current_temp_probe > 0:
            cmd = _cmd_temp_read_lower
        elif self._current_temp_probe < _max_temp_probe + 1:
            cmd = _cmd_temp_read_upper
        else:
            raise TellieException("Temp probe not in known range")
        self._send_command(command=cmd, readout=False)
        pattern = re.compile(r"""[-+]?\d*\.\d+|\d+""")
        #wait for a few seconds before reading out
        temp = None
        start = time.time()
        while not temp:
            output = self._serial.read(100)
            self.logger.debug("Buffer: %s" % output)
            temp = pattern.findall(output)
            if time.time() - start > timeout:
                raise TellieException("Temperature read timeout!")
        if len(temp)>1:
            raise TellieException("Bad number of temp readouts: %s %s" % (len(temp), temp))
        temp = float(temp[0])
        return temp

    def disable_external_trigger(self):
        """Disable the external trigger"""
        self._send_command(command="B")

##################################################
# Command options and corresponding buffer outputs
#
# These are retained such that command chains may
# be called (e.g. set all settings) before running
# a buffer readout.

def command_select_channel(par):
    """Get the command to select a single channel"""
    command = _cmd_channel_select_single_start+chr(par)+_cmd_channel_select_single_end
    buffer_check = "B"+str((int(par)-1)/8+1)+_cmd_channel_select_single_end
    return command, buffer_check


def command_pulse_height(par):
    """Get the command to set a pulse height"""
    if par > _max_pulse_height or par < 0:
        raise TellieException("Invalid pulse height: %s" % par)
    hi = par >> 8
    lo = par & 255
    command = [_cmd_pulse_height_hi+chr(hi)]
    command+= [_cmd_pulse_height_lo+chr(lo)]
    command+= [_cmd_pulse_height_end]
    buffer_check = _cmd_pulse_height_hi + _cmd_pulse_height_lo + _cmd_pulse_height_end
    return command, buffer_check


def command_pulse_width(par):
    """Get the command to set a pulse width"""
    if par > _max_pulse_width or par < 0:
        raise TellieException("Invalid pulse width: %s %s %s" % (par, _max_pulse_width, par>_max_pulse_width))
    hi = par >> 8
    lo = par & 255
    command = [_cmd_pulse_width_hi+chr(hi)]
    command+= [_cmd_pulse_width_lo+chr(lo)+_cmd_pulse_width_end]
    buffer_check = _cmd_pulse_width_hi + _cmd_pulse_width_lo + _cmd_pulse_width_end
    return command, buffer_check


def command_pulse_number(par):
    """Get the command to set a pulse number"""
    if par > _max_pulse_number or par < 0:
        raise TellieException("Invalid pulse number: %s" % (par))
    par = int(par)
    parameters  = ParametersClass()
    adjusted, actual_par, hi, lo = parameters.pulse_number(par)
    if adjusted is True:
        raise TellieException("Invalid pulse number: %s" % (par))
    command = [_cmd_pulse_number_hi+chr(hi)]
    command+= [_cmd_pulse_number_lo+chr(lo)]
    buffer_check = _cmd_pulse_number_hi + _cmd_pulse_number_lo
    return command, buffer_check


def command_pulse_delay(par):
    """Get the command to set a pulse delay"""
    if par > _max_pulse_delay or par < 0:
        raise TellieException("Invalid pulse delay: %s" % par)
    ms = int(par)
    us = int((par-ms)*250)
    command = [_cmd_pulse_delay+chr(ms)]
    command+= [chr(us)]
    buffer_check = _cmd_pulse_delay
    return command, buffer_check


def command_trigger_delay(par):
    """Get the command to set a trigger delay"""
    if par > _max_trigger_delay or par < 0:
        raise TellieException("Invalid trigger delay: %s" % par)
    command = [_cmd_trigger_delay+chr(par/5)]
    buffer_check = _cmd_trigger_delay
    return command, buffer_check


def command_fibre_delay(par):
    """Get the command to set a fibre delay"""
    if par > _max_fibre_delay or par < 0:
        raise TellieException("Invalid fibre delay: %s" % par)
    parameters = ParametersClass()
    adjusted, adj_delay, setting = parameters.fibre_delay(par)
    print "COMMAND", par, adjusted, adj_delay, setting
    if adjusted is True:
        raise TellieException("Invalid delay: %s" % (par))
    command = [_cmd_fibre_delay+chr(setting)]
    buffer_check = _cmd_fibre_delay
    return command, buffer_check


def command_select_temp(par):
    """Select a temperature probe to read"""
    if par > _max_temp_probe or par < 0:
        raise TellieException("Invalid temp. probe number: %s" % par)
    cmd = ""
    par = par
    if par < 33 and par > 0:
        cmd = _cmd_temp_select_lower
        par = par
    elif par < _max_temp_probe + 1:
        cmd = _cmd_temp_select_upper
        par = par - 32 #lower
    else:
        raise TellieException("Invalid temp. probe number: %s" % par)
    command = [cmd+chr(par)]
    return command, None # nothing in buffer


##################################################
# Helper functions
def command_append(inputs, values):
    '''Pass in inputs as (command, buffer_check) and values to append.

    Inputs should be a list.
    Command should be returned as a list, buffer_check as a string.
    '''
    assert len(inputs) == len(values)
    if type(inputs[0]) is not list:
        inputs[0] = [inputs[0]]
    if type(values[0]) is not list:
        inputs[0] += [values[0]]
    else:
        inputs[0] += values[0]
    inputs[1] = "%s%s" % (inputs[1], values[1])
    return inputs

if __name__ == "__main__":
    server = SimpleXMLRPCServer(("0.0.0.0", 5030), allow_none=True)
    
    server.register_instance(SerialCommand(), allow_dotted_names=True)
    
    print "serving..."
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print "Exiting..."
