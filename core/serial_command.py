#!/usr/bin/env python
#
# serial_command
#
# SerialCommand
#
# Command functions to send to the Tellie
# control box
#
# Author: Matt Mottram
#         <m.mottram@sussex.ac.uk>
#         <e.leming@sussex.ac.uk>
#
# History:
# 2013/03/08: First instance
# 2013/10/21: Added new classes for different chips, pep8
# 2015/03/19: Added external trigger functionality
###########################################
###########################################

import serial
import tellie_exception
import re
import sys
import time
from common import tellie_logger, parameters

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
_max_temp_probe = 64

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
_cmd_ph_hi = "L"
_cmd_ph_lo = "M"
_cmd_ph_end = "P"
_cmd_pw_hi = "Q"
_cmd_pw_lo = "R"
_cmd_pw_end = "S"
_cmd_pn_hi = "H"
_cmd_pn_lo = "G"
_cmd_pd = "u"
_cmd_td = "d"
_cmd_fd = "e"
_cmd_temp_select_lower = "n"
_cmd_temp_read_lower = "T"
_cmd_temp_select_upper = "f"
_cmd_temp_read_upper = "k"
_cmd_enable_ext_trig = "A"
_cmd_disable_ext_trig = "B"
_cmd_fire_average_ext_trig_lower = "p"
_cmd_fire_average_ext_trig_upper = "b"
_cmd_fire_ext_trig = "F"

class SerialCommand(object):
    """Serial command object.
    Base class, different chips then inheret from this.
    """

    def __init__(self, port_name=None):
        """Initialise the serial command"""
        if not port_name:
            #self._port_name = "/dev/tty.usbserial-FTE3C0PG"
            self._port_name = "tty.usbserial-FTGA2OCZ"
        else:
            self._port_name = port_name
        # This is the same as a sleep, but with the advantage of breaking
        # if enough characters are seen in the buffer.
        self._port_timeout = 0.3
        self._serial = None
        self.logger = tellie_logger.TellieLogger.get_instance()
        try:
            self._serial = serial.Serial(port=self._port_name, timeout=self._port_timeout)
            self.logger.debug("Serial connection open: %s" % self._serial)
        except serial.SerialException, e:
            raise tellie_exception.TellieSerialException(e)
        #cache current settings - remove need to re-command where possible
        #channel specific settings
        self._channel = [] #always a list
        self._current_pw = [-999]*96
        self._current_ph = [-999]*96
        self._current_fd = [-999]*96
        #global settings
        self._current_pn = None
        self._current_pd = None
        self._current_td = None
        #information on whether the channel is being fired
        self._firing = 0 #must wait for firing to complete
        self._reading = 0 #once a read command has been sent, dont send again!
        #temperature settings
        self._current_temp_probe = None
        #if a new channel is selected should force setting all new parameters
        #restriction only lifted once a fire command has been called
        self._force_setting = False
        #send a reset, to ensure the RTS is set to false
        self.reset()
        #send a clear channel command, just in case
        self.clear_channel()
        #By default stop tellie waiting for external trigger (i.e. running in slave mode).
        #Slave mode can be re-instated later if required.
        self._send_command(_cmd_disable_ext_trig)

    def __del__(self):
        """Deletion function"""
        if self._serial:
            self._send_command(_cmd_disable_ext_trig) # Stop accecpting external trigs
            self._serial.close()

    def _check_clear_buffer(self):
        """Many commands expect an empty buffer, fail if they are not!
        """
        buffer_read = self._serial.read(100)
        if buffer_read != "":
            raise tellie_exception.TellieException("Buffer not clear: %s" % (buffer_read))

    def _send_command(self, command, readout=True, buffer_check=None):
        """Send a command to the serial port.
        Command can be a chr/str (single write) or a list.
        Lists are used for e.g. a high/low bit command where
        the high bit could finish with an endline (i.e. endstream)"""
        self.logger.debug("_send_command:%s" % command)
        if type(command) is str:
            command = [command]
        if type(command) is not list:
            raise tellie_exception.TellieException("Command is not a list: %s %s" % (command, type(command)))
        try:
            for c in command:
                self._serial.write(c)
        except:
            raise tellie_exception.TellieException("Lost connection with TELLIE control!")
        if not buffer_check: # assume returns same as input
            buffer_check = ''
            for c in command:
                buffer_check += c
        if readout is True:
            # One read command (with default timeout of 0.1s) should be
            # enough to get all the chars from the readout.
            buffer_read = self._serial.read(len(buffer_check))
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
                message = "Unexpected buffer output:\nsaw: %s, remainder %s\nexpected: %s" % (buffer_read, remainder, buffer_check)
                self.logger.warn(message)
                raise tellie_exception.TellieException(message)
            else:
                self.logger.debug("success reading buffer:%s" % buffer_read)
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
                raise tellie_exception.TellieException("Cannot run command, in firing mode")
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
            raise tellie_exception.TellieException("Cannot run channel command, no channel selected")
        if len(self._channel)!=1:
            raise tellie_exception.TellieException("Cannot run channel command, must have single channel selected: %s" % (self._channel))
        self._send_setting_command(command=command, buffer_check=buffer_check, while_fire=while_fire)

    def reset(self):
        """Send a reset command!

        Assumes that the port is open (which it is by default)
        """
        self.logger.debug("Reset!")
        self._serial.setRTS(True)
        # sleep, just in case
        time.sleep(3.0)
        self._serial.setRTS(False)
        # close the port and reopen?
        time.sleep(3.0)

    def enable_external_trig(self, while_fire=False):
        """Tell TELLIE to fire on any external trigger.

        Can send a fire command while already in fire mode if required."""
        self.logger.debug("Enable ext triggering mode")
        if self._firing is True and while_fire is False:
            raise tellie_exception.TellieException("Cannot set ext. trig, already in firing mode")
        self._send_command(_cmd_enable_ext_trig)

    def trigger(self):
        """Fire single pulse upon receiving an external trigger. The final PIN reading of a squence
        fired via this method can be read with the read_pin() function. 
        """
        self.logger.debug("Accepting %i triggers!" % self._current_pn)
        if self._firing is True:
            raise tellie_exception.TellieException("Cannot fire, already in firing mode")
        self._send_command(_cmd_fire_ext_trig, False)
        self._firing = True

    def trigger_averaged(self):
        """Request averaged pin reading for externally triggered pulses."""
        self.logger.debug("Accepting %i triggers for averaging!" % self._current_pn)
        if len(self._channel)!=1:
            raise tellie_exception.TellieException("Cannot fire with >1 channel")
        if self._firing is True:
            raise tellie_exception.TellieException("Cannot fire, already in firing mode")
        if self._channel <= 56: #up to box 7
            cmd = _cmd_fire_average_ext_trig_lower
        else:
            cmd = _cmd_fire_average_ext_trig_upper
        self._send_command(cmd, False)
        self._firing = True

    def read_triggered_average_pin(self, interval=0.1, timeout=5.):
        """Poll buffer for averaged PIN readout after an event. """
        if self._firing is False:
            raise tellie_exception.TellieException("TELLIE not in firing mode...")
        start = time.time()
        pattern = re.compile(r"""\d+""")
        while (time.time() - start) < timeout:
            output = self._serial.read(100)
            pin = pattern.findall(output)
            if len(pin):
                self._firing = False
                pin = pattern.findall(output) 
                return pin, self._channel[0]
            time.sleep(interval)
        return False

    def fire(self, while_fire=False):
        """Fire tellie, place class into firing mode.
        Can send a fire command while already in fire mode if required."""
        self.logger.debug("Fire!")
        if self._firing is True and while_fire is False:
            raise tellie_exception.TellieException("Cannot fire, already in firing mode")
        self.check_ready()
        # Set readout to false when firing (must read
        # averaged pin at some later time).
        cmd = None
        buffer_check = _cmd_fire_series
        #if the series is less than 0.5 seconds, also check for the end of sequence
        if (self._current_pn * self._current_pd) < 500:
            buffer_check += _buffer_end_sequence
            self._send_command(_cmd_fire_series, buffer_check=buffer_check)
        else:
            self._send_command(_cmd_fire_series, buffer_check=buffer_check)
            self._firing = True #still firing
        self._force_setting = False

    def fire_averaged(self, while_fire=False):
        """Fire a sequence of pulses, requesting an average PIN reading to be calcualted. 
        Average PIN readings can only be calculated for a single channel.
        """
        self.logger.debug("Fire averaged!")
        if len(self._channel)!=1:
            raise tellie_exception.TellieException("Cannot fire with >1 channel")
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
            raise tellie_exception.TellieException("Cannot fire, already in firing mode")
        if self._channel <= 56: #up to box 7
            cmd = _cmd_read_single_lower
        else:
            cmd = _cmd_read_single_upper
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
            raise tellie_exception.TellieException("Cannot fire, already in firing mode")
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
    
    def stop_triggering(self):
        """Disable external triggering and set firing to false. If in averaging mode, will cause
        current average to appear in buffer. Both the raw buffer and any intergers found returned
        from function."""
        self.logger.debug("Stop accepting triggers!")
        self._send_command(_cmd_disable_ext_trig, False)
        pattern = re.compile(r"""\d+""")
        buffer_contents = self._serial.read(100)
        self._firing = False
        return buffer_contents, pattern.findall(buffer_contents)

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
                    raise tellie_exception.TellieException("Cannot read pin for channel %s, already trying to read channel %s" % (channel, self._channel[0]))
            else:
                self.select_channel(channel)
            if self._channel[0] <= 56: #up to box 7
                cmd = _cmd_read_single_lower
                print "read!"
            else:
                cmd = _cmd_read_single_upper
                print "read!"                
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
                raise tellie_exception.TellieException("Bad number of PIN readouts: %s %s" % (len(pin), pin))
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
            raise tellie_exception.TellieException("Cannot read pin, not in firing mode")
        pattern = re.compile(r"""\d+""")
        output = self._serial.read(100)
        self.logger.debug("BUFFER: %s" % output)
        pin = pattern.findall(output)
        if len(pin)>1:
            self._firing = False
            raise tellie_exception.TellieException("Bad number of PIN readouts: %s %s" % (len(pin), pin))
        elif len(pin) == 0:
            return None, None
        self._firing = False
        channel_dict = {self._channel[0]: pin[0]}
        return channel_dict, self._channel

    def check_ready(self):
        """Check that all settings have been set"""
        not_set = []
        for channel in self._channel:
            if self._current_pw[channel-1] is None:
                not_set += ["Pulse width"]
            if self._current_ph[channel-1] is None:
                not_set += ["Pulse height"]
            if self._current_fd[channel-1] is None:
                not_set += ["Fibre delay"]
        if self._current_pn is None:
            not_set += ["Pulse number"]
        if self._current_pd is None:
            not_set += ["Pulse delay"]
        if self._current_td is None:
            not_set += ["Trigger delay"]
        #print not_set
        if not_set != []:
            raise tellie_exception.TellieException("Undefined options: %s" % (", ".join(opt for opt in not_set)))

    def clear_channel(self):
        """Unselect the channel"""
        self.logger.debug("Clear channel")
        self._send_command(_cmd_channel_clear)
        self._channel = []

    def clear_channel_settings(self, channel):
        """Clear settings for a specific channel"""
        self._current_pw[channel-1] = None
        self._current_ph[channel-1] = None
        self._current_fd[channel-1] = None

    def clear_global_settings(self):
        """Clear settings that affect all channels"""
        self._current_pn = None
        self._current_pd = None
        self._current_td = None

    def select_channel(self, channel):
        """Select a channel"""
        if type(channel) is not int:
            channel = int(channel)
        if self._channel != []:
            if self._channel == [channel]:
                #channel already selected
                return
        self.logger.debug("Select channel %s %s" % (channel, type(channel)))
        command = _cmd_channel_select_single_start+chr(channel)+_cmd_channel_select_single_end
        buffer_check = "B"+str((int(channel)-1)/8+1)+_cmd_channel_select_single_end
        self._send_command(command=command, buffer_check=buffer_check)
        self._channel = [channel]

    def select_channels(self, channels):
        """Select multiple channels, expects list for channels"""
        self.logger.debug("Select channels %s %s" % (channels, type(channels)))
        self.clear_channel()
        command = _cmd_channel_select_many_start
        for channel in channels:
            print channel
            command += chr(channel)
        command += _cmd_channel_select_many_end
        buffer_check = "B"+str((int(channels[0])-1)/8+1)+_cmd_channel_select_many_end
        print "SEND CHANNELS", "CMD", command, "BUF", buffer_check
        self._send_command(command=command, buffer_check=buffer_check)
        print "DONE!"
        self._channel = channels

    def set_pulse_height(self, par):
        """Set the pulse height for the selected channel"""
        if len(self._channel) != 1:
            raise tellie_exception.TellieException("Cannot set parameter with channels set as %s" % (self._channel))
        if par == self._current_ph[self._channel[0]] and not self._force_setting:
            pass #same as current setting
        else:
            self.logger.debug("Set pulse height %s %s" % (par, type(par)))
            command, buffer_check = command_pulse_height(par)
            self._send_channel_setting_command(command=command, buffer_check=buffer_check)
            self._current_ph[self._channel[0]] = par

    def set_pulse_width(self, par, while_fire=False):
        """Set the pulse width for the selected channel.
        This is the only setting that can be modified while in firing mode."""
        if len(self._channel) != 1:
            raise tellie_exception.TellieException("Cannot set parameter with channels set as %s" % (self._channel))
        if par == self._current_pw[self._channel[0]] and not self._force_setting:
            pass #same as current setting
        else:
            self.logger.debug("Set pulse width %s %s" % (par, type(par)))
            command, buffer_check = command_pulse_width(par)
            if while_fire and self._firing:
                self._send_channel_setting_command(command=command, while_fire=while_fire)
            else:
                self._send_channel_setting_command(command=command, buffer_check=buffer_check)
            self._current_pw[self._channel[0]] = par

    def set_fibre_delay(self, par):
        """Set the fibre (channel) delay for the selected channel"""
        if len(self._channel) != 1:
            raise tellie_exception.TellieException("Cannot set parameter with channels set as %s" % (self._channel))
        if par == self._current_fd[self._channel[0]] and not self._force_setting:
            pass
        else:
            self.logger.debug("Set Fibre delay %s %s" % (par, type(par)))
            command, buffer_check = command_fibre_delay(par)
            self._send_channel_setting_command(command=command, buffer_check=buffer_check)
            self._current_fd[self._channel[0]] = par

    def set_pulse_number(self, par):
        """Set the number of pulses to fire (global setting)"""
        if par == self._current_pn and not self._force_setting:
            pass
        else:
            self.logger.debug("Set pulse number %s %s" % (par, type(par)))
            command, buffer_check = command_pulse_number(par)
            self._send_global_setting_command(command=command, buffer_check=buffer_check)
            self._current_pn = par

    def set_pulse_delay(self, par):
        """Set the delay between pulses (global setting)"""
        if par == self._current_pd and not self._force_setting:
            pass
        else:
            self.logger.debug("Set pulse delay %s %s" % (par, type(par)))
            command, buffer_check = command_pulse_delay(par)
            self._send_global_setting_command(command=command, buffer_check=buffer_check)
            self._current_pd = par

    def set_trigger_delay(self, par):
        """Set the trigger delay (global setting)"""
        if par == self._current_td and not self._force_setting:
            pass
        else:
            self.logger.debug("Set trigger delay %s %s" % (par, type(par)))
            command, buffer_check = command_trigger_delay(par)
            self._send_global_setting_command(command=command, buffer_check=buffer_check)
            self._current_td = par

    def select_temp_probe(self, par):
        """Select the temperature probe to read"""
        if par == self._current_temp_probe and not self._force_setting:
            pass
        else:
            self.logger.debug("Select temperature probe %s %s" % (par, type(par)))
            command, buffer_check = command_select_temp(par)
            self._send_command(command=command, readout=False)
            self._current_temp_probe = par
            #read the temperature twice
            #first reading is always junk
            #second reading is sometimes junk
            self.read_temp()
            self.read_temp()

    def read_temp(self, timeout=1.0):
        """Read the temperature"""
        if not self._current_temp_probe:
            raise tellie_exception.TellieException("Cannot read temp: no probe selected")
        cmd = ""
        if self._current_temp_probe < 33 and self._current_temp_probe > 0:
            cmd = _cmd_temp_read_lower
        elif self._current_temp_probe < _max_temp_probe + 1:
            cmd = _cmd_temp_read_upper
        else:
            raise tellie_exception.TellieException("Temp probe not in known range")
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
                raise tellie_exception.TellieException("Temperature read timeout!")
        if len(temp)>1:
            raise tellie_exception.TellieException("Bad number of temp readouts: %s %s" % (len(temp), temp))
        temp = float(temp[0])
        return temp

    def disable_external_trigger(self):
        """Disable the external trigger"""
        self._send_command(command=_cmd_disable_ext_trig)

    ########################################
    # Commands just to check current settings
    def get_pulse_delay(self):
        """Get the pulse delay
        """
        return self._current_pd

    def get_pulse_number(self):
        """Get the pulse delay
        """
        return self._current_pn
        


class SNO6C(SerialCommand):
    """Object for the main TELLIE setup.
    """

    def __init__(self, port_name=None):
        """Initialisation command.
        """
        super(SNO6C, self).__init__(port_name)


class SNO6CKapustinsky(SerialCommand):
    """Object for the Kapustinsky single channel (4) setup.
    """

    def __init__(self, port_name=None):
        """Initialisation command.
        """
        super(SNO6CKapustinsky, self).__init__(port_name)

    def select_channel(self, channel):
        """Select a channel: override"""
        if type(channel) is not int:
            channel = int(channel)
        if self._channel != []:
            if self._channel == [channel]:
                #channel already selected
                return
        self.logger.debug("Select channel %s %s" % (channel, type(channel)))
        command = _cmd_channel_select_single_start+chr(channel)+_cmd_channel_select_single_end
        self._send_command(command=command, buffer_check="N")
        self._channel = [channel]

##################################################
# Command options and corresponding buffer outputs
#

def command_pulse_height(par):
    """Get the command to set a pulse height"""
    if par > _max_pulse_height or par < 0:
        raise tellie_exception.TellieException("Invalid pulse height: %s" % par)
    hi = par >> 8
    lo = par & 255
    command = [_cmd_ph_hi+chr(hi)]
    command+= [_cmd_ph_lo+chr(lo)]
    command+= [_cmd_ph_end]
    buffer_check = _cmd_ph_hi + _cmd_ph_lo + _cmd_ph_end
    return command, buffer_check


def command_pulse_width(par):
    """Get the command to set a pulse width"""
    if par > _max_pulse_width or par < 0:
        raise tellie_exception.TellieException("Invalid pulse width: %s %s %s" % (par, _max_pulse_width, par>_max_pulse_width))
    hi = par >> 8
    lo = par & 255
    command = [_cmd_pw_hi+chr(hi)]
    command+= [_cmd_pw_lo+chr(lo)+_cmd_pw_end]
    buffer_check = _cmd_pw_hi + _cmd_pw_lo + _cmd_pw_end
    return command, buffer_check


def command_pulse_number(par):
    """Get the command to set a pulse number"""
    if par > _max_pulse_number or par < 0:
        raise tellie_exception.TellieException("Invalid pulse number: %s" % (par))
    par = int(par)
    adjusted, actual_par, hi, lo = parameters.pulse_number(par)
    if adjusted is True:
        raise tellie_exception.TellieException("Invalid pulse number: %s" % (par))
    command = [_cmd_pn_hi+chr(hi)]
    command+= [_cmd_pn_lo+chr(lo)]
    buffer_check = _cmd_pn_hi + _cmd_pn_lo
    return command, buffer_check


def command_pulse_delay(par):
    """Get the command to set a pulse delay"""
    if par > _max_pulse_delay or par < 0:
        raise tellie_exception.TellieException("Invalid pulse delay: %s" % par)
    ms = int(par)
    us = int((par-ms)*250)
    command = [_cmd_pd+chr(ms)]
    command+= [chr(us)]
    buffer_check = _cmd_pd
    return command, buffer_check


def command_trigger_delay(par):
    """Get the command to set a trigger delay"""
    if par > _max_trigger_delay or par < 0:
        raise tellie_exception.TellieException("Invalid trigger delay: %s" % par)
    command = [_cmd_td+chr(par/5)]
    buffer_check = _cmd_td
    return command, buffer_check


def command_fibre_delay(par):
    """Get the command to set a fibre delay"""
    if par > _max_fibre_delay or par < 0:
        raise tellie_exception.TellieException("Invalid fibre delay: %s" % par)
    adjusted, adj_delay, setting = parameters.fibre_delay(par)
    print "COMMAND", par, adjusted, adj_delay, setting
    if adjusted is True:
        raise tellie_exception.TellieException("Invalid delay: %s" % (par))
    command = [_cmd_fd+chr(setting)]
    buffer_check = _cmd_fd
    return command, buffer_check


def command_select_temp(par):
    """Select a temperature probe to read"""
    if par > _max_temp_probe or par < 0:
        raise tellie_exception.TellieException("Invalid temp. probe number: %s" % par)
    cmd = ""
    par = par
    if par < 33 and par > 0:
        cmd = _cmd_temp_select_lower
        par = par
    elif par < _max_temp_probe + 1:
        cmd = _cmd_temp_select_upper
        par = par - 32 #lower
    else:
        raise tellie_exception.TellieException("Invalid temp. probe number: %s" % par)
    command = [cmd+chr(par)]
    return command, None # nothing in buffer
