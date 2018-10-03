#!/usr/bin/python3

import binascii
import os
import threading

import bluepy
import yaml
import sphero_sprk.util as util
from sphero_sprk.timeout import Timeout

from sphero_sprk.delegate_object import DelegateObj
from sphero_sprk.sphero_constants import CMD_CODES, MACRO_CODES

#should it be in a different format?
RobotControlService = "22bb746f2ba075542d6f726568705327"
BLEService = "22bb746f2bb075542d6f726568705327"
AntiDosCharacteristic = "22bb746f2bbd75542d6f726568705327"
TXPowerCharacteristic = "22bb746f2bb275542d6f726568705327"
WakeCharacteristic = "22bb746f2bbf75542d6f726568705327"
ResponseCharacteristic = "22bb746f2ba675542d6f726568705327"
CommandsCharacteristic = "22bb746f2ba175542d6f726568705327"

# class DATA_MASK_LIST(object):
#     IMU_PITCH = bytes.fromhex("0004 0000")
#     IMU_ROLL = bytes.fromhex("0002 0000")
#     IMU_YAW = bytes.fromhex("0001 0000")
#     ACCEL_X = bytes.fromhex("0000 8000")
#     ACCEL_Y = bytes.fromhex("0000 4000")
#     ACCEL_Z = bytes.fromhex("0000 2000")
#     GYRO_X = bytes.fromhex("0000 1000")
#     GYRO_Y = bytes.fromhex("0000 0800")
#     GYRO_Z = bytes.fromhex("0000 0400")


class Sphero(object):

    RAW_MOTOR_MODE_OFF = "00"
    RAW_MOTOR_MODE_FORWARD = "01"
    RAW_MOTOR_MODE_REVERSE = "02"
    RAW_MOTOR_MODE_BRAKE = "03"
    RAW_MOTOR_MODE_IGNORE = "04"

    MASK_ORDER = [
        {"name":"accel_raw", "size":3},
        {"name":"gyro_raw", "size":3},
        {"name":"emf_raw", "size":2},
        {"name":"pwm_raw", "size":2},
        {"name":"imu_filtered", "size":3},
        {"name":"accel_filtered", "size":3},
        {"name":"gyro_filtered", "size":3},
        {"name":"emf_filtered", "size":2},
        {"name":"quarternion", "size":4},
        {"name":"odometer", "size":2},
        {"name":"accelone", "size":1},
        {"name":"velocity", "size":2},
    ]

    def __init__(self, addr=None):


        if(addr == None):
            #search for sphero
            sphero_list = search_for_sphero()
            if(len(sphero_list) == 0):
                raise Exception("No Sphero Found in Vicinity")
            addr = sphero_list[0]

        self._addr = addr
        self._connected = False
        self._seq_counter = 0
        self._stream_rate = 10
        #load the mask list
        with open(os.path.join(os.path.dirname(__file__),'data','mask_list1.yaml'),'r') as mask_file1:
            self._mask_list1 = yaml.load(mask_file1)
        with open(os.path.join(os.path.dirname(__file__),'data','mask_list2.yaml'),'r') as mask_file2:
            self._mask_list2 = yaml.load(mask_file2)
        self._data_mask1 = bytes.fromhex("0000 0000")
        self._data_mask2 = bytes.fromhex("0000 0000")

        self._active_masks = {}
        self._active_mask_callbacks = []

        self._notification_lock = threading.RLock()
        #start a listener loop

    def connect(self):
        '''
        Connects the sphero with the address given in the constructor
        :return: True if it succeeds, False if it times out
        '''

        try:
            with Timeout(5):
                self._device = bluepy.btle.Peripheral(self._addr, addrType=bluepy.btle.ADDR_TYPE_RANDOM)
        except Timeout:
            raise TimeoutError("Device Timed out")

        self._notifier = DelegateObj(self, self._notification_lock)
        #set notifier to be notified
        self._device.withDelegate(self._notifier)

        self._devModeOn()
        self._connected = True #Might need to change to be a callback format
        #get the command service
        cmd_service = self._device.getServiceByUUID(RobotControlService)
        self._cmd_characteristics = {}
        characteristic_list = cmd_service.getCharacteristics()
        for characteristic in characteristic_list:
            uuid_str = binascii.b2a_hex(characteristic.uuid.binVal).decode('utf-8')
            self._cmd_characteristics[uuid_str] = characteristic

        return True


    def _devModeOn(self):
        """
        A sequence of read/write that enables the developer mode
        """
        service = self._device.getServiceByUUID(BLEService)
        characteristic_list = service.getCharacteristics()
        #make it into a dict
        characteristic_dict = {}
        for characteristic in characteristic_list:
            uuid_str = binascii.b2a_hex(characteristic.uuid.binVal).decode('utf-8')
            characteristic_dict[uuid_str] = characteristic

        characteristic = characteristic_dict[AntiDosCharacteristic]
        characteristic.write("011i3".encode(),True)
        characteristic = characteristic_dict[TXPowerCharacteristic]
        characteristic.write((7).to_bytes(1, 'big'),True)
        characteristic = characteristic_dict[WakeCharacteristic]
        characteristic.write((1).to_bytes(1, 'big'),True)       

    def command(self, cmd, data, resp=True):
        """
        cmd - (str) Hex String that is the command's code(ff, no need to put \\x in front)
        data - [bytes/str/int] an array of values with what to send. We will reformat int and string
        resp - (bool) whether the command will only return after we get an acknowledgement from Sphero. If set to false, sphero will be set to NOT even send a response to save bandwidth
        -----
        
        return - (tuple) A tuple with the first element being sequence number and second element being the response if blocked, None if not 
        """      
        #format data  
        data_list = self._format_data_array(data)
        #set the sop2 based on the blocking command
        sop2 = "ff" if resp else "fe"
        #send command
        seq_num = self._send_command(sop2, cmd.value[0], cmd.value[1], data_list)
        #check if blocking
        if(resp):
            resp = self._notifier.wait_for_resp(seq_num)
            #return the sequence number and response 
            return (seq_num, resp)
        else:
            return (seq_num, None)

    def _send_command(self,sop2,did,cid,data_list):
        
        sop1 = binascii.a2b_hex("ff")
        sop2 = binascii.a2b_hex(sop2)
        did = bytes([did])
        cid = bytes([cid])
        seq_val = self._get_sequence()
        seq = seq_val.to_bytes(1,"big")
        dlen = (util.count_data_size(data_list)+1).to_bytes(1,"big")#add one for checksum
        packet = [sop1,sop2,did,cid,seq,dlen] + data_list
        packet += [util.cal_packet_checksum(packet[2:]).to_bytes(1,'big')] #calculate the checksum
        #write the command to Sphero
        #print("cmd:{} packet:{}".format(cid, b"".join(packet)))
        with self._notification_lock:
            self._cmd_characteristics[CommandsCharacteristic].write(b"".join(packet))
        return seq_val

    def _listening_loop(self):
        pass
        #while(self._listening_flag):
            #with self._notification_lock:
                #self._device.waitForNotifications(0.001)

    def _get_sequence(self):
        val = self._seq_counter
        self._seq_counter += 1
        self._seq_counter = self._seq_counter%256
        return val

    def _format_data_array(self, arr):
        """
        helper function that converts int or string to bytes, just want to decrease the number of codes
        """
        if isinstance(arr,list): 
            for i,value in enumerate(arr):
                if isinstance(value, str):
                    arr[i] = binascii.a2b_hex(value)
                elif isinstance(value, int):
                    arr[i] = value.to_bytes(1,'big')
        return arr

    """ CORE functionality """

    def ping(self):
        resp = self.command(CMD_CODES.CMD_PING, [], resp=True)
        return resp

    def version(self):
        #NOTE returning weird data not sure what's wrong
        seq_num = self._send_command("ff",0,2,[])
        response = self._notifier.wait_for_resp(seq_num)
        data_response = response[5:-1]
        version_data = {}
        version_data["RECV"] = hex(data_response[0])
        version_data["MDL"] = hex(data_response[1])
        version_data["HW"] = data_response[2]
        version_data["MSA-ver"] = data_response[3]
        version_data["MSA-rev"] = data_response[4]
        version_data["BL"] = hex(data_response[5])
        return version_data

    def get_device_name(self):
        seq_num = self._send_command("ff",0x00, 0x11,[])
        response = self._notifier.wait_for_resp(seq_num)
        name_data = {}
        name_data["name"] = str(response[5:21],'utf-8').rstrip(' \t\r\n\0')
        name_data["bta"] = str(response[21:33],'utf-8')
        name_data["color"] = str(response[33:36],'utf-8')
        return name_data


    """ Sphero functionality """

    def config_locator(self, x, y, yaw_tare, flag=0, resp=False):


        x_bytes = x.to_bytes(2, byteorder='big')
        y_bytes = y.to_bytes(2, byteorder='big')
        yaw_tare_bytes = yaw_tare.to_bytes(2, byteorder='big')

        data = [flag, x_bytes[0], x_bytes[1], y_bytes[0], y_bytes[1],
                yaw_tare_bytes[0], yaw_tare_bytes[1]]
        self.command(CMD_CODES.CMD_LOCATOR, data, resp=False)

    def roll(self, speed, heading, resp=False):
        """
        Roll the ball towards the heading

        speed - (int) speed
        heading - (int) which direction, 0 - 359
        resp - (bool) whether the code will wait for comfirmation from Sphero
        """
        heading_bytes = heading.to_bytes(2,byteorder='big')
        data = [speed,heading_bytes[0],heading_bytes[1], 1]
        #send command
        self.command(CMD_CODES.CMD_ROLL,data, resp=resp)


    def boost(self):
        raise NotImplementedError

    def set_stabilization(self, flag, resp=False):

        data = [b'01'] if flag else [b'00']

        self.command(CMD_CODES.CMD_SET_STABILIZ, data, resp=resp)

    def set_tail_light(self, brightness, resp=False):

        data = [brightness]
        self.command(CMD_CODES.CMD_SET_BACK_LED, data, resp=resp)


    def set_heading(self, heading, resp=False):
        '''
        change the heading of the robot

        Note: Only setting to 0 seems to be working
        :param heading: (int) heading in the range of 0-355
        :param resp:(bool) Whether to receive comfirmation response from sphero
        :return:
        '''
        heading_bytes = heading.to_bytes(2,byteorder='big')
        data = [heading_bytes[0],heading_bytes[1]]
        #send command
        resp = self.command(CMD_CODES.CMD_SET_HEADING,data, resp=resp)
        pass

    def set_rgb_led(self, red, green, blue, persist = False, resp=False):
        '''
        Set the color of Sphero's LED
        :param red: (int) Color of red in range 0-255
        :param green: (int) Color of green in range 0-255
        :param blue: (int) Color of blue in range 0-255
        :param persist: Whether to set as default color
        :param resp: Whether to wait for response
        :return:
        '''
        #set data
        data = [red, green, blue, int(persist)]
        #send command
        self.command(CMD_CODES.CMD_SET_RGB_LED, data, resp=resp)


    def get_rgb_led(self):
        """
        Get the color of Sphero's LED
        ----
        return - tuple of the color in RGB
        """

        #set the correct command
        (seq_num, resp) = self.command(CMD_CODES.CMD_GET_RGB_LED,[])
        #parse the response packet and make sure it's correct
        if resp and resp[4] == 4:
            MRSP = resp[2]
            red = resp[5]
            green = resp[6]
            blue = resp[7]
            return (red, green, blue)
        else:
            return None

    def _handle_mask(self,group_name, mask=1, remove=False):
        '''
        Setup Mask for Data
        :param group_name: Name of Group in Yaml File
        :param mask: Which YAML file to use (1 or 2)
        :param remove: Whether this is a remove action
        :return:
        '''

        if(remove):
            optr = util.XOR_mask
        else:
            optr = util.OR_mask

        if(mask==1):
            for i,group in enumerate(self._mask_list1):
                if(group["name"] == group_name):
                    for i, value in enumerate(group["values"]):
                        self._data_mask1 = optr(self._data_mask1, bytes.fromhex(value["mask"]))
        elif(mask==2):
            for i,group in enumerate(self._mask_list2):
                if(group["name"] == group_name):
                    for i, value in enumerate(group["values"]):
                        self._data_mask2 = optr(self._data_mask2, bytes.fromhex(value["mask"]))


    def _send_data_command(self,rate,mask1,mask2,sample=1):
        N = ((int)(400/rate)).to_bytes(2,byteorder='big')
        #N = (40).to_bytes(2,byteorder='big')
        M = (sample).to_bytes(2,byteorder='big')
        PCNT = (0).to_bytes(1,'big')
        #MASK2 = (mask2).to_bytes(4,'big')
        data = [N,M, mask1 ,PCNT,mask2]
        resp = self.command(CMD_CODES.CMD_SET_DATA_STREAMING,data, resp=True) #make sure sphero actully receive this
        return resp


    def _stop_data_stream(self, group_name, mask_id = 1):
        #handle mask
        self._handle_mask(group_name, mask=mask_id, remove=True)
        self._send_data_command(self._stream_rate, self._data_mask1, (0).to_bytes(4, 'big'))

    def get_mask_order(self):
        mask_order = []

        mask_list = self._active_masks

        for mask in Sphero.MASK_ORDER:
            name = mask['name']

            if(name in mask_list):
                if(not mask_list[name] is None):
                    mask_order.append({'len': mask['size'], 'callback': self._active_masks[name]})

        return mask_order

    def add_mask(self, mask, callback):
        self._active_masks[mask] = callback

    def remove_mask(self, mask):
        self._active_masks[mask] = None

    def update_streaming(self, rate=10):
        '''
        Update Streaming
        :param rate: Rate of Streaming (Make sure factor of 400Hz)
        :return:
        '''
        self._stream_rate = rate
        self._send_data_command(rate, self._data_mask1, self._data_mask2)
        self._notifier.update_callbacks()
        pass

    def set_stream_callback(self, name, callback, mask_id = 1):
        '''
        Set a callback that streams the specified data to the callback

        :param name: Name of Group, must match mask_list1 or mask_list2
        :param callback: (function) function that we will pass the information when there is a callback
        :param mask_id: Which mask (1 or 2)
        :return:
        '''

        #first we register the callback with the notifier
        self._notifier.register_async_callback(name,callback)
        self.add_mask(name, callback)

        # enable mask
        self._handle_mask(name, mask=mask_id)

    def remove_stream_callback(self, name, mask_id = 1):
        self._handle_mask(name, mask=mask_id, remove=True)

    def set_stabilization(self,bool_flag, resp=False):
        """
        Enable/Disable stabilization of Sphero

        bool_flag - (bool) stabilization on/off
        resp[Optional] - (bool) whether the code will wait for comfirmation from Sphero, default to False
        """
        data = ["01" if bool_flag else "00"]
        self.command(CMD_CODES.CMD_SET_STABILIZ,data, resp=resp)


    def set_raw_motor_values(self,lmode,lpower,rmode,rpower, resp=False):
        """
        Set the raw motor values of Sphero
        lmode - (str) the hex string(without \\x) of the mode
        lpower - (int) the value of the power from 0-255
        rmode - (str) the hex string(without \\x) of the mode
        rpower - (int) the value of the power from 0-255
        resp[Optional] - (bool) whether the code will wait for comfirmation from Sphero, default to False
        """
        data = [lmode, int(lpower), rmode, int(rpower)]
        
        #By default, we going to cancel it
        self.command(CMD_CODES.CMD_SET_RAW_MOTORS,data, resp=resp)

    """ About MACRO  """

    def abort_macro(self, id_):
        """
        Abort the current macro with the given ID
        id - (int) the ID of the macro to stop
        """
        data = [id_]
        self.command(CMD_CODES.CMD_ABORT_MACRO,data)

    def run_macro(self, id_):
        """
        Start the macro with the given ID
        id_ - (int) the 8-bit ID of the macro
        """
        data = [id_]
        self.command(CMD_CODES.CMD_RUN_MACRO,data)

    """ OrbBasic the programming language """

    STORAGE_RAM = "00"
    STORAGE_PERSISTENT = "01"

    def erase_orb_basic_storage(self, area, block=True):
        """
        Erase any existing program in the stored area
        area - (str) hex name of the area to be cleaned
        """
        data = [area]
        seq = self.command(CMD_CODES.CMD_ERASE_ORBBAS, data)
        if(block):
            return self._notifier.wait_for_sim_response(seq)
        else:
            return True

    def run_orb_basic_program(self, area, start_line):
        """
        Run a the orb_basic program stored in that area
        area - (str) hex name of the area
        start_line - (int) the decimal line number to start
        """
        data = [area,start_line.to_bytes(2,byteorder='big')]

        seq = self.command(CMD_CODES.CMD_EXEC_ORBBAS, data)
        return self._notifier.wait_for_sim_response(seq)

    def abort_orb_basic_program(self):
        """
        Abort the orb_basic program
        """
        data = []
        seq = self.command(CMD_CODES.CMD_ABORT_ORBBAS, data)
        return self._notifier.wait_for_sim_response(seq)

    def append_orb_basic_fragment(self, area,val):
        """
        Append the value into ht orb basic given the area
        val - (list of strings) the command broken down into a list of hex values
        area - (str) hex name of the area
        """
        val.insert(0, area)
        data = val
        seq = self.command(CMD_CODES.CMD_APPEND_FRAG,data)
        if self._notifier.wait_for_sim_response(seq):
            pass
        else:
            print("error in appending orbbasic fragments")

    def append_orb_basic_line(self, area,code):
        """
        Append the line to the existing code
        """
        #first convert the line into list of bytes
        code_list = []
        for c in code:
            code_list.append(bytes(c,encoding="UTF-8"))
        if len(code_list) == 0:
            code_list.append(b'\x00') # NULL in the end
        #code_list.append(b'\x0a')#append the terminating line
        #send it to next part of the program
        self.append_orb_basic_fragment(area,code_list)




