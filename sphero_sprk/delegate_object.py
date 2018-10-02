import struct

import bluepy
from collections import namedtuple

class DelegateObj(bluepy.btle.DefaultDelegate):
    """
    Delegate object that get calls when there is a notification
    """

    SPHERO_PKT_HEADER = namedtuple('SPHERO_PKT_HEADER', 'sop1 sop2 mrsp seq dlen')

    def __init__(self, sphero_obj, lock):
        bluepy.btle.DefaultDelegate.__init__(self)
        self._sphero_obj = sphero_obj
        self._callback_dict = {}
        self._wait_list = {}
        self._data_group_callback = {}
        self._enabled_group = []
        self._buffer_bytes = b''
        self._notification_lock = lock
        self._mask_callbacks = []
        self.start_time = None

    def update_callbacks(self):
        self._mask_callbacks = self._sphero_obj.get_mask_order()

    def register_callback(self, seq, callback):
        self._callback_dict[seq] = callback

    def register_async_callback(self, group_name, callback):
        self._data_group_callback[group_name] = callback
        self._enabled_group = list(set(self._enabled_group) | set([group_name]))

    def handle_callbacks(self, packet):
        # unregister callback
        callback = self._callback_dict.pop(packet[3])
        MRSP = packet[2]
        dlen = (packet[4] - 1)
        data = []
        if (dlen > 0):
            data = packet[5:5 + dlen]
        # parse the packet
        callback(MRSP, data)

    def wait_for_resp(self, seq, timeout=None):
        # this is a dangerous function, it waits for a response in the handle notification part
        self._wait_list[seq] = None;
        while (self._wait_list[seq] == None):
            # time.sleep(0.1)
            with self._notification_lock:
                self._sphero_obj._device.waitForNotifications(timeout)
        return self._wait_list.pop(seq)

    def wait_for_sim_response(self, seq, timeout=None):
        # this is a dangerous function, it waits for a response in the handle notification part
        self._wait_list[seq] = None;
        while (self._wait_list[seq] == None):
            # time.sleep(0.1)
            with self._notification_lock:
                self._sphero_obj._device.waitForNotifications(timeout)
        data = self._wait_list.pop(seq)
        return (len(data) == 6 and data[0] == 255)

    def verify_checksum(self, data):
        data_length = int.from_bytes(data[3:5], 'big') - 1  # minus one for the checksum_val

        pkt_lent = len(data)

        my_sum = sum(data[3:data_length + 3]) % 255
        chk_sum = (my_sum ^ 65535) % 255

        chk_sum_start = data_length + 5
        chk_sum_end = data_length + 6
        pkt_chk_sum = int.from_bytes(data[chk_sum_start:chk_sum_end], 'big')
        return chk_sum == pkt_chk_sum

    def process_sensor_pkt(self, active_masks, data):
        if (len(active_masks) == 0):
            return

        data_length = int.from_bytes(data[3:5], 'big') - 1  # minus one for the checksum_val

        total_processed = 0

        start = 5
        stop = 5

        for mask in active_masks:
            len_mask = mask['len'] * 2
            stop = len_mask + start
            sub_data = data[start:stop]

            mask['callback'](sub_data)
            total_processed = total_processed + len_mask
            start = stop

            if (total_processed >= data_length):
                break

        if (total_processed != data_length):
            print("Data Length Did not match mask list")

    def parse_pkt(self, data):
        '''
        Parse a single Packet
        :param data: packet
        :return:
        '''

        if (data[1] == 255):
            # get the sequence number and check if a callback is assigned
            if (data[3] in self._callback_dict):
                self.handle_callbacks(data)
            # check if we have it in the wait list
            elif (data[3] in self._wait_list):
                self._wait_list[data[3]] = data
            # simple response
            elif (len(data) == 6 and data[0] == 255 and data[2] == 0):
                pass
                # print("receive simple response for seq:{}".format(data[3]))
            else:
                print("unknown response:{}".format(data))
            # Sync Message
        elif (data[1] == 254):
            ##print("receive async")
            # Async Message
            if (data[2] == int.from_bytes(b'\x03', 'big')):
                # the message is sensor data streaming

                self.process_sensor_pkt(self._mask_callbacks, data)

                # self.process_sensor_package(data, mask_list)

            elif (data[2] == int.from_bytes(b'\x09', 'big')):
                # orbbasic error message:
                print("orbBasic Error Message:")
                print(data[2:])
            elif (data[2] == int.from_bytes(b'\x0A', 'big')):
                print(data[2:])
            else:
                print("unknown async response:{}".format(data))
        else:
            pass

    def process_buffer(self, bytes):
        '''
        Process all bytes, returning any bytes left over from a complete packet
        :param bytes:
        :return: remaining bytes
        '''

        while (len(bytes) > 0):
            buffer_len = len(bytes)

            if(buffer_len < 6): # Always a checksum byte
                return  bytes

            # Get Header, and try to process a packet
            header = DelegateObj.SPHERO_PKT_HEADER._make(struct.unpack('<BBBBB', bytes[0:5]))

            pkt_len = header.dlen + 5

            if(buffer_len >= pkt_len):
                pkt_bytes = bytes[0:pkt_len]
                self.parse_pkt(pkt_bytes)
                bytes = bytes[pkt_len:]
            else:
                return bytes

        return  bytes

    def handleNotification(self, cHandle, data):

        # Check to see if the data is len(0), if so return

        # merge data with any incomplete instance
        self._buffer_bytes = self._buffer_bytes + data

        if(len(self._buffer_bytes) < 10):
            return

        self._buffer_bytes = self.process_buffer(self._buffer_bytes)

        if(self._buffer_bytes is None):
            self._buffer_bytes = b''

