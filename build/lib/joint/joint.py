import enum
import struct
from crccheck.crc import Crc32Mpeg2 as CRC32
import time
import serial

Index = enum.IntEnum('Index', [
    'Header',
    'DeviceID',
    'PackageSize',
    'Command',
    'SoftwareVersion',
    'HardwareVersion',
    # //////
    'ErrorCount',
    'CurrentPosition',
    'CurrentSpeed',
    'LimitSwitchStatus',
    'BusVoltage',
    'V5Voltage',
    # //// Position Control Parameters //////
    'MinPosition',
    'MaxPosition',
    'MinSpeed_POS',
    'MaxSpeed_POS',
    'MaxAccel_POS',
    'GoalPosition',
    'EncoderDeadband_POS',
    'Trajectory_time',
    'Trajectory_accel',
    'Trajectory_maxSpeed',
    'LimitSwitch_enable',
    # ////// Velocity Control Parameters //////
    'MaxSpeed_VELOCITY',
    'GoalSpeed',
    'Acceleration_VELOCITY',
    # ////////// General /////////////////////
    'StepperRunMode',
    'IsEncoderFeedBack',
    'Baudrate',
    'TorqueEn',
    'PIOMode0',
    'PIOMode1',
    'PIOMode2',
    'PIOMode3',
    'PIOData0',
    'PIOData1',
    'PIOData2',
    'PIOData3',
    'ExternalEncoder',
    'TestValue_i_1',
    'TestValue_i_2',
    'TestValue_i_3',
    'TestValue_i_4',
    'TestValue_f_1',
    'TestValue_f_2',
    'TestValue_f_3',
    'TestValue_f_4',
    'TestValue_b_1',
    'TestValue_b_2',
    'TestValue_b_3',
    'TestValue_b_4',
    'CRCValue',
], start=0)

class Commands(enum.IntEnum):
    PING = 0x00
    READ = 0x01
    WRITE = 0x02
    REBOOT = 0x10
    EEPROM_WRITE = 0x20
    CALIBRATE = 0x25
    BL_JUMP = 0x30
    WRITE_SYNC = 0x40
    READ_SYNC = 0x41
    SLOW_STOP = 0x60
    STOP = 0x61
    ACK = 0x80
    WRITE_ACK = 0x80 | 0x02
    EEPROM_WRITE_ACK = 0x20 | 0x02

class _Data:
    def __init__(self, index, var_type, rw=True, value=0):
        self.__index = index
        self.__type = var_type
        self.__size = struct.calcsize(self.__type)
        self.__value = value
        self.__rw = rw

    def value(self, value=None):
        if value is None:
            return self.__value
        elif self.__rw:
            self.__value = struct.unpack('<' + self.__type,
                                         struct.pack('<' + self.__type, value))[0]

    def index(self) -> enum.IntEnum:
        return self.__index

    def size(self) -> int:
        return self.__size

    def type(self) -> str:
        return self.__type

class Joint:
    _BATCH_ID = 254
    def __init__(self, ID) -> None:
        self.__ack_size = 0
        self.vars = [
            _Data(Index.Header, 'B', False, 0x55),
            _Data(Index.DeviceID, 'B'),
            _Data(Index.PackageSize, 'B'),
            _Data(Index.Command, 'B'),
            _Data(Index.SoftwareVersion, 'I'),
            _Data(Index.HardwareVersion, 'I'),
            _Data(Index.ErrorCount, 'I'),
            _Data(Index.CurrentPosition, 'i'),
            _Data(Index.CurrentSpeed, 'I'),
            _Data(Index.LimitSwitchStatus, 'B'),
            _Data(Index.BusVoltage, 'H'),
            _Data(Index.V5Voltage, 'H'),
            # ////////////////////////////////////////
            _Data(Index.MinPosition, 'i'),
            _Data(Index.MaxPosition, 'i'),
            _Data(Index.MinSpeed_POS, 'i'),
            _Data(Index.MaxSpeed_POS, 'i'),
            _Data(Index.MaxAccel_POS, 'i'),
            _Data(Index.GoalPosition, 'i'),
            _Data(Index.EncoderDeadband_POS, 'I'),
            _Data(Index.Trajectory_time, 'f'),
            _Data(Index.Trajectory_accel, 'f'),
            _Data(Index.Trajectory_maxSpeed, 'f'),
            _Data(Index.LimitSwitch_enable, 'B'),
            # ////// Velocity Control Parameters //////
            _Data(Index.MaxSpeed_VELOCITY, 'i'),
            _Data(Index.GoalSpeed, 'i'),
            _Data(Index.Acceleration_VELOCITY, 'i'),
            # ////////// General /////////////////////
            _Data(Index.StepperRunMode, 'B'),
            _Data(Index.IsEncoderFeedBack, 'B'),
            _Data(Index.Baudrate, 'I'),
            _Data(Index.TorqueEn, 'B'),
            _Data(Index.PIOMode0, 'B'),
            _Data(Index.PIOMode1, 'B'),
            _Data(Index.PIOMode2, 'B'),
            _Data(Index.PIOMode3, 'B'),
            _Data(Index.PIOData0, 'I'),
            _Data(Index.PIOData1, 'I'),
            _Data(Index.PIOData2, 'I'),
            _Data(Index.PIOData3, 'I'),
            _Data(Index.ExternalEncoder, 'I'),
            _Data(Index.TestValue_i_1, 'i'),
            _Data(Index.TestValue_i_2, 'i'),
            _Data(Index.TestValue_i_3, 'i'),
            _Data(Index.TestValue_i_4, 'i'),
            _Data(Index.TestValue_f_1, 'f'),
            _Data(Index.TestValue_f_2, 'f'),
            _Data(Index.TestValue_f_3, 'f'),
            _Data(Index.TestValue_f_4, 'f'),
            _Data(Index.TestValue_b_1, 'b'),
            _Data(Index.TestValue_b_2, 'b'),
            _Data(Index.TestValue_b_3, 'b'),
            _Data(Index.TestValue_b_4, 'b'),
            _Data(Index.CRCValue, 'I')
        ]

        if ID > 255 or ID < 0:
            raise ValueError("Device ID must be between 0 and 255!")
        else:
            self.vars[Index.DeviceID].value(ID)

    def get_ack_size(self):
        return self.__ack_size

    def set_variables(self, index_list=None, value_list=None, ack=False):
        index_list = index_list or []
        value_list = value_list or []
        # Set command to WRITE or WRITE_ACK
        self.vars[Index.Command].value(Commands.WRITE_ACK if ack else Commands.WRITE)

        fmt_str = '<' + ''.join(var.type() for var in self.vars[:4])
        for idx in index_list:
            fmt_str += 'B' + self.vars[idx].type()

        self.__ack_size = struct.calcsize(fmt_str)
        struct_out = list(struct.pack(
            fmt_str,
            *[var.value() for var in self.vars[:4]],
            *[v for pair in zip(index_list, [self.vars[i].value() for i in index_list]) for v in pair]
        ))
        struct_out[Index.PackageSize] = len(struct_out) + self.vars[Index.CRCValue].size()
        self.vars[Index.CRCValue].value(CRC32.calc(struct_out))
        return bytes(struct_out) + struct.pack(
            '<' + self.vars[Index.CRCValue].type(),
            self.vars[Index.CRCValue].value()
        )

    def get_variables(self, index_list=None):
        index_list = index_list or []
        self.vars[Index.Command].value(Commands.READ)

        fmt_str = '<' + ''.join(var.type() for var in self.vars[:4])
        for _ in index_list:
            fmt_str += 'B'

        self.__ack_size = (
            struct.calcsize(fmt_str + self.vars[Index.CRCValue].type()) +
            sum(self.vars[idx].size() for idx in index_list)
        )

        struct_out = list(struct.pack(
            fmt_str,
            *[var.value() for var in self.vars[:4]],
            *[int(idx) for idx in index_list]
        ))
        struct_out[Index.PackageSize] = len(struct_out) + self.vars[Index.CRCValue].size()
        self.vars[Index.CRCValue].value(CRC32.calc(struct_out))
        return bytes(struct_out) + struct.pack(
            '<' + self.vars[Index.CRCValue].type(),
            self.vars[Index.CRCValue].value()
        )

    def reboot(self):
        self.vars[Index.Command].value(Commands.REBOOT)
        fmt_str = '<' + ''.join(var.type() for var in self.vars[:4])
        struct_out = list(struct.pack(fmt_str, *[var.value() for var in self.vars[:4]]))
        struct_out[Index.PackageSize] = len(struct_out) + self.vars[Index.CRCValue].size()
        self.vars[Index.CRCValue].value(CRC32.calc(struct_out))
        self.__ack_size = 0
        return bytes(struct_out) + struct.pack(
            '<' + self.vars[Index.CRCValue].type(),
            self.vars[Index.CRCValue].value()
        )

    def EEPROM_write(self, ack=False):
        self.vars[Index.Command].value(Commands.EEPROM_WRITE_ACK if ack else Commands.EEPROM_WRITE)
        fmt_str = '<' + ''.join(var.type() for var in self.vars[:4])
        struct_out = list(struct.pack(fmt_str, *[var.value() for var in self.vars[:4]]))
        struct_out[Index.PackageSize] = len(struct_out) + self.vars[Index.CRCValue].size()
        self.vars[Index.CRCValue].value(CRC32.calc(struct_out))
        self.__ack_size = struct.calcsize(fmt_str + self.vars[Index.CRCValue].type())
        return bytes(struct_out) + struct.pack(
            '<' + self.vars[Index.CRCValue].type(),
            self.vars[Index.CRCValue].value()
        )

    def ping(self):
        self.vars[Index.Command].value(Commands.PING)
        fmt_str = '<' + ''.join(var.type() for var in self.vars[:4])
        struct_out = list(struct.pack(fmt_str, *[var.value() for var in self.vars[:4]]))
        struct_out[Index.PackageSize] = len(struct_out) + self.vars[Index.CRCValue].size()
        self.vars[Index.CRCValue].value(CRC32.calc(struct_out))
        self.__ack_size = struct.calcsize(fmt_str + self.vars[Index.CRCValue].type())
        return bytes(struct_out) + struct.pack(
            '<' + self.vars[Index.CRCValue].type(),
            self.vars[Index.CRCValue].value()
        )

    def slow_stop(self):
        self.vars[Index.Command].value(Commands.SLOW_STOP)
        fmt_str = '<' + ''.join(var.type() for var in self.vars[:4])
        struct_out = list(struct.pack(fmt_str, *[var.value() for var in self.vars[:4]]))
        struct_out[Index.PackageSize] = len(struct_out) + self.vars[Index.CRCValue].size()
        self.vars[Index.CRCValue].value(CRC32.calc(struct_out))
        self.__ack_size = struct.calcsize(fmt_str + self.vars[Index.CRCValue].type())
        return bytes(struct_out) + struct.pack(
            '<' + self.vars[Index.CRCValue].type(),
            self.vars[Index.CRCValue].value()
        )

    def calibrate(self):
        self.vars[Index.Command].value(Commands.CALIBRATE)
        fmt_str = '<' + ''.join(var.type() for var in self.vars[:4])
        struct_out = list(struct.pack(fmt_str, *[var.value() for var in self.vars[:4]]))
        struct_out[Index.PackageSize] = len(struct_out) + self.vars[Index.CRCValue].size()
        self.vars[Index.CRCValue].value(CRC32.calc(struct_out))
        self.__ack_size = 0
        return bytes(struct_out) + struct.pack(
            '<' + self.vars[Index.CRCValue].type(),
            self.vars[Index.CRCValue].value()
        )

class Master:
    def __init__(self, portname, baudrate=115200):
        self.__driver_list = [Joint(255)] * 256
        if baudrate > 9500000 or baudrate < 1200:
            raise ValueError("Baudrate must be in range of 1200 to 9.5M")
        else:
            self.__baudrate = baudrate
            self.__post_sleep = 0
            self.__ph = serial.Serial(port=portname, baudrate=self.__baudrate, timeout=0.1)

    def __del__(self):
        try:
            self.__ph.close()
        except:
            pass

    def __write_bus(self, data):
        self.__ph.write(data)

    def __read_bus(self, size) -> bytes:
        self.__ph.flushInput()
        return self.__ph.read(size=size)

    def attach(self, joint: Joint):
        self.__driver_list[joint.vars[Index.DeviceID].value()] = joint

    def busy_wait(self, wait_time):
        start_time = time.time()
        while (time.time() - start_time) < wait_time:
            pass

    def parse_received(self, data):
        dev_id = data[Index.DeviceID]
        payload = data[4:-4]
        fmt_str = '<'

        i = 0
        while i < len(payload):
            idx = payload[i]
            fmt_str += 'B' + self.__driver_list[dev_id].vars[idx].type()
            i += self.__driver_list[dev_id].vars[idx].size() + 1

        unpacked = list(struct.unpack(fmt_str, payload))
        grouped = zip(*(iter(unpacked),) * 2)
        for group in grouped:
            self.__driver_list[dev_id].vars[group[0]].value(group[1])

    def set_variables(self, id, idx_val_pairs=None, ack=False) -> list:
        idx_val_pairs = idx_val_pairs or []
        index_list = [pair[0] for pair in idx_val_pairs]
        value_list = [pair[1] for pair in idx_val_pairs]

        self.__write_bus(self.__driver_list[id].set_variables(index_list, value_list, ack))
        if ack and self.__read_ack(id):
            return [self.__driver_list[id].vars[index].value() for index in index_list]
        return [None]

    def get_variables(self, id, index_list) -> list:
        self.__write_bus(self.__driver_list[id].get_variables(index_list))
        if self.__read_ack(id):
            return [self.__driver_list[id].vars[index].value() for index in index_list]
        else:
            return [None]

    def __read_ack(self, id) -> bool:
        ret = self.__read_bus(self.__driver_list[id].get_ack_size())
        if len(ret) == self.__driver_list[id].get_ack_size():
            if CRC32.calc(ret[:-4]) == struct.unpack('<I', ret[-4:])[0]:
                if ret[int(Index.PackageSize)] > 8:
                    self.parse_received(ret)
                    return True
                else:
                    return True
            else:
                return False
        else:
            return False

    def reboot(self, id):
        self.__write_bus(self.__driver_list[id].reboot())
        time.sleep(self.__post_sleep)

    def EEPROM_write(self, id, ack=False):
        index_list = [int(index) for index in Index]
        self.__write_bus(self.__driver_list[id].EEPROM_write(ack))
        if ack and self.__read_ack(id):
            return [self.__driver_list[id].vars[index].value() for index in index_list]
        time.sleep(self.__post_sleep)
        return [None]

    def ping(self, id):
        self.__write_bus(self.__driver_list[id].ping())
        if self.__read_ack(id):
            return True

    def calibrate(self, id):
        self.__write_bus(self.__driver_list[id].calibrate())
        time.sleep(self.__post_sleep)

    def slow_stop(self, id):
        self.__write_bus(self.__driver_list[id].slow_stop())
        time.sleep(self.__post_sleep)

    def set_variable_combined(self, val_indexes, values_lists, device_size, ack=False) -> list:
        """
        - val_indexes should be list [val_index1, val_index2, ...]
        - values_lists should be list of lists [[value1,...], [value2,...], ...]
        """
        if len(val_indexes) != len(values_lists):
            raise ValueError("val_indexes and values_lists must have same length")
        for value_list in values_lists:
            if len(value_list) != device_size:
                raise ValueError("Each values_list must match device_size")

        self.__driver_list[255].vars[Index.Command].value(Commands.WRITE_SYNC)
        fmt_str = '<' + 'BBBBB'
        for val_index in val_indexes:
            fmt_str += 'B' + self.__driver_list[255].vars[val_index].type() * device_size

        indexes_and_values = []
        for val_index, value_list in zip(val_indexes, values_lists):
            indexes_and_values.append(val_index)
            indexes_and_values.extend(value_list)

        struct_out = list(struct.pack(
            fmt_str,
            *[var.value() for var in self.__driver_list[255].vars[:4]],
            device_size,
            *indexes_and_values
        ))
        struct_out[int(Index.PackageSize)] = len(struct_out) + self.__driver_list[255].vars[Index.CRCValue].size()
        self.__driver_list[255].vars[Index.CRCValue].value(CRC32.calc(struct_out))

        ready = bytes(struct_out) + struct.pack(
            '<' + self.__driver_list[255].vars[Index.CRCValue].type(),
            self.__driver_list[255].vars[Index.CRCValue].value()
        )
        self.__write_bus(ready)
        return [None]
