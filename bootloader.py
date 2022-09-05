import sys
import serial
import sys
import time
import struct
from functools import reduce
class SerialConnection:
    """Wrap a serial.Serial connection and toggle reset and boot0."""

    # pylint: disable=too-many-instance-attributes

    def __init__(self, baud_rate=115200):
        """Construct a SerialConnection (not yet connected)."""
        self.serial_port = '/dev/ttyS0'
        self.baud_rate = baud_rate
        self.parity = serial.PARITY_NONE

        self.swap_rts_dtr = False
        self.reset_active_high = False
        self.boot0_active_low = False

        # don't connect yet; caller should use connect() separately
        self.serial_connection = None
        self._timeout = 5

    @property
    def timeout(self):
        """Get timeout."""
        return self._timeout

    @timeout.setter
    def timeout(self, timeout):
        """Set timeout."""
        self._timeout = timeout
        self.serial_connection.timeout = timeout

    def connect(self):
        """Connect to the UART serial port."""
        self.serial_connection = serial.Serial(
            port=self.serial_port,
            baudrate=self.baud_rate,
            # number of write_data bits
            bytesize=8,
            parity=self.parity,
            stopbits=1,
            # don't enable software flow control
            xonxoff=0,
            # don't enable RTS/CTS flow control
            rtscts=0,
            # set a timeout value, None for waiting forever
            timeout=self._timeout,
        )

    def disconnect(self):
        """Close the connection."""
        if not self.serial_connection:
            return

        self.serial_connection.close()
        self.serial_connection = None

    def write(self, *args, **kwargs):
        """Write the given data to the serial connection."""
        return self.serial_connection.write(*args, **kwargs)

    def read(self, *args, **kwargs):
        """Read the given amount of bytes from the serial connection."""
        return self.serial_connection.read(*args, **kwargs)

    def enable_reset(self, enable=True):
        """Enable or disable the reset IO line."""
        # reset on the STM32 is active low (0 Volt puts the MCU in reset)
        # but the RS-232 modem control DTR and RTS signals are active low
        # themselves, so these get inverted -- writing a logical 1 outputs
        # a low voltage, i.e. enables reset)
        level = int(enable)
        if self.reset_active_high:
            level = 1 - level

        if self.swap_rts_dtr:
            self.serial_connection.setRTS(level)
        else:
            self.serial_connection.setDTR(level)

    def enable_boot0(self, enable=True):
        """Enable or disable the boot0 IO line."""
        level = int(enable)

        # by default, this is active high
        if not self.boot0_active_low:
            level = 1 - level

        if self.swap_rts_dtr:
            self.serial_connection.setDTR(level)
        else:
            self.serial_connection.setRTS(level)

    def flush_imput_buffer(self):
        """Flush the input buffer to remove any stale read data."""
        self.serial_connection.reset_input_buffer()

class Stm32Loader:

    class Command:
        """STM32 native bootloader command values."""

        # pylint: disable=too-few-public-methods
        # FIXME turn into intenum

        # See ST AN3155, AN4872
        GET = 0x00
        GET_VERSION = 0x01
        GET_ID = 0x02
        READ_MEMORY = 0x11
        GO = 0x21
        WRITE_MEMORY = 0x31
        ERASE = 0x43
        READOUT_PROTECT = 0x82
        READOUT_UNPROTECT = 0x92
        # these not supported on BlueNRG
        EXTENDED_ERASE = 0x44
        WRITE_PROTECT = 0x63
        WRITE_UNPROTECT = 0x73

        # not used so far
        READOUT_PROTECT = 0x82
        READOUT_UNPROTECT = 0x92

        # not really listed under commands, but still...
        # 'wake the bootloader' == 'activate USART' == 'synchronize'
        SYNCHRONIZE = 0x7F

    class Reply:
        """STM32 native bootloader reply status codes."""

        # pylint: disable=too-few-public-methods
        # FIXME turn into intenum

        # See ST AN3155, AN4872
        ACK = 0x79
        NACK = 0x1F

    UID_ADDRESS = {
        # No unique id for these parts
        "F0": None,
        # ST RM0008 section 30.1 Unique device ID register
        # F101, F102, F103, F105, F107
        "F1": 0x1FFFF7E8,
        # ST RM0366 section 29.1 Unique device ID register
        # ST RM0365 section 34.1 Unique device ID register
        # ST RM0316 section 34.1 Unique device ID register
        # ST RM0313 section 32.1 Unique device ID register
        # F303/328/358/398, F301/318, F302, F37x
        "F3": 0x1FFFF7AC,
        # ST RM0090 section 39.1 Unique device ID register
        # F405/415, F407/417, F427/437, F429/439
        "F4": 0x1FFF7A10,
        # ST RM0385 section 41.2 Unique device ID register
        "F7": 0x1FF0F420,
        # ST RM0433 section 61.1 Unique device ID register
        "H7": 0x1FF1E800,
        # ST RM0394 47.1 Unique device ID register (96 bits)
        "L4": 0x1FFF7590,
        # ST RM0451 25.2 Unique device ID register (96 bits)
        "L0": 0x1FF80050,
        # ST RM0444 section 38.1 Unique device ID register
        "G0": 0x1FFF7590,
    }

    UID_SWAP = [[1, 0], [3, 2], [7, 6, 5, 4], [11, 10, 9, 8]]

    # Part does not support unique ID feature
    UID_NOT_SUPPORTED = 0
    # stm32loader does not know the address for the unique ID
    UID_ADDRESS_UNKNOWN = -1

    FLASH_SIZE_ADDRESS = {
        # ST RM0360 section 27.1 Memory size data register
        # F030x4/x6/x8/xC, F070x6/xB
        "F0": 0x1FFFF7CC,
        # ST RM0008 section 30.2 Memory size registers
        # F101, F102, F103, F105, F107
        "F1": 0x1FFFF7E0,
        # ST RM0366 section 29.2 Memory size data register
        # ST RM0365 section 34.2 Memory size data register
        # ST RM0316 section 34.2 Memory size data register
        # ST RM0313 section 32.2 Flash memory size data register
        # F303/328/358/398, F301/318, F302, F37x
        "F3": 0x1FFFF7CC,
        # ST RM0090 section 39.2 Flash size
        # F405/415, F407/417, F427/437, F429/439
        "F4": 0x1FFF7A22,
        # ST RM0385 section 41.2 Flash size
        "F7": 0x1FF0F442,
        # ST RM0433 61.2 Flash size
        "H7": 0x1FF1E880,
        # ST RM0394
        "L4": 0x1FFF75E0,
        # ST RM4510 25.1 Memory size register
        "L0": 0x1FF8007C,
        # ST RM0444 section 38.2 Flash memory size data register
        "G0": 0x1FFF75E0,
    }

    DATA_TRANSFER_SIZE = {
        "default": 256,
        # No unique id for these parts
        "F0": 256,  # bytes
        # ST RM0008 section 30.1 Unique device ID register
        # F101, F102, F103, F105, F107
        "F1": 256,  # bytes
        # ST RM0366 section 29.1 Unique device ID register
        # ST RM0365 section 34.1 Unique device ID register
        # ST RM0316 section 34.1 Unique device ID register
        # ST RM0313 section 32.1 Unique device ID register
        # F303/328/358/398, F301/318, F302, F37x
        "F3": 256,  # bytes
        # ST RM0090 section 39.1 Unique device ID register
        # F405/415, F407/417, F427/437, F429/439
        "F4": 256,  # bytes
        # ST RM0385 section 41.2 Unique device ID register
        "F7": 256,  # bytes
        # ST RM0394 47.1 Unique device ID register (96 bits)
        "L4": 256,  # bytes
        # ST RM0451 25.2 Unique device ID register (96 bits)
        "L0": 128,  # bytes
        # ST RM0444 section 38.1 Unique device ID register
        "G0": 256,  # bytes
    }

    FLASH_PAGE_SIZE = {
        "default": 1024,
        # ST RM0360 section 27.1 Memory size data register
        # F030x4/x6/x8/xC, F070x6/xB
        "F0": 1024,  # bytes
        # ST RM0008 section 30.2 Memory size registers
        # F101, F102, F103, F105, F107
        "F1": 1024,  # bytes
        # ST RM0366 section 29.2 Memory size data register
        # ST RM0365 section 34.2 Memory size data register
        # ST RM0316 section 34.2 Memory size data register
        # ST RM0313 section 32.2 Flash memory size data register
        # F303/328/358/398, F301/318, F302, F37x
        "F3": 2048,  # bytes
        # ST RM0090 section 39.2 Flash size
        # F405/415, F407/417, F427/437, F429/439
        "F4": 1024,  # bytes
        # ST RM0385 section 41.2 Flash size
        "F7": 1024,  # bytes
        # ST RM0394
        "L4": 1024,  # bytes
        # ST RM4510 25.1 Memory size register
        "L0": 128,  # bytes
        # ST RM0444 section 38.2 Flash memory size data register
        "G0": 1024,  # bytes
    }

    CHIP_IDS = {
    # see ST AN2606 Table 136 Bootloader device-dependent parameters
    # 16 to 32 KiB
    0x412: "STM32F10x Low-density",
    0x444: "STM32F03xx4/6",
    # 64 to 128 KiB
    0x410: "STM32F10x Medium-density",
    0x420: "STM32F10x Medium-density value line",
    0x460: "STM32G0x1",
    # 256 to 512 KiB (5128 Kbyte is probably a typo?)
    0x414: "STM32F10x High-density",
    0x428: "STM32F10x High-density value line",
    # 768 to 1024 KiB
    0x430: "STM3210xx XL-density",
    # flash size to be looked up
    0x417: "STM32L05xxx/06xxx",
    0x416: "STM32L1xxx6(8/B) Medium-density ultralow power line",
    0x411: "STM32F2xxx",
    0x433: "STM32F4xxD/E",
    # STM32F3
    0x432: "STM32F373xx/378xx",
    0x422: "STM32F302xB(C)/303xB(C)/358xx",
    0x439: "STM32F301xx/302x4(6/8)/318xx",
    0x438: "STM32F303x4(6/8)/334xx/328xx",
    0x446: "STM32F302xD(E)/303xD(E)/398xx",
    # RM0090 in ( 38.6.1 MCU device ID code )
    0x413: "STM32F405xx/07xx and STM32F415xx/17xx",
    0x419: "STM32F42xxx and STM32F43xxx",
    0x449: "STM32F74xxx/75xxx",
    0x450: "STM32H76xxx/77xxx",
    0x451: "STM32F76xxx/77xxx",
    # RM0394 46.6.1 MCU device ID code
    0x435: "STM32L4xx",
    # see ST AN4872
    # requires parity None
    0x11103: "BlueNRG",
    # STM32F0 RM0091 Table 136. DEV_ID and REV_ID field values
    0x440: "STM32F030x8",
    0x445: "STM32F070x6",
    0x448: "STM32F070xB",
    0x442: "STM32F030xC",
    # Cortex-M0 MCU with hardware TCP/IP and MAC
    # (SweetPeas custom bootloader)
    0x801: "Wiznet W7500",
    }

    SYNCHRONIZE_ATTEMPTS = 2
    def __init__(self) -> None:
         self._timeout = 5
         self.serial_port = '/dev/ttyS0'
         self.baud_rate = 115200,
         self.parity = 'none',

    def connect(self):
        self.serial_connection = SerialConnection();

        try:
            self.serial_connection.connect()
        except IOError as e:
            print(str(e) + "\n")
            sys.exit(1)
        
        try:
            print("Activating bootloader (select UART)")
            self.reset_from_system_memory()
        except Exception as e:
            print(str(e))
            print(
                "Can't init into bootloader. Ensure that BOOT0 is enabled and reset the device.",
                file=sys.stderr,
            )
            self.reset_from_flash()
            sys.exit(1)

    def _enable_boot0(self, enable=True):
        """Enable or disable the boot0 IO line (if possible)."""
        if not hasattr(self.serial_connection, "enable_boot0"):
            return

        self.serial_connection.enable_boot0(enable)

    def reset_from_flash(self):
        """Reset the MCU with boot0 disabled."""
        self._enable_boot0(False)
        self._reset()

    def _reset(self):
        """Enable or disable the reset IO line (if possible)."""
        if not hasattr(self.serial_connection, "enable_reset"):
            return
        self.serial_connection.enable_reset(True)
        time.sleep(0.1)
        self.serial_connection.enable_reset(False)
        time.sleep(0.5)

    def reset_from_system_memory(self):
        """Reset the MCU with boot0 enabled to enter the bootloader."""
        self._enable_boot0(True)
        self._reset()

        # Flush the input buffer to avoid reading old data.
        # It's known that the CP2102N at high baudrate fails to flush
        # its buffer when the port is opened.
        if hasattr(self.serial_connection, "flush_input_buffer"):
            self.serial_connection.flush_input_buffer()

        # Try the 0x7F synchronize that selects UART in bootloader mode
        # (see ST application notes AN3155 and AN2606).
        # If we are right after reset, it returns ACK, otherwise first
        # time nothing, then NACK.
        # This is not documented in STM32 docs fully, but ST official
        # tools use the same algorithm.
        # This is likely an artifact/side effect of each command being
        # 2-bytes and having xor of bytes equal to 0xFF.

        for attempt in range(self.SYNCHRONIZE_ATTEMPTS):
            if attempt:
                print("Bootloader activation timeout -- retrying", file=sys.stderr)
            self.write(self.Command.SYNCHRONIZE)
            read_data = bytearray(self.serial_connection.read())

            if read_data and read_data[0] in (self.Reply.ACK, self.Reply.NACK):
                # success
                return

        # not successful
        raise Exception("Bad reply from bootloader")
    
    def debug(self, level, message):
        """Print the given message if its level is low enough."""
        #if self.verbosity >= level:
        print(message, file=sys.stderr)

    def get(self):
        """Return the bootloader version and remember supported commands."""
        self.command(self.Command.GET, "Get")
        length = bytearray(self.serial_connection.read())[0]
        version = bytearray(self.serial_connection.read())[0]
        self.debug(10, "    Bootloader version: " + hex(version))
        data = bytearray(self.serial_connection.read(length))
        if self.Command.EXTENDED_ERASE in data:
            self.extended_erase = True
        self.debug(10, "    Available commands: " + ", ".join(hex(b) for b in data))
        self._wait_for_ack("0x00 end")
        return version

    def get_id(self):
        """Send the 'Get ID' command and return the device (model) ID."""
        self.command(self.Command.GET_ID, "Get ID")
        length = bytearray(self.serial_connection.read())[0]
        id_data = bytearray(self.serial_connection.read(length + 1))
        self._wait_for_ack("0x02 end")
        _device_id = reduce(lambda x, y: x * 0x100 + y, id_data)
        return _device_id


    def write(self, *data):
        """Write the given data to the MCU."""
        for data_bytes in data:
            if isinstance(data_bytes, int):
                data_bytes = struct.pack("B", data_bytes)
            self.serial_connection.write(data_bytes)
        
    def read_device_id(self):
        """Show chip ID and bootloader version."""
        boot_version = self.get()
        self.debug(0, "Bootloader version: 0x%X" % boot_version)
        device_id = self.get_id()
        self.debug(
            0, "Chip id: 0x%X (%s)" % (device_id, self.CHIP_IDS.get(device_id, "Unknown"))
        )
    
    def _wait_for_ack(self, info=""):
        """Read a byte and raise CommandError if it's not ACK."""
        read_data = bytearray(self.serial_connection.read())
        print('Wait For Ack. Read:', read_data)

        if not read_data:
            raise Exception("Can't read port or timeout")
        reply = read_data[0]
        if reply == self.Reply.NACK:
            raise Exception("NACK " + info)
        if reply != self.Reply.ACK:
            raise Exception("Unknown response. " + info + ": " + hex(reply))

        return 1

    def write_and_ack(self, message, *data):
        """Write data to the MCU and wait until it replies with ACK."""
        # Note: this is a separate method from write() because a keyword
        # argument after *args was not possible in Python 2
        print('Data to write:', data)
        self.write(*data)
        return self._wait_for_ack(message)
        
    def command(self, command, description):
        """
        Send the given command to the MCU.
        Raise CommandError if there's no ACK replied.
        """
        self.debug(10, "*** Command: %s" % description)
        ack_received = self.write_and_ack("Command", command, command ^ 0xFF)
        if not ack_received:
            raise Exception("%s (%s) failed: no ack" % (description, command))

    def reset(self):
        """Reset the microcontroller."""
        self.reset_from_flash()

def main():
    """
    Parse arguments and execute tasks.
    Default usage is to supply *sys.argv[1:].
    """
    try:
        loader = Stm32Loader()
        loader.connect()
        try:
            loader.read_device_id()
            #loader.read_device_uid()
            #loader.perform_commands()
        finally:
            loader.reset()
    except SystemExit:
        print('Ocurrio un error')

if __name__ == "__main__":
    main()
