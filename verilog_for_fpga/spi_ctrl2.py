import argparse
import sys
import serial
import struct
import random
import time

parser = argparse.ArgumentParser()
parser.add_argument("com_port", help="The com port of the FPGA (ex: 'COM3:')")
parser.add_argument("prom_file", help="The BIN prom file to program")
args = parser.parse_args()
ser = serial.Serial(args.com_port, 115200, timeout=2)

def write_byte(num):
    byte = struct.pack('B', int(num))
    ser.write(byte)
    #ser.flush()

def lower_cs():
    ser.write(b'l')

def raise_cs():
    ser.write(b'r')

def readwrite_spi_byte(to):
    ser.write(b'b')
    write_byte(to)
    b = ser.read()
    if len(b) != 1:
        # Something unexpected happened. Try to raise chip select to place SPI
        # flash in known state.
        raise_cs()
        raise_cs()
        raise_cs()
        raise_cs()
        raise Exception("Timeout reading data from COM port.")

    return b

def transact(cmd, bytes_to, num_bytes_back):
    lower_cs()
    readwrite_spi_byte(cmd)
    for b in bytes_to:
        readwrite_spi_byte(b)
    bytes_back = []
    for i in range(num_bytes_back):
        bytes_back.append(readwrite_spi_byte(0))
    raise_cs()
    
    return b''.join(bytes_back)

def validate_id():
    id = transact(0x9f, b'', 3)
    if id != bytes.fromhex('c22012'):
        raise Exception("ID from chip doesn't match. Expected 0xc22013 but got 0x%s." % (id.hex()))
        pass
    return id

def make_three_byte_address(addr):
    # Note the > in struct.pack because this address must be big endian as it is
    # sent to flash LSB first.
    addr_bytes = struct.pack('>I', addr)
    if (addr_bytes[0] != 0):
        raise Exception("Invalid address: 0x%x." % (addr))
    return addr_bytes[1:]
    
def read_page(addr):
    data = transact(0x03, make_three_byte_address(addr), 256)
    #print(repr(data))
    return data

def read_status(write_enable_set, write_in_progress_set = False, raise_if_not = True):
    status = transact(0x05, b'', 1)
    status = struct.unpack('B', status)[0]
    #print('Read status of 0x%x' % (status))
    write_enable_status = status & 2 != 0
    write_in_progress_status = status & 1 != 0
    if write_enable_set != write_enable_status or write_in_progress_set != write_in_progress_status:
        if raise_if_not:
            raise Exception("Status byte not as expected: 0x%x. write_enable_set:%s write_in_progress_set:%s" % (status, write_enable_set, write_in_progress_set))
        return False
    return True

def assert_page_is_blank(addr):
    d = read_page(addr)
    blank = d == b'\xff'*256
    if not blank:
        print(repr(d))
        raise Exception("Page 0x%x isn't blank." % (addr))

def enable_write():
    transact(0x06, b'', 0)
    read_status(write_enable_set = True)

def disable_write():
    transact(0x04, b'', 0)
    status = read_status(write_enable_set = False)

def wait_for_write_complete():
    status_reads = 0
    while read_status(write_enable_set = True, write_in_progress_set = True, raise_if_not = False) == True:
        #print('wait_for_write_complete ...')
        status_reads += 1

    read_status(write_enable_set = False)
    #print('Write complete after %d status reads.' % (status_reads))
    
def erase_sector(addr):
    # TODO: Check if sector is blank first

    enable_write()

    transact(0x20, make_three_byte_address(addr), 0)

    wait_for_write_complete()

def read_page_fast(addr):
    toser = b'g' + struct.pack('B', 0x03) + make_three_byte_address(addr)
    ser.write(toser)
    b = ser.read(256)
    #print(len(b))
    assert len(b) == 256
    return b
    
def program_page(addr, data):
    #assert_page_is_blank(addr)

    enable_write()

    #print(len(make_three_byte_address(addr) + data))
    
    #transact(0x02, make_three_byte_address(addr) + data, 0)
    toser = b'f' + struct.pack('B', 0x02) + make_three_byte_address(addr) + data
    #print('will send this much data:')
    #print(len(toser))
    #print('and it is')
    #print(repr(toser))
    
    ser.write(toser)
    b = ser.read()
    #print(repr(b))
    assert b == b'a'

    b = ser.read()
    #print(repr(b))
    assert b == b'd'

    #wait_for_write_complete()

def test():
    random_page_of_data = b''
    for i in range(256):
        random_page_of_data += struct.pack('B', random.randint(0,255))
    
    print('Erasing sector')
    erase_sector(0x07F000)
    print('Done')
    
    print('Am going to program this:')
    print(repr(random_page_of_data))
    print(len(random_page_of_data))
    program_page(0x07F000, random_page_of_data)
    print('Done. Now reading it back.')
    data_back = read_page(0x07F000)
    print('Got back:')
    print(repr(data_back))
    print(random_page_of_data == data_back)

def put_fpga_into_reset():
    ser.write(b'q')
    b = ser.read()
    #print(repr(b))
    assert b == b'q'

def put_fpga_out_of_reset():
    ser.write(b'w')
    b = ser.read()
    assert b == b'w'

def wait_for_fpga_done():
    status_reads = 0
    b = b'\x00'

    while b == b'\x00':
        if status_reads > 40:
            raise Exception('FPGA not done after %s status reads.' % (status_reads))

        status_reads += 1
        ser.write(b'e')
        b = ser.read()

    assert b == b'\x01'
    print('FPGA done after %d status reads.' % (status_reads))

def release_spi_flash_from_power_down():
    ser.write(b't')
    b = ser.read()
    assert b == b't'

def put_avr_into_passthrough_mode():
    ser.write(b'z')
    b = ser.read()
    print(repr(b))
    # TODO Not sure why this doesn't work; probably need a dealy
    assert b == b'z'

def program_xilinx_bin_file(binfile):
    validate_data = True

    with open(binfile, 'rb') as f:
        data = f.read()

    pad_amt = 4096 - (len(data) % 4096)
    print('Input file contains %d bytes of data' % (len(data)))
    print('Padding with %d bytes' % (pad_amt))
    data += b'\xff' * pad_amt

    print('Will program %d pages' % (len(data)/256))
    print('Will program %d sectors' % (len(data)/4096))
    print('To address %x' % (len(data)))

    # regain control from FPGA in case in passthrough mode
    # (it probably isn't because opening the com port will reset the AVR)
#    print('Sending break')
#    ser.send_break()
#    ser.write(b'?')
    b = ser.read()
    print(repr(b))

    put_fpga_into_reset()
    release_spi_flash_from_power_down()

    validate_id()

    for address in range(0, len(data), 256):
        if (address % 4096) == 0:
            print('Erasing sector %x, programming and verifying' % (address))
            erase_sector(address)
            #print('Done')

        page_data = data[address:address+256]
        blank_page_data = b'\xff' * 256

        if page_data == blank_page_data:
            print('Skipping page addr %x because blank' % (address))
            continue

        #print('Programming page addr %x' % (address))
        program_page(address, page_data)
        #print('Done')

        if validate_data:
            #print('Reading page back')
            data_back  = read_page_fast(address)
            #print('Done')

            if data_back != page_data:
                print('ERROR: Data received back did not match.')
                print(repr(page_data))
                print(repr(data_back))

    put_fpga_out_of_reset()

    print('Done programming. Waiting for FPGA to be done.')
    wait_for_fpga_done()
    print('Programming complete.')

    # put serial port into passthrough mode so if it wants to, the FPGA can use it
    put_avr_into_passthrough_mode()

def test2(binfile):
    with open(binfile, 'rb') as f:
        data = f.read()
    print('Start')
    ser.write(data)
    print('End')

#while True:
#    print(repr(validate_id()))
program_xilinx_bin_file(args.prom_file)
#test2(args.prom_file)
#put_fpga_into_reset()
#release_spi_flash_from_power_down()
#validate_id()
#put_fpga_out_of_reset()
#wait_for_fpga_done()





























