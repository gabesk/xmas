LEDS_PER_BOARD = 24

def to_bin(val, bits=3):
    str = []
    for i in range(bits):
        if val & (1 << i):
            str.append('1')
        else:
            str.append('0')
    str.reverse()
    return ''.join(str)

def to_hex(str):
    intval = 0
    for i in range(len(str)-1, -1, -1):
        if str[i] == '1':
            intval |= (1 << (len(str)-i-1))
    return ('%018x' % (intval))

class MemoryEntry():
    FULL_BRIGHTNESS = 7
    MIN_BRIGHTNESS = 0
    def __init__(self):
        self.mem_pieces = []
        self.finalized = False

    def add_led(self, intensity):
        '''Call 24 times to add LEDs from 0 to 23 to array.
        Intensity from 0 to 7'''
        self.mem_pieces.append(to_bin(intensity))

    def get_entry(self):
        '''Call to print the line of memory.'''
        if not self.finalized:
            self.mem_pieces.reverse()
            self.mem_pieces = ''.join(self.mem_pieces)
            self.finalized = True

        return to_hex(self.mem_pieces)


#            *R
#            RG 
#          RG  RG
#        RG  GR  GR
#      GR  RG  GR  RG
#            RG
leds = [
    ['r','g'], # 0 1
    ['r','g'], # 2 3
    ['r','g'],['r','g'], # 4 5  6 7
    ['r','g'],['g','r'],['g','r'], # 8 9  10 11  12 13
    ['g','r'],['r','g'],['g','r'],['r','g'], # 14 15  16 17  18 19  20 21
    ['r','g'] # 22 23
]

levels_red = [
    [0],
    [2],
    [4,6],
    [8, 11, 13],
    [15, 16, 19, 20],
    [22]
]

levels_green = [
    [0],
    [3],
    [5,7],
    [9,10,12],
    [14,17,18,21],
    [23]
]
