import argparse
from led_mem_utils import *

parser = argparse.ArgumentParser()
parser.add_argument("pattern_file", help="A text file which will contain the new pattern")
args = parser.parse_args()

def sweep():
    '''Generates a pattern where each light fades up gradually one after another.'''
    sweep_entries = []
    # For each led on the board,
    for sweep_led in range(LEDS_PER_BOARD):
        # for each brightness level,
        #  (half the levels because it is too slow to iterate through each one)
        for led_val in range((MemoryEntry.FULL_BRIGHTNESS+1)//2):
            # generate a memory entry.
            m = MemoryEntry()
            # Each memory entry contains values for all LEDs.
            for led_idx in range(LEDS_PER_BOARD):
                # Each LED entry is off unless it is the sweep_led.
                if led_idx == sweep_led:
                    m.add_led(led_val*2)
                else:
                    m.add_led(MemoryEntry.MIN_BRIGHTNESS)
            sweep_entries.append(m.get_entry())

    return sweep_entries

def waterfall():
    '''Generates a pattern where lights fade on from top to bottom.'''
    waterfall_entries = []
    # First fade all the red colors, then the green ones.
    for color in [levels_red, levels_green]:
        previous_levels = []
        # Fade all the LEDs in each level on at the same time.
        for level in color:
            # Fade from off to fully on.
            for intensity in range(MemoryEntry.MIN_BRIGHTNESS, MemoryEntry.FULL_BRIGHTNESS+1):
                # Each memory entry describes all LEDs, so loop through all.
                m = MemoryEntry()
                for led in range(LEDS_PER_BOARD):
                    # Keep track of previous level to determine if other levels
                    # are on or off.
                    if led in level:
                        m.add_led(intensity)
                    elif led in previous_levels:
                        m.add_led(MemoryEntry.FULL_BRIGHTNESS)
                    else:
                        m.add_led(MemoryEntry.MIN_BRIGHTNESS)

                waterfall_entries.append(m.get_entry())
            previous_levels += level

        # Turn all on for a bit at the end of the fade before switching to
        # something to allow user to enjoy LEDs.
        for intensity in range(8):
            waterfall_entries.append(m.get_entry())

    return waterfall_entries

print('Genering waterfall pattern.')
entries = waterfall()
print('\tDone. Used %d memory entries out of 256.' % (len(entries)))
print('Genering sweep pattern.')
entries += sweep()
print('\tDone. Used %d memory entries out of 256' % (len(entries)))
with open(args.pattern_file, 'w') as f:
    for e in entries:
        f.write(e + '\n')