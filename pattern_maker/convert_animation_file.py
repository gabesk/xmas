usage = '''
This program accepts an animation file in the format of a frame of the
animation per line.

Each line specifies the LEDs, from the top left to the bottom right.

Each LED is two digits from 0 to 7 for the red, then green brightness.
For readability, spaces can be added as desired and are ignored.

For example, to light the red LED on the top, then the entire row of green on
the bottom, the line would look like the following:

70  00  00 00  00 00 00  07 07 07 07  00

For additional readbility, you can also use a blank line as the separator
between frames in the animation.
This allows you to write the pattern out visually, at the cost of vertical
space, like so:

           70
           00 
         00  00
       00  00  00
     07  07  07  07
           00

This program converts the animation into a memory buffer ready to program into
the Xmas Tree Board. Use upload_new_pattern.py to do so.
'''

# How should the fade work? One simple algorithm is to look at the maximum
# difference in any single LED between A and B. That's the number of steps for
# interpolation. Then, for each frame, and with that interpolation constant,
# create a new calculate the new LED value.
# This only works if you want to be able to fade the value of a single LED,
# however. Suppose you specified frame A with the top LED on and frame B with
# the bottom row on, and you wanted to interpolate between them. What would the
# right behavior be? One approach would be to fade out the top and fade in the
# bottom. What if you wanted to do night-rider style where the transition lit up
# all the rows? Could you treat it like a fluid and flow the water from one to
# the next? How would you deal with the fact that you have a different quantity
# of water at the start than the end in terms of dealing with LED intensity?
# Sadly that seems like the sort of thing best left to the user at the moment.

import argparse
import sys
import struct
import pdb
import led_mem_utils
from led_mem_utils import LEDS_PER_BOARD
import itertools

parser = argparse.ArgumentParser()
parser.add_argument("input_pattern_file", help="The input pattern file to process.")
parser.add_argument("output_data", help="The raw output data ready to program.")
parser.add_argument('--verbose', dest='verbose', action='store_const',
                    const=True, default=False,
                    help="Pretty-print the output which will be written to memory.")
if len(sys.argv) == 1:
    print(usage)
    parser.print_help()
    sys.exit(0)

args = parser.parse_args()

# Read the contents of the pattern file into memory.
input_lines = []
with open(args.input_pattern_file, 'r') as f:
    for line in f:
        input_lines.append(line.strip())

# Go through each line. Remove all the spaces. If a given line
# does not contain enough digits to contain an entire line, assume that the next
# lines will fill out the line. If, however, the line contains no digits at all,
# this indicates the end of a frame of animation when the LEDs are spread across
# multiple lines, and all LEDs must be specified before that point. Therefore,
# indicate an error to the user.
animation_frames = []
current_frame = []
frame_start_line = 0
in_fade = False
fade_speed = 1
repeat_markers = {}
repeat_forever = False

def add_frame():
    global current_frame
    # Add current frame to animation and clear current frame state.
    animation_frames.append((frame_start_line,current_frame))
    current_frame = []
    if in_fade:
        fade()

def fade():
    global in_fade
    in_fade = False
    start = animation_frames[-2]
    end = animation_frames[-1]
    print('Fading frames {0} to {1}'.format(len(animation_frames) - 1, len(animation_frames)))
    frame_max_delta = 0
    # What's the greatest change between start and end frames?
    for idx, start_led in enumerate(start[1]):
        end_led = end[1][idx]
        led_delta = abs(end_led - start_led)
        if led_delta > frame_max_delta:
            frame_max_delta = led_delta
    # If there's not enough difference to fade, return.
    if frame_max_delta < 2:
        return
    # Slow down the fade by fade_speed, by increasing the frame_max_delta
    frame_max_delta *= fade_speed
    # We're going to add frames so temporarily remove the end frame from the 
    # collection.
    animation_frames.pop()
    # Calculate each frame of the fade.
    for step in range(frame_max_delta - 1):
        intermediate_frame = []
        for idx, start_led in enumerate(start[1]):
            start_led *= fade_speed
            end_led = end[1][idx] * fade_speed
            led_delta = end_led - start_led
            derating_factor = led_delta / frame_max_delta
            transition_led = (start_led + derating_factor * (step + 1)) / fade_speed
            intermediate_frame.append(int(transition_led))
        animation_frames.append((start[0], intermediate_frame))
    animation_frames.append(end)

for line_idx, line in enumerate(input_lines):
    if line.strip() != '' and repeat_forever:
        print('Line {0} has LED information, but repeat_forever was specified, '
              'which must be the last line in the file.'.format(line_idx + 1))
        sys.exit(1)
    if line.strip() == '':
        # If this is an empty line, ensure sufficient LEDs accumulated.
        if not len(current_frame) in (0,):
            print('Line {0} is blank, indicating the start of a new animation '
                  'frame. However, not enough LEDs were found in the previous '
                  'lines to form a complete frame. Only {1} LEDs were found '
                  'between line {2} and this one. '
                  'There needs to be exactly 24.'.format(
                   line_idx + 1, len(current_frame), frame_start_line + 1))
            sys.exit(1)
        if len(current_frame) == LEDS_PER_BOARD:
            add_frame()
    elif line.strip().startswith('fade_to'):
        # Fade commands must come between two complete frames.
        if not animation_frames or current_frame:
            print('Line {0} specified a fade, but did not come between '
                  'two complete frames (see if the previous frame had too few '
                  'or too many LED entries).'.format(line_idx + 1))
            sys.exit(1)
        fade_speed = 1
        in_fade = True
        if ':' in line:
            fade_speed = int(line.split(':')[1].strip())
    elif line.strip().startswith('set_marker'):
        # Set a marker so a group of frames can be repeated.
        # Marker commands must not be in the middle of a frame.
        if current_frame:
            print('Line {0} specified a marker, but is in the middle of a '
                  'frame (see if the previous frame had too few '
                  'or too many LED entries).'.format(line_idx + 1))
            sys.exit(1)
        marker_name = 'default'
        if ':' in line:
            marker_name = line.split(':')[1].strip()
        if animation_frames:
            repeat_markers[marker_name] = len(animation_frames) - 1
        else:
            repeat_markers[marker_name] = 0
        print('Setting marker {0} to line {1}.'.format(
              marker_name, repeat_markers[marker_name]))
    elif line.startswith('repeat') or line.startswith('repeat_forever'):
        # Repeat everything from here to the marker.
        # Repeat commands must not be in the middle of a frame.
        if current_frame:
            print('Line {0} specified a repeat, but is in the middle of a '
                  'frame (see if the previous frame had too few '
                  'or too many LED entries).'.format(line_idx + 1))
            sys.exit(1)

        marker_name = 'default'
        if ':' in line:
            marker_name = line.split(':')[1].strip()
        # There must have been a marker to repeat.
        if not repeat_markers or not marker_name in repeat_markers:
            print('Line {0} specified a repeat, '
                  'but there is no marker set.'.format(line_idx + 1))
            sys.exit(1)
        repeat_marker = repeat_markers[marker_name]
        so_far = len(animation_frames)
        # repeat_forever must be the last entry in the file
        if line.startswith('repeat_forever'):
            repeat_forever = True
            space_left = 256 - so_far
            repetition_length = so_far - repeat_marker + 1
            repetitions = space_left // repetition_length
            for r in range(repetitions):
                print('Repeating frame {0} to {1}'.format(repeat_marker, so_far+1))
                animation_frames += animation_frames[repeat_marker:so_far+1]
        else:
            print('Repeating frame {0} (marker {1}) to {2}'.format(repeat_marker, marker_name, so_far+1))
            animation_frames += animation_frames[repeat_marker:so_far+1]
    else:
        # Add LEDs from this line to the frame.
        for char_idx, char in enumerate(line):
            # Skip if a space character
            if char == ' ':
                continue
            if not current_frame:
                frame_start_line = line_idx
            # Check that the current frame is not full.
            if len(current_frame) == LEDS_PER_BOARD:
                print('On line {0}, at character {1}, there are too many LEDs '
                      'for the current animation frame. There should only be '
                      'exactly 24.'.format(line_idx, char_idx))
                sys.exit(1)
            # Check that the symbol is a digit in the valid brightness range.
            if not char.isdigit() or int(char) < 0 or int(char) > 7:
                print('On line {0}, at character {1}, invalid brightness '
                      'value for LED. It is {2} but should be a number '
                      'between 0 and 7.'.format(line_idx, char_idx, repr(char)))
                sys.exit(1)
            # Checks passed; add to animation frame.
            current_frame.append(int(char))
        # If frame is complete, add to animation and clear current frame state.
        if len(current_frame) == LEDS_PER_BOARD:
            add_frame()

# Check that something was read.
if not animation_frames:
    print('No animation frames read from input file!')
    sys.exit(1)

# Check that the final line was complete.
if current_frame:
    print('End of file at line {0}. However, not enough LEDs were found to '
          'form a complete frame. Only {1} LEDs were found between line {2} '
          'and this one. There needs to be exactly 24.'.format(
           line_idx + 1, len(current_frame), frame_start_line + 1))
    sys.exit(1)

print('Successfully created {0} animation frames '
      'from input file or commands.'.format(len(animation_frames)))

def pretty_print_frame(frame):
    '''Prints an animation frame to stdout in multi-line format as in help.'''
    frame = list(frame[1])
    longest = (4 + 3) * 2
    leds_lines = (1, 1, 2, 3, 4, 1)
    for leds_line in leds_lines:
        if frame == 'in_fade':
            print('in_fade')
            continue
        line_str = ''
        for led in range(leds_line):
            line_str += str(frame.pop(0))
            line_str += str(frame.pop(0))
            line_str += '  '
        line_str = line_str[:-2]
        padding_amt = (longest - len(line_str)) // 2
        line_str = ' ' * padding_amt + line_str
        print(line_str)
    print()

# Convert each frame to format accepted by christmas tree board.
output_lines = []
for idx, frame in enumerate(animation_frames):
    if args.verbose:
        pretty_print_frame(frame)
    if frame == 'in_fade':
        continue
    m = led_mem_utils.MemoryEntry()
    for led in frame[1]:
        m.add_led(led)
    output_lines.append(m.get_entry())

# Check that there weren't too many frames to fit in memory.
if len(output_lines) > 256:
    print('{0} frames in animation but only 256 fit in memory. '
          'Last {1} frames are thrown out.'.format(len(output_lines),
                                                   len(output_lines) - 256))
    output_lines = output_lines[:256]

# If there were not 256 frames in the input animation, repeat the last frame
# until there are.
if len(output_lines) < 256:
    print('Repeating last frame {0} more times '
          'to make 256 frames in animation.'.format(256 - len(output_lines)))

for i in range(256 - len(output_lines)):
    output_lines.append(output_lines[-1])

# Write them out to the output file.
with open(args.output_data, 'w') as f:
    f.writelines(itertools.chain.from_iterable(zip(output_lines, ['\n'] * len(output_lines))))

print('Successfully converted animation and wrote output to {0}.'.format(
      args.output_data))
