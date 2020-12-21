# xmas
# xmascard
Merry Christmas!

Welcome to the Electronic Christmas Tree Christmas Card. As you no doubt figured out already, to get started just push the button and enjoy the light show.

Additionally, this board is intended to be at least somewhat more than just an amusement; you can do things with it, and the entire schematic and program are open source on GitHub for you to see.

It’s powered by a Lattice FPGA, and you can add new patterns by plugging the USB connector in to the computer.

Unfortunately, I haven't gotten a chance to document this yet; I'll try to do so by Christmas itself. In the meantime, you can poke at the source code and try the instructions below, but I'm not sure they are actually correct. :(

## Interacting with the board

Double click the `light_control.py` file on your computer. It should open a window. Click **Connect**, and follow the instructions. It will ask you to plug in the Christmas Tree board, and if all goes well, it will print that it was successfully connected in the bottom of the window.

Now that you're connected, click any of the lights on the tree. They should change on the board, too! :)

## Creating your own patterns
What else can you do? Well, you can make your own patterns, using the `convert_animation_file.py` and `upload_new_pattern.py` programs.

These programs accept text files containing animations patterns and downloads them to the board for display.

The animation files work like so:

Each step in the animation is specified on a line. Each line specifies the brightness of the LEDs, from the top left to the bottom right. Each LED is two digits from 0 to 7 for the red, then green brightness. The value 0 is off, while 7 is all the way bright. For readability, spaces can be added as desired and are ignored.

For example, to light the red LED on the top, then the entire row of green on the bottom, the line would look like the following:

```
70  00  00 00  00 00 00  07 07 07 07  00
```

Since that can still be hard to read, you can alternately put the LEDs on different lines and separate each step in the animation with a blank line.

This allows you to write the pattern out visually. For example, the above example could also be written as:

```
            70
            00
          00  00
        00  00  00
      07  07  07  07
            00
```

The animation runs at a fixed speed of # frames per second, and the board stores 256 animation steps for a total of # seconds of animation. If you don't use all 256 frames of animation, the program will just repeat the last frame until memory is full.

In order to make it a little easier to create animations, you can tell the program to fade between animation steps automatically.
For example, to create an animation which fades between all the LEDs on, off again, then back on, create a text file like so:

```
set_marker
       77
       77
     77  77
   77  77  77
 77  77  77  77
       77
       77
fade_to
       00
       00
     00  00
   00  00  00
 00  00  00  00
       00
fade_to
       77
       77
     77  77
   77  77  77
 77  77  77  77
       77
       77
repeat_forever
```

Suppose your text file is named animation.txt. First, convert it to an FPGA memory entry like so:
```
convert_animation_file.py animation.txt memory.txt
```
Next, program it to the FPGA like so:
```
upload_new_pattern.py COM3 memory.txt
```
You can find the correct value to use for COM3 by running the `light_control.py` and looking at the value in the box at the bottom of the program after clicking **Connect**.
If everything went well, you should see your animation running on the FPGA. If not, well, you can either treat this as an exercise in learning about Python, or email me and I'll help. :)
