Readme.txt

Introduction
------------
This contains a basic DHT22 example, that'll transmit temperature and humidity
data in a 'water meter' type packet. You will need to move the pulldown
resistor on the digital dev board, and put a suitable pullup resistor in it's
place.

Roughly every 80 seconds, the device will poll the sensor and transmit the 
result in the form of a water meter counter packet. In order to pair with a
Current Cost base unit, choose a channel on the base unit, and put it in 
pairing mode. Next hold down the button on the digital dev board until the LED
starts flashing, which is about 5 or 6 seconds. The digital dev board unit 
will stay in pairing mode for approximately 12 seconds.

Battery life should be pretty good. I have one sensor that has been running
for about a year on the same set of AA batteries.

See EnviR Output to understand how the sensor values are transmitted, and how
to intepret the output from the EnviR.

Wiring
------
See the image DHT22_example.png first. Some changes to the board are needed
for the sensor to operate properly. You need to remove R10 (a pulldown) from
the board. Then, you will need to place a 5K resistor (a pullup) between VCC
and SIG. You can take GND from where R10 used to live. VCC, SIG and GND all
need to be connected to the correct lines on the DHT22 sensor.


EEPROM use
----------
The code uses the first two memory locations in EEPROM to store the device
address.

Precompiled HEX
---------------
If all you need is a hex file, there is a precompiled .hex file 
(PIC16-CurrentCost-HygroTherm-Framework.X.production.hex) in the 
dist/default/production directory.

EnviR Output
------------
The output from the EnviR should be something like:
<msg><src>CC128-v1.29</src><dsb>00759</dsb><time>19:09:09</time><tmpr>20.7</tmpr><sensor>8</sensor><id>12345</id><type>4</type><imp>0989912344</imp><ipu>0001</ipu></msg>

The <imp> value is a combination of temperature and humidity. The first 16
bits of the value are the temperature in 0.1 of a degree C. The next 8 bits
are the humidity in %. The last 8 bits are a checksum. This checksum should
have the same value as the adding together the most significat three bytes
and ANDing with 0xFF.

Notes
-----
This was built using a 'framework' I'd written to make it easier to produce
code that could interface with Current Cost "C2" equipment. 

This codeis distributed in the hope that it will be useful, but WITHOUT ANY 
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
A PARTICULAR PURPOSE. 
