# PMW3360DM-T2QU
Functional library driver for PMW3360DM-T2QU optical navigation sensor.

# Official Datasheet Outdated
[Datasheet](www.pixart.com/upload/PMS0058-PMW3360DM-T2QU-NNDS-R1.30-06042016_20160902201411.pdf) provided on the official [website](http://www.pixart.com/product_data.asp?product_id=159&productclassify_id=1&productclassify2_id=3&partnumber=PMW3360DM-T2QU) by manufacturer PixArt is outdated. This document is labeled as "Version 1.30 |6 April 2016".
An updated datasheet can be found on [Tindie.com](https://www.tindie.com/products/jkicklighter/pmw3360-motion-sensor/?pt=ac_prod_search) website, [here](https://d3s5r33r268y59.cloudfront.net/datasheets/9604/2017-05-07-18-19-11/PMS0058-PMW3360DM-T2QU-DS-R1.50-26092016._20161202173741.pdf). This file is labeled as "Version 1.50 | 26 Sep 2016" and includes more information like the chip's registers functionalitys detailed.

# Test
This code has been test on **Adafruit Feather M0 Express** board (SAMD21G18A microcontroller) and **Sparkfun Arduino Pro Micro 3.3v** board (ATmega32u4 microcontroller).

# Firmware
In order to make work this sensor, is needed to load a firmware provided by manufacturer (Pixart) every time is powered on. I don't have found any clue of this firmware on the official website, also, manufacturer doesn't respond to my emails. I used firmware I found on @mrjohnk [Github](https://github.com/mrjohnk/PMW3360DM-T2QU). Thank you very much friend.

Note that this firmware is not the same used on the ADNS-9800 sensor. PMW3360DM-T2QU's firmware weights about 4Kbytes and ADNS-9800's firmware can load 1.5Kbytes or 3Kbytes. Latest release of this firmware looks like to be 0x40.

 

