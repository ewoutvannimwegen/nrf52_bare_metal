
This file summarices which problems I've encountered while building this driver, some
of them took hours to solve and might save the reader a lot of time.

## Write data 
Pay attention to how the I2C slave wants to receive data. 
The SX1509B only accepts the following seq (see datasheet):

addr, data byte 0, data byte 1, data byte 2 ... data byte n - 1 data byte n

These data bytes all go to the same address! So it is not possible to 
e.g. burst send 32 bytes to different registers (using the current setup).

## Power of SX1509B (classic :O)

The SX1509B has his own power pin on the Thingy52 that must be set high before 
the SX1509B even turns on! 

## SHORTS

The short register takes a while to toggle, so I've implemented a small delay
to make sure it is toggled when starting I2C.

