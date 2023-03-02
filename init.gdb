target extended-remote localhost:3333
monitor reset halt
load

echo Note: attached GDB\n
set breakpoint pending on
set confirm off

set $counter = 0

del 1 2 3 4 5 6 
info break

break blinky.c:23 
commands
echo HFCLKSTAT:
p /t NRF_CLOCK->HFCLKSTAT
echo \n
continue
end

break blinky.c:28 if $counter < 3
commands
p /t NRF_GPIO->OUTSET & (1UL << LED1)
p /t NRF_GPIO->OUTCLR & (1UL << LED1)
p /t NRF_GPIO->OUTSET & (1UL << LED2)
p /t NRF_GPIO->OUTCLR & (1UL << LED2)
set $counter = $counter + 1 
continue
end

break blinky.c:34 if $counter < 3
commands
p /t NRF_GPIO->OUTSET & (1UL << LED1)
p /t NRF_GPIO->OUTCLR & (1UL << LED1)
p /t NRF_GPIO->OUTSET & (1UL << LED2)
p /t NRF_GPIO->OUTCLR & (1UL << LED2)
set $counter = $counter + 1 
continue
end

continue
