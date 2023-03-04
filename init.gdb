target extended-remote localhost:3333
monitor reset halt
load

echo Note: attached GDB\n
set breakpoint pending on
set confirm off

set $counter = 0

del 1 2 3 4 5 6 
info break

set $compare_reg = 0
set $compare_exp = 0
set $compare_msg = ""

define compare
    p $compare_msg

    if $compare_reg == $compare_exp
        echo passed\n  
    else
        p $compare_reg 
        p $compare_exp
        echo error\n
        quit
    end
end 

break blinky.c:27
commands

set $compare_reg = (NRF_CLOCK->HFCLKSTAT & (1UL << CLOCK_HFCLKSTAT_SRC_Pos)) >> CLOCK_HFCLKSTAT_SRC_Pos
set $compare_exp = CLOCK_HFCLKSTAT_SRC_RC
set $compare_msg = "HFCLK src"
compare

set $compare_reg = (NRF_CLOCK->HFCLKSTAT & (1UL << CLOCK_HFCLKSTAT_STATE_Pos)) >> CLOCK_HFCLKSTAT_STATE_Pos
set $compare_exp = CLOCK_HFCLKSTAT_STATE_Running
set $compare_msg = "HFCLK state"
compare

continue
end

break blinky.c:31 if $counter < 10
commands
p /t (NRF_GPIO->OUTSET & (1UL << LED1)) >> LED1
p /t (NRF_GPIO->OUTSET & (1UL << LED2)) >> LED2
set $counter = $counter + 1 
continue
end

break blinky.c:37 if $counter < 10
commands
p /t (NRF_GPIO->OUTSET & (1UL << LED1)) >> LED1
p /t (NRF_GPIO->OUTSET & (1UL << LED2)) >> LED2
set $counter = $counter + 1 
continue
end

break clock.c:5
commands

set $compare_reg = (NRF_CLOCK->HFCLKSTAT & (1UL << CLOCK_HFCLKSTAT_SRC_Pos)) >> CLOCK_HFCLKSTAT_SRC_Pos
set $compare_exp = CLOCK_HFCLKSTAT_SRC_RC
set $compare_msg = "HFCLKSTAT src"
compare

set $compare_reg = (NRF_CLOCK->HFCLKSTAT & (1UL << CLOCK_HFCLKSTAT_STATE_Pos)) >> CLOCK_HFCLKSTAT_STATE_Pos
set $compare_exp = CLOCK_HFCLKSTAT_STATE_Running
set $compare_msg = "HFCLKSTAT state"
compare

set $compare_reg = (NRF_CLOCK->LFCLKSTAT & (1UL << CLOCK_LFCLKSTAT_STATE_Pos)) >> CLOCK_LFCLKSTAT_STATE_Pos
set $compare_exp = CLOCK_LFCLKSTAT_STATE_NotRunning
set $compare_msg = "LFCLKSTAT state"
compare

printf "Fcpu: %dMHz", (SystemCoreClock/1000000)

continue
end

break clock.c:9
commands
set $compare_reg = (NRF_CLOCK->LFCLKSRC & (1UL << CLOCK_LFCLKSRC_SRC_Pos)) >> CLOCK_LFCLKSRC_SRC_Pos
set $compare_exp = CLOCK_LFCLKSRC_SRC_RC
set $compare_msg = "LFCLKSRC src"
compare

set $compare_reg = (NRF_CLOCK->LFCLKSRC & (1UL << CLOCK_LFCLKSRC_BYPASS_Pos)) >> CLOCK_LFCLKSRC_BYPASS_Pos
set $compare_exp = CLOCK_LFCLKSRC_BYPASS_Disabled
set $compare_msg = "LFCLKSRC bypass"
compare

set $compare_reg = (NRF_CLOCK->LFCLKSRC & (1UL << CLOCK_LFCLKSRC_EXTERNAL_Pos)) >> CLOCK_LFCLKSRC_EXTERNAL_Pos
set $compare_exp = CLOCK_LFCLKSRC_EXTERNAL_Disabled
set $compare_msg = "LFCLKSRC external"
compare

set $compare_reg = (NRF_CLOCK->LFCLKSTAT & (1UL << CLOCK_LFCLKSTAT_SRC_Pos)) >> CLOCK_LFCLKSTAT_SRC_Pos
set $compare_exp = CLOCK_LFCLKSTAT_SRC_RC
set $compare_msg = "LFCLKSTAT src"
compare

set $compare_reg = (NRF_CLOCK->LFCLKSTAT & (1UL << CLOCK_LFCLKSTAT_STATE_Pos)) >> CLOCK_LFCLKSTAT_STATE_Pos
set $compare_exp = CLOCK_LFCLKSTAT_STATE_Running
set $compare_msg = "LFCLKSTAT state"
compare

printf "Fcpu: %dMHz", (SystemCoreClock/1000000)
quit
end

break temp.c:21
commands
printf "Current temperature: %d", NRF_TEMP->TEMP/4
continue
end

break temp.c:24
commands
printf "Average temperature: %d", avg
continue
end
continue
