target extended-remote localhost:3333
monitor reset halt
load

echo Note: attached GDB\n
set breakpoint auto-hw off
set breakpoint pending on
set confirm off

set $counter = 0

del 1 2 3 4 5 6 7 8 9 10 11 12 13
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

#break clock.c:5
#commands
#
#set $compare_reg = (NRF_CLOCK->HFCLKSTAT & (1UL << CLOCK_HFCLKSTAT_SRC_Pos)) >> CLOCK_HFCLKSTAT_SRC_Pos
#set $compare_exp = CLOCK_HFCLKSTAT_SRC_RC
#set $compare_msg = "HFCLKSTAT src"
#compare
#
#set $compare_reg = (NRF_CLOCK->HFCLKSTAT & (1UL << CLOCK_HFCLKSTAT_STATE_Pos)) >> CLOCK_HFCLKSTAT_STATE_Pos
#set $compare_exp = CLOCK_HFCLKSTAT_STATE_Running
#set $compare_msg = "HFCLKSTAT state"
#compare
#
#set $compare_reg = (NRF_CLOCK->LFCLKSTAT & (1UL << CLOCK_LFCLKSTAT_STATE_Pos)) >> CLOCK_LFCLKSTAT_STATE_Pos
#set $compare_exp = CLOCK_LFCLKSTAT_STATE_NotRunning
#set $compare_msg = "LFCLKSTAT state"
#compare
#
#printf "Fcpu: %dMHz", (SystemCoreClock/1000000)
#
#continue
#end
#
#break clock.c:9
#commands
#set $compare_reg = (NRF_CLOCK->LFCLKSRC & (1UL << CLOCK_LFCLKSRC_SRC_Pos)) >> CLOCK_LFCLKSRC_SRC_Pos
#set $compare_exp = CLOCK_LFCLKSRC_SRC_RC
#set $compare_msg = "LFCLKSRC src"
#compare
#
#set $compare_reg = (NRF_CLOCK->LFCLKSRC & (1UL << CLOCK_LFCLKSRC_BYPASS_Pos)) >> CLOCK_LFCLKSRC_BYPASS_Pos
#set $compare_exp = CLOCK_LFCLKSRC_BYPASS_Disabled
#set $compare_msg = "LFCLKSRC bypass"
#compare
#
#set $compare_reg = (NRF_CLOCK->LFCLKSRC & (1UL << CLOCK_LFCLKSRC_EXTERNAL_Pos)) >> CLOCK_LFCLKSRC_EXTERNAL_Pos
#set $compare_exp = CLOCK_LFCLKSRC_EXTERNAL_Disabled
#set $compare_msg = "LFCLKSRC external"
#compare
#
#set $compare_reg = (NRF_CLOCK->LFCLKSTAT & (1UL << CLOCK_LFCLKSTAT_SRC_Pos)) >> CLOCK_LFCLKSTAT_SRC_Pos
#set $compare_exp = CLOCK_LFCLKSTAT_SRC_RC
#set $compare_msg = "LFCLKSTAT src"
#compare
#
#set $compare_reg = (NRF_CLOCK->LFCLKSTAT & (1UL << CLOCK_LFCLKSTAT_STATE_Pos)) >> CLOCK_LFCLKSTAT_STATE_Pos
#set $compare_exp = CLOCK_LFCLKSTAT_STATE_Running
#set $compare_msg = "LFCLKSTAT state"
#compare
#
#printf "Fcpu: %dMHz", (SystemCoreClock/1000000)
#quit
#end
#
#break temp.c:21
#commands
#printf "Current temperature: %d", NRF_TEMP->TEMP/4
#continue
#end
#
#break temp.c:24
#commands
#printf "Average temperature: %d", avg
#continue
#end
#
#break button_isr.c:134
#commands
#continue
#end

#hbreak button_isr.c:137
#commands
#continue
#end

#hbreak accel.c:119
#commands
#printf "GPIO->PIN_CNF[SDA]_DIR: %d\n", ((NRF_GPIO->PIN_CNF[PIN_SDA] >> GPIO_PIN_CNF_DIR_Pos) & 1UL)
#printf "GPIO->PIN_CNF[SDA]_INPUT: %d\n", ((NRF_GPIO->PIN_CNF[PIN_SDA] >> GPIO_PIN_CNF_INPUT_Pos) & 1UL)
#printf "GPIO->PIN_CNF[SDA]_DRIVE: %d\n", ((NRF_GPIO->PIN_CNF[PIN_SDA] >> GPIO_PIN_CNF_DRIVE_Pos) & 7UL)
#printf "GPIO->PIN_CNF[SCL]_DIR: %d\n", ((NRF_GPIO->PIN_CNF[PIN_SCL] >> GPIO_PIN_CNF_DIR_Pos) & 1UL)
#printf "GPIO->PIN_CNF[SCL]_INPUT: %d\n", ((NRF_GPIO->PIN_CNF[PIN_SCL] >> GPIO_PIN_CNF_INPUT_Pos) & 1UL)
#printf "GPIO->PIN_CNF[SCL]_DRIVE: %d\n", ((NRF_GPIO->PIN_CNF[PIN_SCL] >> GPIO_PIN_CNF_DRIVE_Pos) & 7UL)
#printf "TWIM0->ENABLE: 0x%x\n", NRF_TWIM0->ENABLE
#printf "TWIM0->FREQUENCY: 0x%x\n", NRF_TWIM0->FREQUENCY
#printf "TWIM0->SHORTS_LASTTX_STARTRX: %d\n", ((NRF_TWIM0->SHORTS >> TWIM_SHORTS_LASTTX_STARTRX_Pos) & 1UL)
#printf "TWIM0->SHORTS_LASTRX_STOP: %d\n", ((NRF_TWIM0->SHORTS >> TWIM_SHORTS_LASTRX_STOP_Pos) & 1UL)
#printf "TWIM0->PSEL.SCL_CONNECT: %d\n", ((NRF_TWIM0->PSEL.SCL >> TWIM_PSEL_SCL_CONNECT_Pos) & 1UL) 
#printf "TWIM0->PSEL.SDA_CONNECT: %d\n", ((NRF_TWIM0->PSEL.SDA >> TWIM_PSEL_SDA_CONNECT_Pos) & 1UL) 
#printf "TWIM0->PSEL.SCL_PIN: %x\n", NRF_TWIM0->PSEL.SCL >> TWIM_PSEL_SCL_PIN_Pos
#printf "TWIM0->PSEL.SDA_PIN: %x\n", NRF_TWIM0->PSEL.SDA >> TWIM_PSEL_SDA_PIN_Pos
#printf "TWIM0->ADDRESS: 0x%x\n", NRF_TWIM0->ADDRESS
#printf "TWIM0->TXD.PTR: 0x%x\n", NRF_TWIM0->TXD.PTR
#printf "TWIM0->TXD.MAXCNT: %d\n", NRF_TWIM0->TXD.MAXCNT
#printf "TWIM0->RXD.PTR: 0x%x\n", NRF_TWIM0->RXD.PTR
#printf "TWIM0->RXD.MAXCNT: %d\n", NRF_TWIM0->RXD.MAXCNT
#printf "TWIM0->INTENSET_STOPPED: %d\n", ((NRF_TWIM0->INTENSET >> TWIM_INTENSET_STOPPED_Pos) & 1UL)
#printf "TWIM0->INTENSET_ERROR: %d\n", ((NRF_TWIM0->INTENSET >> TWIM_INTENSET_ERROR_Pos) & 1UL)
#printf "TWIM0->INTENSET_SUSPENDED: %d\n", ((NRF_TWIM0->INTENSET >> TWIM_INTENSET_SUSPENDED_Pos) & 1UL)
#printf "TWIM0->INTENSET_RXSTARTED: %d\n", ((NRF_TWIM0->INTENSET >> TWIM_INTENSET_RXSTARTED_Pos) & 1UL)
#printf "TWIM0->INTENSET_TXSTARTED: %d\n", ((NRF_TWIM0->INTENSET >> TWIM_INTENSET_TXSTARTED_Pos) & 1UL)
#printf "TWIM0->INTENSET_LASTRX: %d\n", ((NRF_TWIM0->INTENSET >> TWIM_INTENSET_LASTRX_Pos) & 1UL)
#printf "TWIM0->INTENSET_LASTTX: %d\n", ((NRF_TWIM0->INTENSET >> TWIM_INTENSET_LASTTX_Pos) & 1UL)
#printf "TWIM0->EVENT_STOPPED: %d\n", NRF_TWIM0->EVENTS_STOPPED
#printf "TWIM0->EVENT_ERROR: %d\n", NRF_TWIM0->EVENTS_ERROR
#printf "TWIM0->EVENT_SUSPENDED: %d\n", NRF_TWIM0->EVENTS_SUSPENDED
#printf "TWIM0->EVENT_RXSTARTED: %d\n", NRF_TWIM0->EVENTS_RXSTARTED
#printf "TWIM0->EVENT_TXSTARTED: %d\n", NRF_TWIM0->EVENTS_TXSTARTED
#printf "TWIM0->EVENT_LASTRX: %d\n", NRF_TWIM0->EVENTS_LASTRX
#printf "TWIM0->EVENT_LASTTX: %d\n", NRF_TWIM0->EVENTS_LASTTX
#continue
#end #
#hbreak accel.c:151
#commands
#continue
#end


hbreak accel.c:393
commands
printf "%d,%d\n", lis2dh12_xy[0], lis2dh12_xy[1]
continue
end

hbreak accel.c:393
commands
set $idx = 0
printf "TWIM0->SX1509_RX_BUF: "
while ($idx < 16)
    printf "0x%x ", sx1509_read_buf[$idx]
    set $idx=$idx+1
end
printf "\n"
printf "TWIM0->RXD.AMOUNT: %d\n", NRF_TWIM0->RXD.AMOUNT
continue
end

continue
