target extended-remote localhost:3333
monitor reset halt
load

echo Note: attached GDB\n
set breakpoint pending on

break main
commands
echo Note: breakpoint hit\n
continue
end

continue
