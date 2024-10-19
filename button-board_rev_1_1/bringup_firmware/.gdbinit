target extended-remote :3333
set remote hardware-watchpoint-limit 2
set listsize 20
mon reset halt
maintenance flush register-cache
