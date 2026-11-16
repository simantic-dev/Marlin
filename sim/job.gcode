; heat, home, print a 40 mm square, cool down. Sent one line at a time, next line after the firmware's "ok".
M115              ; capabilities: the firmware names itself
M155 S1           ; temperature report every second
M104 S190         ; start heating, do not wait
G28               ; home X, Y, Z against the scripted endstops
M114
G90
M83
M109 S190         ; wait for 190 C
G1 Z0.3 F600
G1 X30 Y30 F6000  ; travel to the corner
G1 X70 Y30 E1.6 F3000
G1 X70 Y70 E1.6
G1 X30 Y70 E1.6
G1 X30 Y30 E1.6
G1 X70 Y70 F3000  ; diagonal
M400
M114
M104 S0
