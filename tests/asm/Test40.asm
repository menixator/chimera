LDZ #0x342A
LDAA #0x00E3
LDP #0x00E3
LDAA #0x00BB
STORA 0xAADD
LDAA #0x00BB
STORA 0xAADE
MVR E,#0x0042
LDAA #0x0059
CMP A,E
STORA 0x01FA
MVR E,#0x0086
LDAA #0x0086
CMP A,E
STORA 0x01FB
MVR E,#0x00A0
LDAA #0x00F8
CMP A,E
STORA 0x01FC
MVR E,#0x0045
LDAA #0x0097
CMP A,E
STORA 0x01FD
MVR E,#0x00DF
LDAA #0x0056
CMP A,E
STORA 0x01FE
MVR E,#0x0065
LDAA #0x0054
CMP A,E
STORA 0x01FF
MVR E,#0x0023
LDAA #0x00F2
CMP A,E
TSA
STORA 0x0200
MVR E,#0x0020
LDAA #0x003B
CMP A,E
TSA
STORA 0x0201
MVR E,#0x005C
LDAA #0x00CE
CMP A,E
TSA
STORA 0x0202
MVR E,#0x0042
LDAA #0x00C2
CMP A,E
TSA
STORA 0x0203
MVR E,#0x00CC
LDAA #0x0004
CMP A,E
TSA
STORA 0x0204
MVR E,#0x0001
LDAA #0x000A
CMP A,E
TSA
STORA 0x0205
WAI 
