LDZ #0x342A
LDAA #0x00E3
LDP #0x00E3
LDAA #0x00BB
STORA 0xAADD
LDAA #0x00BB
STORA 0xAADE
LODS #0xAA54
JR J506
STORA 0x01FA
JR J507
STORA 0x01FB
JR J508
STORA 0x01FC
JR J509
STORA 0x01FD
JR J510
STORA 0x01FE
JR J511
STORA 0x01FF
JR J512
STORA 0x0200
JR J513
STORA 0x0201
JR J514
STORA 0x0202
JR J515
STORA 0x0203
JR J516
STORA 0x0204
JR J517
STORA 0x0205
WAI 
J506:
LDAA #0x00E9
MVR C,#0x0094
CMP A,C
RTS
LDAA #0x00F7
RTS
WAI 
J507:
LDAA #0x006C
MVR C,#0x005A
CMP A,C
RTS
LDAA #0x0015
RTS
WAI 
J508:
LDAA #0x0020
MVR C,#0x007E
CMP A,C
RTS
LDAA #0x007F
RTS
WAI 
J509:
LDAA #0x00FD
MVR C,#0x0083
CMP A,C
RTS
LDAA #0x0051
RTS
WAI 
J510:
LDAA #0x0046
MVR C,#0x000C
CMP A,C
RTS
LDAA #0x0070
RTS
WAI 
J511:
LDAA #0x00C0
MVR C,#0x00AB
CMP A,C
RTS
LDAA #0x006C
RTS
WAI 
J512:
LDAA #0x009B
MVR C,#0x001A
CMP A,C
RTS
LDAA #0x009F
RTS
WAI 
J513:
LDAA #0x0002
MVR C,#0x0016
CMP A,C
RTS
LDAA #0x008B
RTS
WAI 
J514:
LDAA #0x00B8
MVR C,#0x0091
CMP A,C
RTS
LDAA #0x002B
RTS
WAI 
J515:
LDAA #0x0039
MVR C,#0x009E
CMP A,C
RTS
LDAA #0x0031
RTS
WAI 
J516:
LDAA #0x002C
MVR C,#0x0010
CMP A,C
RTS
LDAA #0x0093
RTS
WAI 
J517:
LDAA #0x00B6
MVR C,#0x0075
CMP A,C
RTS
LDAA #0x0093
RTS
WAI 
