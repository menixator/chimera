LDZ #0x342A
LDAA #0x00E3
LDP #0x00E3
LDAA #0x00BB
STORA 0xAADD
LDAA #0x00BB
STORA 0xAADE
LDAA #0x0063
MVR C,#0x00E6
CMP A,C
LDAA #0x0000
BEQ J506
LDAA #0x0040
J506:
STORA 0x01FA
LDAA #0x001C
MVR C,#0x00AE
CMP A,C
BEQ J507
LDAA #0x00E8
J507:
STORA 0x01FB
LDAA #0x00C6
MVR C,#0x00DF
CMP A,C
LDAA #0x0000
BEQ J508
LDAA #0x005C
J508:
STORA 0x01FC
LDAA #0x007F
MVR C,#0x00C0
CMP A,C
BEQ J509
LDAA #0x00F9
J509:
STORA 0x01FD
LDAA #0x0072
MVR C,#0x00E7
CMP A,C
BEQ J510
LDAA #0x00B0
J510:
STORA 0x01FE
LDAA #0x00DB
MVR C,#0x0061
CMP A,C
BEQ J511
LDAA #0x0081
J511:
STORA 0x01FF
LDAA #0x005D
MVR C,#0x0097
CMP A,C
BEQ J512
LDAA #0x0067
J512:
STORA 0x0200
LDAA #0x0075
MVR C,#0x008A
CMP A,C
LDAA #0x0000
BEQ J513
LDAA #0x0077
J513:
STORA 0x0201
LDAA #0x0034
MVR C,#0x000A
CMP A,C
LDAA #0x0000
BEQ J514
LDAA #0x00FC
J514:
STORA 0x0202
LDAA #0x0058
MVR C,#0x0098
CMP A,C
BEQ J515
LDAA #0x0022
J515:
STORA 0x0203
LDAA #0x0077
MVR C,#0x00AF
CMP A,C
LDAA #0x0000
BEQ J516
LDAA #0x0051
J516:
STORA 0x0204
LDAA #0x0043
MVR C,#0x00C9
CMP A,C
BEQ J517
LDAA #0x0009
J517:
STORA 0x0205
WAI 
