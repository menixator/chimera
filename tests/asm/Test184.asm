LDAA #0x00BB
STORA 0xAADD
LDAA #0x00BB
STORA 0xAADE
LODS #0xAA54
LDAA #0x0008
MVR C,#0x0055
CMP A,C
CCC J506
LDAA #0x00CC
PUSH A
PUSH A
J506:
POP A
POP A
STORA 0x01FB
LDAA #0x00A3
MVR C,#0x009B
CMP A,C
CCC J508
LDAA #0x000A
PUSH A
PUSH A
J508:
POP A
POP A
STORA 0x01FD
LDAA #0x0093
MVR C,#0x00CB
CMP A,C
CCC J510
LDAA #0x0030
PUSH A
PUSH A
J510:
POP A
POP A
STORA 0x01FF
LDAA #0x004C
MVR C,#0x00AC
CMP A,C
CCC J512
LDAA #0x0088
PUSH A
PUSH A
J512:
POP A
POP A
STORA 0x0201
LDAA #0x00FE
MVR C,#0x0082
CMP A,C
CCC J514
LDAA #0x0047
PUSH A
PUSH A
J514:
POP A
POP A
STORA 0x0203
LDAA #0x00EE
MVR C,#0x007A
CMP A,C
CCC J516
LDAA #0x00A5
PUSH A
PUSH A
J516:
POP A
POP A
STORA 0x0205
WAI 
data506: dw J506
data508: dw J508
data510: dw J510
data512: dw J512
data514: dw J514
data516: dw J516
