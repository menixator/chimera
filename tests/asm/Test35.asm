LDZ #0x342A
LDAB #0x00E3
LDP #0x00E3
LDAB #0x00BB
STORB 0xAADD
LDAB #0x00BB
STORB 0xAADE
MVR D,#0x0053
LDAB #0x000E
SUB B,D
STORB 0x01FA
MVR D,#0x0015
LDAB #0x0065
SUB B,D
STORB 0x01FB
MVR D,#0x0017
LDAB #0x007B
SUB B,D
STORB 0x01FC
MVR D,#0x00E3
LDAB #0x006C
SUB B,D
STORB 0x01FD
MVR D,#0x00B5
LDAB #0x00A5
SUB B,D
STORB 0x01FE
MVR D,#0x003F
LDAB #0x0079
SUB B,D
STORB 0x01FF
MVR D,#0x00C7
LDAB #0x00ED
SUB B,D
TSA
STORA 0x0200
MVR D,#0x00AC
LDAB #0x0065
SUB B,D
TSA
STORA 0x0201
MVR D,#0x0048
LDAB #0x00E3
SUB B,D
TSA
STORA 0x0202
MVR D,#0x0017
LDAB #0x00BB
SUB B,D
TSA
STORA 0x0203
MVR D,#0x000B
LDAB #0x00B6
SUB B,D
TSA
STORA 0x0204
MVR D,#0x003E
LDAB #0x005A
SUB B,D
TSA
STORA 0x0205
WAI 
