LDZ #0x342A
LDAB #0x00E3
LDP #0x00E3
LDAB #0x00BB
STORB 0xAADD
LDAB #0x00BB
STORB 0xAADE
MVR D,#0x003F
LDAB #0x0009
XOR B,D
STORB 0x01FA
MVR D,#0x0079
LDAB #0x002B
XOR B,D
STORB 0x01FB
MVR D,#0x0094
LDAB #0x00A2
XOR B,D
STORB 0x01FC
MVR D,#0x0010
LDAB #0x00CC
XOR B,D
STORB 0x01FD
MVR D,#0x006D
LDAB #0x0066
XOR B,D
STORB 0x01FE
MVR D,#0x007F
LDAB #0x00F8
XOR B,D
STORB 0x01FF
MVR D,#0x00CA
LDAB #0x0003
XOR B,D
TSA
STORA 0x0200
MVR D,#0x004E
LDAB #0x00F8
XOR B,D
TSA
STORA 0x0201
MVR D,#0x00ED
LDAB #0x00F5
XOR B,D
TSA
STORA 0x0202
MVR D,#0x007B
LDAB #0x0028
XOR B,D
TSA
STORA 0x0203
MVR D,#0x00AD
LDAB #0x00D8
XOR B,D
TSA
STORA 0x0204
MVR D,#0x00E0
LDAB #0x007A
XOR B,D
TSA
STORA 0x0205
WAI 