
#include <arpa/inet.h>
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#pragma comment(lib, "wsock32.lib")

#define STUDENT_NUMBER "12345678"

#define IP_ADDRESS_SERVER "127.0.0.1"

#define PORT_SERVER 0x1984 // We define a port that we are going to use.
#define PORT_CLIENT 0x1985 // We define a port that we are going to use.

#define WORD unsigned short
#define DWORD unsigned long
#define BYTE unsigned char

#define MAX_FILENAME_SIZE 500
#define MAX_BUFFER_SIZE 500

// Picking up where windows falls short

// Dont scream, windows
#define SOCKADDR_IN sockaddr_in

// In linux, sockets are referred to using descriptors.
#define SOCKET int

// *Sighs*
#define SOCKADDR sockaddr

// Some wonky data types that doesn't exist on linux
#define WSADATA char *
#define _TCHAR char

// Socket startup and cleanup functions
// Why, you ask? Do I look like I know?
#define WSAStartup(t, y) 0
#define WSACleanup()                                                           \
  do {                                                                         \
  } while (0)

// Because word?
#define MAKEWORD

// Oy vey windows.
#define closesocket(sock) close(sock)

// Seriously? A seperate function, just to pass the address and feel smug
// about not assigning it? Nice save. thx.
#define fopen_s(_ptr, path, mode)                                              \
  do {                                                                         \
    *_ptr = fopen(path, mode);                                                 \
  } while (0)

// Doesn't exist on linux
#define SOCKET_ERROR -1

SOCKADDR_IN server_addr;
SOCKADDR_IN client_addr;

SOCKET sock; // This is our socket, it is the handle to the IO address to
             // read/write packets

WSADATA data;

char InputBuffer[MAX_BUFFER_SIZE];

char hex_file[MAX_BUFFER_SIZE];
char trc_file[MAX_BUFFER_SIZE];

//////////////////////////
//   Registers          //
//////////////////////////

#define FLAG_N 0x80
#define FLAG_Z 0x40
#define FLAG_I 0x10
#define FLAG_C 0x01
#define REGISTER_A 5
#define REGISTER_B 4
#define REGISTER_F 3
#define REGISTER_E 2
#define REGISTER_D 1
#define REGISTER_C 0
WORD BaseRegister;
BYTE PageRegister;

BYTE Registers[6];
BYTE Flags;
WORD ProgramCounter;
WORD StackPointer;

////////////
// Memory //
////////////

#define MEMORY_SIZE 65536

BYTE Memory[MEMORY_SIZE];

#define TEST_ADDRESS_1 0x01FA
#define TEST_ADDRESS_2 0x01FB
#define TEST_ADDRESS_3 0x01FC
#define TEST_ADDRESS_4 0x01FD
#define TEST_ADDRESS_5 0x01FE
#define TEST_ADDRESS_6 0x01FF
#define TEST_ADDRESS_7 0x0200
#define TEST_ADDRESS_8 0x0201
#define TEST_ADDRESS_9 0x0202
#define TEST_ADDRESS_10 0x0203
#define TEST_ADDRESS_11 0x0204
#define TEST_ADDRESS_12 0x0205

///////////////////////
// Control variables //
///////////////////////

bool memory_in_range = true;
bool halt = false;

///////////////////////
// Disassembly table //
///////////////////////

char opcode_mneumonics[][14] = {
    "JR abs       ", "CCC abs      ", "CCS abs      ", "CNE abs      ",
    "CEQ abs      ", "CMI abs      ", "CPL abs      ", "CHI abs      ",
    "CLE abs      ", "TSA impl     ", "TAP impl     ", "TPA impl     ",
    "ILLEGAL     ",  "ILLEGAL     ",  "LX  #,A      ", "ILLEGAL     ",

    "TST abs      ", "INC abs      ", "DEC abs      ", "RCR abs      ",
    "RLC abs      ", "ASL abs      ", "ASR abs      ", "LSR abs      ",
    "NOT abs      ", "NEG abs      ", "RL abs       ", "RR abs       ",
    "STORA abs    ", "STORB abs    ", "ILLEGAL     ",  "ILLEGAL     ",

    "TSTA A,A     ", "INCA A,A     ", "DECA A,A     ", "RCRA A,A     ",
    "RLCA A,A     ", "ASLA A,A     ", "ASRA A,A     ", "LSRA A,A     ",
    "NOTA A,A     ", "NEGA A,0     ", "RLA A,A      ", "RRA A,A      ",
    "STORA zpg    ", "STORB zpg    ", "NOP impl     ", "WAI impl     ",

    "TSTB B,B     ", "INCB B,B     ", "DECB B,B     ", "RCRB B,B     ",
    "RLCB B,B     ", "ASLB B,B     ", "ASRB B,B     ", "LSRB B,B     ",
    "NOTB B,B     ", "NEGB B,0     ", "RLB B,B      ", "RRB B,B      ",
    "STORA (ind)  ", "STORB (ind)  ", "SWI impl     ", "RTI impl     ",

    "STP abs      ", "LDAA  #      ", "LDAB  #      ", "LODS  #      ",
    "LDZ  #       ", "ILLEGAL     ",  "ILLEGAL     ",  "DEP impl     ",
    "INP impl     ", "DEZ impl     ", "INZ impl     ", "ILLEGAL     ",
    "STORA pag    ", "STORB pag    ", "ILLEGAL     ",  "ILLEGAL     ",

    "STP zpg      ", "LDAA abs     ", "LDAB abs     ", "LODS abs     ",
    "LDZ abs      ", "STZ abs      ", "CLC impl     ", "STC impl     ",
    "CLI impl     ", "SEI impl     ", "CMC impl     ", "ILLEGAL     ",
    "STORA bas    ", "STORB bas    ", "PUSH  ,A     ", "POP A,       ",

    "STP (ind)    ", "LDAA zpg     ", "LDAB zpg     ", "LODS zpg     ",
    "LDZ zpg      ", "STZ zpg      ", "ILLEGAL     ",  "ADD A,C      ",
    "SUB A,C      ", "CMP A,C      ", "IOR A,C      ", "AND A,C      ",
    "XOR A,C      ", "JMP abs      ", "PUSH  ,B     ", "POP B,       ",

    "STP pag      ", "LDAA (ind)   ", "LDAB (ind)   ", "LODS (ind)   ",
    "LDZ (ind)    ", "STZ (ind)    ", "ILLEGAL     ",  "ADD A,D      ",
    "SUB A,D      ", "CMP A,D      ", "IOR A,D      ", "AND A,D      ",
    "XOR A,D      ", "ILLEGAL     ",  "PUSH  ,s     ", "POP s,       ",

    "STP bas      ", "LDAA pag     ", "LDAB pag     ", "LODS pag     ",
    "LDZ pag      ", "STZ pag      ", "RTS impl     ", "ADD A,E      ",
    "SUB A,E      ", "CMP A,E      ", "IOR A,E      ", "AND A,E      ",
    "XOR A,E      ", "ILLEGAL     ",  "PUSH  ,C     ", "POP C,       ",

    "LDP  #       ", "LDAA bas     ", "LDAB bas     ", "LODS bas     ",
    "LDZ bas      ", "STZ bas      ", "ILLEGAL     ",  "ADD A,F      ",
    "SUB A,F      ", "CMP A,F      ", "IOR A,F      ", "AND A,F      ",
    "XOR A,F      ", "ILLEGAL     ",  "PUSH  ,D     ", "POP D,       ",

    "LDP abs      ", "MOV A,A      ", "MOV B,A      ", "MOV C,A      ",
    "MOV D,A      ", "MOV E,A      ", "MOV F,A      ", "ADD B,C      ",
    "SUB B,C      ", "CMP B,C      ", "IOR B,C      ", "AND B,C      ",
    "XOR B,C      ", "ILLEGAL     ",  "PUSH  ,E     ", "POP E,       ",

    "LDP zpg      ", "MOV A,B      ", "MOV B,B      ", "MOV C,B      ",
    "MOV D,B      ", "MOV E,B      ", "MOV F,B      ", "ADD B,D      ",
    "SUB B,D      ", "CMP B,D      ", "IOR B,D      ", "AND B,D      ",
    "XOR B,D      ", "ILLEGAL     ",  "PUSH  ,F     ", "POP F,       ",

    "LDP (ind)    ", "MOV A,C      ", "MOV B,C      ", "MOV C,C      ",
    "MOV D,C      ", "MOV E,C      ", "MOV F,C      ", "ADD B,E      ",
    "SUB B,E      ", "CMP B,E      ", "IOR B,E      ", "AND B,E      ",
    "XOR B,E      ", "ILLEGAL     ",  "ILLEGAL     ",  "ILLEGAL     ",

    "LDP pag      ", "MOV A,D      ", "MOV B,D      ", "MOV C,D      ",
    "MOV D,D      ", "MOV E,D      ", "MOV F,D      ", "ADD B,F      ",
    "SUB B,F      ", "CMP B,F      ", "IOR B,F      ", "AND B,F      ",
    "XOR B,F      ", "ILLEGAL     ",  "ILLEGAL     ",  "ILLEGAL     ",

    "LDP bas      ", "MOV A,E      ", "MOV B,E      ", "MOV C,E      ",
    "MOV D,E      ", "MOV E,E      ", "MOV F,E      ", "BRA rel      ",
    "BCC rel      ", "BCS rel      ", "BNE rel      ", "BEQ rel      ",
    "BMI rel      ", "BPL rel      ", "BLS rel      ", "BHI rel      ",

    "ILLEGAL     ",  "MOV A,F      ", "MOV B,F      ", "MOV C,F      ",
    "MOV D,F      ", "MOV E,F      ", "MOV F,F      ", "ILLEGAL     ",
    "MVR  #,C     ", "MVR  #,D     ", "MVR  #,E     ", "MVR  #,F     ",
    "CPIA  #      ", "CPIB  #      ", "ANIA  #      ", "ANIB  #      ",

};
// OPCodes
// LDAA (Loads data into Accumulator A)
#define LDAA_IMM 0x41
#define LDAA_ABS 0x51
#define LDAA_ZPG 0x61
#define LDAA_IND 0x71
#define LDAA_PAG 0x81
#define LDAA_BAS 0x91

// LDAB (Loads data into Accumulator B)
#define LDAB_IMM 0x42
#define LDAB_ABS 0x52
#define LDAB_ZPG 0x62
#define LDAB_IND 0x72
#define LDAB_PAG 0x82
#define LDAB_BAS 0x92

// STORA (Stores Accumulator A into a Memory Address)
// Immediate Addressing is absent.
#define STORA_ABS 0x1C
#define STORA_ZPG 0x2C
#define STORA_IND 0x3C
#define STORA_PAG 0x4C
#define STORA_BAS 0x5C

// STORB (Stores Accumulator B into a Memory Address)
// Immediate Addressing is absent.
#define STORB_ABS 0x1D
#define STORB_ZPG 0x2D
#define STORB_IND 0x3D
#define STORB_PAG 0x4D
#define STORB_BAS 0x5D

#define TST 0x10
#define TSTA 0x20
#define TSTB 0x30

#define INC 0x11
#define INCA 0x21
#define INCB 0x31

#define DEC 0x12
#define DECA 0x22
#define DECB 0x32

#define CPIA 0xFC
#define CPIB 0xFD
#define ANIA 0xFE
#define ANIB 0xFF

#define JMP 0x6D
#define JR 0x00
#define RTS 0x86
#define BRA 0xE7

#define RCR 0x13
#define RCRA 0x23
#define RCRB 0x33

#define RLC 0x14
#define RLCA 0x24
#define RLCB 0x34

#define ASL 0x15
#define ASLA 0x25
#define ASLB 0x35

#define ASR 0x16
#define ASRA 0x26
#define ASRB 0x36

#define LSR 0x17
#define LSRA 0x27
#define LSRB 0x37

#define NOT 0x18
#define NOTA 0x28
#define NOTB 0x38

#define NEG 0x19
#define NEGA 0x29
#define NEGB 0x39

#define RL 0x1A
#define RLA 0x2A
#define RLB 0x3A

#define RR 0x1B
#define RRA 0x2B
#define RRB 0x3B

#define LODS_IMM 0x43
#define LODS_ABS 0x53
#define LODS_ZPG 0x63
#define LODS_IND 0x73
#define LODS_PAG 0x83
#define LODS_BAS 0x93

#define TSA 0x09

#define PUSH_A 0x5E
#define PUSH_B 0x6E
#define PUSH_FL 0x7E
#define PUSH_C 0x8E
#define PUSH_D 0x9E
#define PUSH_E 0xAE
#define PUSH_F 0xBE

#define POP_A 0x5F
#define POP_B 0x6F
#define POP_FL 0x7F
#define POP_C 0x8F
#define POP_D 0x9F
#define POP_E 0xAF
#define POP_F 0xBF

#define LX 0x0E
#define MVR_C 0xF8
#define MVR_D 0xF9
#define MVR_E 0xFA
#define MVR_F 0xFB

#define BCC 0xE8
#define BCS 0xE9
#define BNE 0xEA
#define BEQ 0xEB
#define BMI 0xEC
#define BPL 0xED
#define BLS 0xEE
#define BHI 0xEF

#define CCC 0x01
#define CCS 0x02
#define CNE 0x03
#define CEQ 0x04
#define CMI 0x05
#define CPL 0x06
#define CHI 0x07
#define CLE 0x08

#define CLC 0x56
#define STC 0x57

#define CLI 0x58
#define SEI 0x59

#define CMC 0x5A
#define NOP 0x2E
#define WAI 0x2F

#define SWI 0x3E
#define RTI 0x3F

#define LDP_IMM 0x90
#define LDP_ABS 0xA0
#define LDP_ZPG 0xB0
#define LDP_IND 0xC0
#define LDP_PAG 0xD0
#define LDP_BAS 0xE0

#define STP_ABS 0x40
#define STP_ZPG 0x50
#define STP_IND 0x60
#define STP_PAG 0x70
#define STP_BAS 0x80

#define TAP 0x0A
#define TPA 0x0B

#define LDZ_IMM 0x44
#define LDZ_ABS 0x54
#define LDZ_ZPG 0x64
#define LDZ_IND 0x74
#define LDZ_PAG 0x84
#define LDZ_BAS 0x94

#define STZ_ABS 0x55
#define STZ_ZPG 0x65
#define STZ_IND 0x75
#define STZ_PAG 0x85
#define STZ_BAS 0x95

#define DEZ 0x49
#define INZ 0x4A

#define DPE 0x47
#define INP 0x48

#define ADD_A_C 0x67
#define ADD_A_D 0x77
#define ADD_A_E 0x87
#define ADD_A_F 0x97

#define ADD_B_C 0xA7
#define ADD_B_D 0xB7
#define ADD_B_E 0xC7
#define ADD_B_F 0xD7

#define SUB_A_C 0x68
#define SUB_A_D 0x78
#define SUB_A_E 0x88
#define SUB_A_F 0x98

#define SUB_B_C 0xA8
#define SUB_B_D 0xB8
#define SUB_B_E 0xC8
#define SUB_B_F 0xD8

#define CMP_A_C 0x69
#define CMP_A_D 0x79
#define CMP_A_E 0x89
#define CMP_A_F 0x99

#define CMP_B_C 0xA9
#define CMP_B_D 0xB9
#define CMP_B_E 0xC9
#define CMP_B_F 0xD9

#define IOR_A_C 0x6A
#define IOR_A_D 0x7A
#define IOR_A_E 0x8A
#define IOR_A_F 0x9A

#define IOR_B_C 0xAA
#define IOR_B_D 0xBA
#define IOR_B_E 0xCA
#define IOR_B_F 0xDA

#define AND_A_C 0x6B
#define AND_A_D 0x7B
#define AND_A_E 0x8B
#define AND_A_F 0x9B

#define AND_B_C 0xAB
#define AND_B_D 0xBB
#define AND_B_E 0xCB
#define AND_B_F 0xDB

#define XOR_A_C 0x6C
#define XOR_A_D 0x7C
#define XOR_A_E 0x8C
#define XOR_A_F 0x9C

#define XOR_B_C 0xAC
#define XOR_B_D 0xBC
#define XOR_B_E 0xCC
#define XOR_B_F 0xDC

////////////////////////////////////////////////////////////////////////////////
//                           Simulator/Emulator (Start)                       //
////////////////////////////////////////////////////////////////////////////////
BYTE fetch() {
  BYTE byte = 0;

  if ((ProgramCounter >= 0) && (ProgramCounter <= MEMORY_SIZE)) {
    memory_in_range = true;
    byte = Memory[ProgramCounter];
    ProgramCounter++;
  } else {
    memory_in_range = false;
  }
  return byte;
}

#define MSB 0x80
#define LSB 0x01
#define BYTE_MAX 0xFF

void fset(int flag) { Flags |= flag; }
void ftoggle(int flag) { Flags ^= flag; }
void fclear(int flag) { Flags &= 0xFF - flag; }
bool efcheck(int flag) { return (Flags & flag) != 0; }
bool fcheck(int flag) { return (Flags & flag) == flag; }

bool msbset(BYTE byte) { return (byte & MSB) == MSB; }
bool lsbset(BYTE byte) { return (byte & LSB) == LSB; }

void build_address_abs(BYTE *high, BYTE *low, WORD *addr) {
  *low = fetch();
  *high = fetch();
  *addr += (WORD)((WORD)*high << 8) + *low;
}

void build_address_ind(BYTE *high, BYTE *low, WORD *addr) {
  build_address_abs(high, low, addr);
  *low = Memory[*addr];
  *high = Memory[*addr + 1];
  *addr = (WORD)((WORD)*high << 8) + *low;
}

void build_address_pag(BYTE *high, BYTE *low, WORD *addr) {
  *high = PageRegister;
  *low = fetch();
  *addr += (WORD)((WORD)*high << 8) + *low;
}

void build_address_zpg(BYTE *high, BYTE *low, WORD *addr) {
  *addr += 0x0000 | (WORD)fetch();
}

void build_address_bas(BYTE *high, BYTE *low, WORD *addr) {
  *low = fetch();
  if (msbset(*low)) {
    *addr += BaseRegister + (0x00 - *low);
  } else {
    *addr += BaseRegister + *low;
  }
}

bool is_addressable(WORD addr) { return addr >= 0 && addr < MEMORY_SIZE; };

// Sets ZERO flag
void ztest(BYTE byte) {
  if (byte == 0) {
    fset(FLAG_Z);
  } else {
    fclear(FLAG_Z);
  }
}

void ntest(BYTE byte) {
  if (msbset(byte)) {
    fset(FLAG_N);
  } else {
    fclear(FLAG_N);
  }
}

void ntestw(WORD word) {
  if (word < 0) {
    Flags |= FLAG_N;
  } else {
    Flags &= (BYTE_MAX - FLAG_N);
  }
}

void ztestw(WORD word) {
  if (word == 0) {
    Flags |= FLAG_Z;
  } else {
    Flags &= (BYTE_MAX - FLAG_Z);
  }
}

void ctest(WORD word) {
  if (word > BYTE_MAX) {
    fset(FLAG_C);
  } else {
    fclear(FLAG_C);
  }
}

void testw(WORD word) {
  ntestw(word);
  ztestw(word);
}
void test(BYTE dst) {
  ntest(dst);
  ztest(dst);
}

void logical_shift_right(BYTE *byte) {
  if (lsbset(*byte) != fcheck(FLAG_C)) {
    ftoggle(FLAG_C);
  }
  *byte >>= 1;
  test(*byte);
}

void negate(BYTE *byte) {
  WORD buffer = ~*byte;
  *byte = (BYTE)buffer;
  ctest(buffer);
  test(*byte);
}

void twos_complement(BYTE *byte) {
  *byte = 0 - *byte;
  test(*byte);
}

void rotate_right(BYTE *byte) {
  BYTE lsb = (*byte & LSB) == LSB;
  *byte >>= 1;
  *byte |= (lsb << 7);
  test(*byte);
}

void rotate_left(BYTE *byte) {
  BYTE msb = (*byte & MSB) == MSB;
  *byte <<= 1;
  *byte |= msb;
  test(*byte);
}

void upush(BYTE reg) {
  Memory[StackPointer] = reg;
  StackPointer--;
}

bool push(BYTE reg) {
  if (StackPointer >= 1 && StackPointer < MEMORY_SIZE) {
    upush(reg);
    return true;
  }
  return false;
}

void upop(BYTE *reg) {
  StackPointer++;
  *reg = Memory[StackPointer];
}

bool pop(BYTE *reg) {
  if (StackPointer >= 0 && StackPointer < MEMORY_SIZE - 1) {
    upop(reg);
    return true;
  }
  return false;
}

bool pushw(WORD word) {
  if ((StackPointer >= 2) && (StackPointer < MEMORY_SIZE)) {
    // Low first
    upush(word & BYTE_MAX);
    // High second
    upush((word >> 8) & BYTE_MAX);
    return true;
  }
  return false;
}
bool popw(WORD *word) {
  BYTE LB = 0;
  BYTE HB = 0;

  if ((StackPointer >= 0) && (StackPointer < MEMORY_SIZE - 2)) {
    upop(&HB);
    upop(&LB);
    *word = ((WORD)HB << 8) + (WORD)LB;
    return true;
  }
  return false;
}

void branch(bool condition) {
  BYTE LB = fetch();

  if (!condition)
    return;

  WORD offset = (WORD)LB;

  if ((offset & MSB) != 0) {
    offset = offset + 0xFF00;
  }

  if (is_addressable(ProgramCounter + offset)) {
    ProgramCounter += offset;
  }
}

void add(BYTE *dst, BYTE src) {
  WORD buffer = (WORD)*dst + (WORD)src;

  if ((Flags & FLAG_C) != 0) {
    buffer++;
  }

  ctest(buffer);
  *dst = (BYTE)buffer;
  test(*dst);
}

void sub(BYTE *dst, BYTE src) {
  WORD buffer = (WORD)*dst - (WORD)src;
  if ((Flags & FLAG_C) != 0) {
    buffer--;
  }
  ctest(buffer);
  *dst = (BYTE)buffer;
  test(*dst);
}

void cmp(BYTE dst, BYTE src) {
  WORD buffer = (WORD)dst - (WORD)src;

  ctest(buffer);
  test((BYTE)buffer);
}

void ior(BYTE *dst, BYTE src) {
  *dst |= src;
  test(*dst);
}

void bitwise_and(BYTE *dst, BYTE src) {
  *dst &= src;
  test(*dst);
}

void bitwise_xor(BYTE *dst, BYTE src) {
  *dst ^= src;
  test(*dst);
}

void decrement(BYTE *dst) {
  *dst = *dst - 1;
  test(*dst);
}

void increment(BYTE *dst) {
  *dst = *dst + 1;
  test(*dst);
}

void decrement_value_at(WORD addr) {
  if (is_addressable(addr)) {
    decrement(&Memory[addr]);
  }
}

void increment_value_at(WORD addr) {
  if (is_addressable(addr)) {
    increment(&Memory[addr]);
  }
}

void test_value_at(WORD addr) {
  if (is_addressable(addr)) {
    test(Memory[addr]);
  }
}

void rotate_right_through_carry(BYTE *byte) {
  BYTE old_carry = fcheck(FLAG_C);
  if ((*byte & LSB) != old_carry) {
    ftoggle(FLAG_C);
  }
  *byte >>= 1;
  *byte |= old_carry << 7;
  test(*byte);
}

void rotate_left_through_carry(BYTE *byte) {
  BYTE old_carry = fcheck(FLAG_C);

  if ((*byte & MSB) >> 7 != old_carry) {
    ftoggle(FLAG_C);
  }
  *byte <<= 1;
  *byte |= old_carry;
  test(*byte);
}

void arithmetic_shift_left(BYTE *byte) {
  if (((*byte & MSB) >> 7) != fcheck(FLAG_C)) {
    ftoggle(FLAG_C);
  }
  *byte <<= 1;
  test(*byte);
}

void arithmetic_shift_right(BYTE *byte) {
  if ((*byte & LSB) != fcheck(FLAG_C)) {
    ftoggle(FLAG_C);
  }

  *byte >>= 1;
  *byte |= (*byte >> 6) << 7;
  test(*byte);
}

void call(bool condition) {
  BYTE LB = 0;
  BYTE HB = 0;
  WORD address = 0;
  build_address_abs(&HB, &LB, &address);

  if (is_addressable(address) && condition) {
    pushw(ProgramCounter);
    ProgramCounter = address;
  }
}

void load(BYTE *dst, WORD address) {
  if (is_addressable(address)) {
    *dst = Memory[address];
  }
}

void store(BYTE dst, WORD address) {
  if (is_addressable(address)) {
    Memory[address] = dst;
  }
}

void loadw(WORD *word, WORD address) {
  if (address >= 0 && address <= MEMORY_SIZE - 2) {
    *word = (WORD)Memory[address];
    *word |= (((WORD)Memory[address + 1]) << 8);

    testw(*word);
  }
}
void storew(WORD word, WORD address) {
  if (address >= 0 && address <= MEMORY_SIZE - 2) {
    Memory[address] = (BYTE)word;
    Memory[address + 1] = (BYTE)word >> 8;
    ntestw(word);
    ztestw(word);
  }
}

void Group_1(BYTE opcode) {
  BYTE LB = 0;
  BYTE HB = 0;

  WORD address = 0;
  WORD data = 0;

  switch (opcode) {
  default:
    printf("Unimplemented opcode: %#04X\n", opcode);
    assert(strcmp(opcode_mneumonics[opcode], "ILLEGAL     ") == 0);
    break;

  case LDAA_IMM:
    data = fetch();
    Registers[REGISTER_A] = data;
    break;

  // LDAA(Load Accumulator A) abs
  case LDAA_ABS:
    build_address_abs(&HB, &LB, &address);
    load(&Registers[REGISTER_A], address);
    break;

  case LDAA_ZPG:
    build_address_zpg(&HB, &LB, &address);
    load(&Registers[REGISTER_A], address);
    break;

  case LDAA_IND:
    build_address_ind(&HB, &LB, &address);
    load(&Registers[REGISTER_A], address);
    break;

  case LDAA_PAG:
    build_address_pag(&HB, &LB, &address);
    load(&Registers[REGISTER_A], address);
    break;
  case LDAA_BAS:
    build_address_bas(&HB, &LB, &address);
    load(&Registers[REGISTER_A], address);
    break;

  // LDAB(Load Accumulator B) abs
  case LDAB_IMM:
    data = fetch();
    Registers[REGISTER_B] = data;
    break;

  case LDAB_ABS:
    build_address_abs(&HB, &LB, &address);
    load(&Registers[REGISTER_B], address);
    break;

  case LDAB_ZPG:
    build_address_zpg(&HB, &LB, &address);
    load(&Registers[REGISTER_B], address);
    break;

  case LDAB_IND:
    build_address_ind(&HB, &LB, &address);
    load(&Registers[REGISTER_B], address);
    break;

  case LDAB_PAG:
    build_address_pag(&HB, &LB, &address);
    load(&Registers[REGISTER_B], address);
    break;

  case LDAB_BAS:
    build_address_bas(&HB, &LB, &address);
    load(&Registers[REGISTER_B], address);
    break;

  case STORA_ABS:
    build_address_abs(&HB, &LB, &address);
    store(Registers[REGISTER_A], address);
    break;

  case STORA_ZPG:
    build_address_zpg(&HB, &LB, &address);
    store(Registers[REGISTER_A], address);
    break;

  case STORA_IND:
    build_address_ind(&HB, &LB, &address);
    store(Registers[REGISTER_A], address);
    break;

  case STORA_PAG:
    build_address_pag(&HB, &LB, &address);
    store(Registers[REGISTER_A], address);
    break;

  case STORA_BAS:
    build_address_bas(&HB, &LB, &address);
    store(Registers[REGISTER_A], address);
    break;

  case STORB_ABS:
    build_address_abs(&HB, &LB, &address);
    store(Registers[REGISTER_B], address);
    break;

  case STORB_ZPG:
    build_address_zpg(&HB, &LB, &address);
    store(Registers[REGISTER_B], address);
    break;

  case STORB_IND:
    build_address_ind(&HB, &LB, &address);
    store(Registers[REGISTER_B], address);
    break;

  case STORB_PAG:
    build_address_pag(&HB, &LB, &address);
    store(Registers[REGISTER_B], address);
    break;

  case STORB_BAS:
    build_address_bas(&HB, &LB, &address);
    store(Registers[REGISTER_B], address);
    break;

  case ADD_A_C:
    add(&Registers[REGISTER_A], Registers[REGISTER_C]);
    break;
  case ADD_A_D:
    add(&Registers[REGISTER_A], Registers[REGISTER_D]);
    break;
  case ADD_A_E:
    add(&Registers[REGISTER_A], Registers[REGISTER_E]);
    break;
  case ADD_A_F:
    add(&Registers[REGISTER_A], Registers[REGISTER_F]);
    break;
  case ADD_B_C:
    add(&Registers[REGISTER_B], Registers[REGISTER_C]);
    break;
  case ADD_B_D:
    add(&Registers[REGISTER_B], Registers[REGISTER_D]);
    break;
  case ADD_B_E:
    add(&Registers[REGISTER_B], Registers[REGISTER_E]);
    break;
  case ADD_B_F:
    add(&Registers[REGISTER_B], Registers[REGISTER_F]);
    break;

  case SUB_A_C:
    sub(&Registers[REGISTER_A], Registers[REGISTER_C]);
    break;
  case SUB_A_D:
    sub(&Registers[REGISTER_A], Registers[REGISTER_D]);
    break;
  case SUB_A_E:
    sub(&Registers[REGISTER_A], Registers[REGISTER_E]);
    break;
  case SUB_A_F:
    sub(&Registers[REGISTER_A], Registers[REGISTER_F]);
    break;
  case SUB_B_C:
    sub(&Registers[REGISTER_B], Registers[REGISTER_C]);
    break;
  case SUB_B_D:
    sub(&Registers[REGISTER_B], Registers[REGISTER_D]);
    break;
  case SUB_B_E:
    sub(&Registers[REGISTER_B], Registers[REGISTER_E]);
    break;
  case SUB_B_F:
    sub(&Registers[REGISTER_B], Registers[REGISTER_F]);
    break;

  case CMP_A_C:
    cmp(Registers[REGISTER_A], Registers[REGISTER_C]);
    break;
  case CMP_A_D:
    cmp(Registers[REGISTER_A], Registers[REGISTER_D]);
    break;
  case CMP_A_E:
    cmp(Registers[REGISTER_A], Registers[REGISTER_E]);
    break;
  case CMP_A_F:
    cmp(Registers[REGISTER_A], Registers[REGISTER_F]);
    break;
  case CMP_B_C:
    cmp(Registers[REGISTER_B], Registers[REGISTER_C]);
    break;
  case CMP_B_D:
    cmp(Registers[REGISTER_B], Registers[REGISTER_D]);
    break;
  case CMP_B_E:
    cmp(Registers[REGISTER_B], Registers[REGISTER_E]);
    break;
  case CMP_B_F:
    cmp(Registers[REGISTER_B], Registers[REGISTER_F]);
    break;

  case IOR_A_C:
    ior(&Registers[REGISTER_A], Registers[REGISTER_C]);
    break;
  case IOR_A_D:
    ior(&Registers[REGISTER_A], Registers[REGISTER_D]);
    break;
  case IOR_A_E:
    ior(&Registers[REGISTER_A], Registers[REGISTER_E]);
    break;
  case IOR_A_F:
    ior(&Registers[REGISTER_A], Registers[REGISTER_F]);
    break;
  case IOR_B_C:
    ior(&Registers[REGISTER_B], Registers[REGISTER_C]);
    break;
  case IOR_B_D:
    ior(&Registers[REGISTER_B], Registers[REGISTER_D]);
    break;
  case IOR_B_E:
    ior(&Registers[REGISTER_B], Registers[REGISTER_E]);
    break;
  case IOR_B_F:
    ior(&Registers[REGISTER_B], Registers[REGISTER_F]);
    break;

  case AND_A_C:
    bitwise_and(&Registers[REGISTER_A], Registers[REGISTER_C]);
    break;
  case AND_A_D:
    bitwise_and(&Registers[REGISTER_A], Registers[REGISTER_D]);
    break;
  case AND_A_E:
    bitwise_and(&Registers[REGISTER_A], Registers[REGISTER_E]);
    break;
  case AND_A_F:
    bitwise_and(&Registers[REGISTER_A], Registers[REGISTER_F]);
    break;
  case AND_B_C:
    bitwise_and(&Registers[REGISTER_B], Registers[REGISTER_C]);
    break;
  case AND_B_D:
    bitwise_and(&Registers[REGISTER_B], Registers[REGISTER_D]);
    break;
  case AND_B_E:
    bitwise_and(&Registers[REGISTER_B], Registers[REGISTER_E]);
    break;
  case AND_B_F:
    bitwise_and(&Registers[REGISTER_B], Registers[REGISTER_F]);
    break;

  case XOR_A_C:
    bitwise_xor(&Registers[REGISTER_A], Registers[REGISTER_C]);
    break;
  case XOR_A_D:
    bitwise_xor(&Registers[REGISTER_A], Registers[REGISTER_D]);
    break;
  case XOR_A_E:
    bitwise_xor(&Registers[REGISTER_A], Registers[REGISTER_E]);
    break;
  case XOR_A_F:
    bitwise_xor(&Registers[REGISTER_A], Registers[REGISTER_F]);
    break;
  case XOR_B_C:
    bitwise_xor(&Registers[REGISTER_B], Registers[REGISTER_C]);
    break;
  case XOR_B_D:
    bitwise_xor(&Registers[REGISTER_B], Registers[REGISTER_D]);
    break;
  case XOR_B_E:
    bitwise_xor(&Registers[REGISTER_B], Registers[REGISTER_E]);
    break;
  case XOR_B_F:
    bitwise_xor(&Registers[REGISTER_B], Registers[REGISTER_F]);
    break;

  case CPIA:
    cmp(fetch(), Registers[REGISTER_A]);
    break;

  case CPIB:
    cmp(fetch(), Registers[REGISTER_B]);
    break;

  case ANIA:
    bitwise_and(&Registers[REGISTER_A], fetch());
    break;

  case ANIB:
    bitwise_and(&Registers[REGISTER_B], fetch());
    break;

  case TST:
    build_address_abs(&HB, &LB, &address);
    test_value_at(address);
    break;

  case INC:
    build_address_abs(&HB, &LB, &address);
    increment_value_at(address);
    break;

  case DEC:
    build_address_abs(&HB, &LB, &address);
    decrement_value_at(address);
    break;

  case TSTA:
    test(Registers[REGISTER_A]);
    break;
  case TSTB:
    test(Registers[REGISTER_B]);
    break;
  case INCA:
    increment(&Registers[REGISTER_A]);
    break;

  case INCB:
    increment(&Registers[REGISTER_B]);
    break;

  case DECA:
    decrement(&Registers[REGISTER_A]);
    break;

  case DECB:
    decrement(&Registers[REGISTER_B]);
    break;

  case JMP:
    build_address_abs(&HB, &LB, &address);
    ProgramCounter = address;
    break;

  case JR:
    build_address_abs(&HB, &LB, &address);
    // We are going to push two bytes
    if (pushw(ProgramCounter)) {
      ProgramCounter = address;
    }
    break;

  case RTS:
    popw(&ProgramCounter);
    break;

  case BRA:
    branch(true);
    break;

  // ROTATE RIGHT THROUGH MEMORY
  case RCR:
    build_address_abs(&HB, &LB, &address);
    if (is_addressable(address)) {
      rotate_right_through_carry(&Memory[address]);
    }
    break;

  case RCRA:
    rotate_right_through_carry(&Registers[REGISTER_A]);
    break;

  case RCRB:
    rotate_right_through_carry(&Registers[REGISTER_B]);
    break;

  case RLC:
    build_address_abs(&HB, &LB, &address);
    if (is_addressable(address)) {
      rotate_left_through_carry(&Memory[address]);
    }
    break;

  case RLCA:
    rotate_left_through_carry(&Registers[REGISTER_A]);
    break;

  case RLCB:
    rotate_left_through_carry(&Registers[REGISTER_B]);
    break;

  // Shift Left
  case ASL:
    build_address_abs(&HB, &LB, &address);
    if (is_addressable(address)) {
      arithmetic_shift_left(&Memory[address]);
    }
    break;
  case ASLA:
    arithmetic_shift_left(&Registers[REGISTER_A]);
    break;
  case ASLB:
    arithmetic_shift_left(&Registers[REGISTER_B]);
    break;

  // Arithmetic shift right
  case ASR:
    build_address_abs(&HB, &LB, &address);
    if (is_addressable(address)) {
      arithmetic_shift_right(&Memory[address]);
    }
    break;

  case ASRA:
    arithmetic_shift_right(&Registers[REGISTER_A]);
    break;

  case ASRB:
    arithmetic_shift_right(&Registers[REGISTER_B]);
    break;

  case LSR:
    build_address_abs(&HB, &LB, &address);
    if (is_addressable(address)) {
      logical_shift_right(&Memory[address]);
    }
    break;

  case LSRA:
    logical_shift_right(&Registers[REGISTER_A]);
    break;

  case LSRB:
    logical_shift_right(&Registers[REGISTER_B]);
    break;

  case NOT:
    build_address_abs(&HB, &LB, &address);
    if (is_addressable(address)) {
      negate(&Memory[address]);
    }
    break;

  case NOTA:
    negate(&Registers[REGISTER_A]);
    break;

  case NOTB:
    negate(&Registers[REGISTER_B]);
    break;

  case NEG:
    build_address_abs(&HB, &LB, &address);
    if (is_addressable(address)) {
      twos_complement(&Memory[address]);
    }
    break;

  case NEGA:
    twos_complement(&Registers[REGISTER_A]);
    break;

  case NEGB:
    twos_complement(&Registers[REGISTER_B]);
    break;

  case RL:
    build_address_abs(&HB, &LB, &address);
    if (is_addressable(address)) {
      rotate_left(&Memory[address]);
    }
    break;

  case RLA:
    rotate_left(&Registers[REGISTER_A]);
    break;

  case RLB:
    rotate_left(&Registers[REGISTER_B]);
    break;

  case RR:
    build_address_abs(&HB, &LB, &address);
    if (is_addressable(address)) {
      rotate_right(&Memory[address]);
    }
    break;

  case RRA:
    rotate_right(&Registers[REGISTER_A]);
    break;

  case RRB:
    rotate_right(&Registers[REGISTER_B]);
    break;

  case LODS_IMM:
    StackPointer = (WORD)fetch() | ((WORD)fetch() << 8);
    break;

  case LODS_ABS:
    build_address_abs(&HB, &LB, &address);
    StackPointer = address;
    break;
  case LODS_ZPG:
    build_address_zpg(&HB, &LB, &address);
    StackPointer = address;
    break;
  case LODS_IND:
    build_address_ind(&HB, &LB, &address);
    StackPointer = address;
    break;
  case LODS_PAG:
    build_address_pag(&HB, &LB, &address);
    StackPointer = address;
    break;
  case LODS_BAS:
    build_address_bas(&HB, &LB, &address);
    StackPointer = address;
    break;

  case TSA:
    Registers[REGISTER_A] = Flags;
    break;

  case PUSH_A:
    push(Registers[REGISTER_A]);
    break;

  case PUSH_B:
    push(Registers[REGISTER_B]);
    break;

  case PUSH_FL:
    push(Flags);
    break;

  case PUSH_C:
    push(Registers[REGISTER_C]);
    break;

  case PUSH_D:
    push(Registers[REGISTER_D]);
    break;
  case PUSH_E:
    push(Registers[REGISTER_E]);
    break;

  case PUSH_F:
    push(Registers[REGISTER_F]);
    break;

  case POP_A:
    pop(&Registers[REGISTER_A]);
    break;
  case POP_B:
    pop(&Registers[REGISTER_B]);
    break;
  case POP_FL:
    pop(&Flags);
    break;

  case POP_C:
    pop(&Registers[REGISTER_C]);
    break;

  case POP_D:
    pop(&Registers[REGISTER_D]);
    break;
  case POP_E:
    pop(&Registers[REGISTER_E]);
    break;

  case POP_F:
    pop(&Registers[REGISTER_F]);
    break;
  case LX:
    Registers[REGISTER_A] = fetch();
    Registers[REGISTER_B] = fetch();
    break;
  case MVR_C:
    Registers[REGISTER_C] = fetch();
    break;
  case MVR_D:
    Registers[REGISTER_D] = fetch();
    break;
  case MVR_E:
    Registers[REGISTER_E] = fetch();
    break;
  case MVR_F:
    Registers[REGISTER_F] = fetch();
    break;
  case BCC:
    branch(!fcheck(FLAG_C));
    break;

  case BCS:
    branch(fcheck(FLAG_C));
    break;

  case BNE:
    branch(!fcheck(FLAG_Z));
    break;

  case BEQ:
    branch(fcheck(FLAG_Z));
    break;

  case BMI:
    branch(fcheck(FLAG_N));
    break;

  case BPL:
    branch(!fcheck(FLAG_N));
    break;

  case BLS:
    branch(efcheck(FLAG_C | FLAG_Z));
    break;

  case BHI:
    branch(!efcheck(FLAG_C | FLAG_Z));
    break;
  // Call on Carry Clear
  case CCC:
    call(!fcheck(FLAG_C));
    break;
  // Call on Carry Set
  case CCS:
    call(fcheck(FLAG_C));
    break;
  // Call on Result Not Equal to Zero
  case CNE:
    call(!fcheck(FLAG_Z));
    break;
  case CEQ:
    call(fcheck(FLAG_Z));
    break;
  case CMI:
    call(fcheck(FLAG_N));
    break;
  case CPL:
    call(!fcheck(FLAG_N));
    break;

  case CHI:
    call(efcheck(FLAG_C | FLAG_Z));
    break;
  case CLE:
    call(!efcheck(FLAG_C | FLAG_Z));
    break;
  case CLC:
    fclear(FLAG_C);
    break;

  case STC:
    fset(FLAG_C);
    break;

  case CLI:
    fclear(FLAG_I);
    break;

  case SEI:
    fset(FLAG_I);
    break;

  case CMC:
    ftoggle(FLAG_C);
    break;

  case NOP:
    halt = true;
    break;

  case WAI:
    halt = true;
    break;

  case SWI:
    push(Registers[REGISTER_A]);
    push(Registers[REGISTER_B]);
    pushw(ProgramCounter);
    push(Flags);
    push(Registers[REGISTER_C]);
    push(Registers[REGISTER_D]);
    push(Registers[REGISTER_E]);
    push(Registers[REGISTER_F]);

    Flags |= FLAG_I;
    break;

  case RTI:
    pop(&Registers[REGISTER_F]);
    pop(&Registers[REGISTER_E]);
    pop(&Registers[REGISTER_D]);
    pop(&Registers[REGISTER_C]);
    pop(&Flags);
    popw(&ProgramCounter);
    pop(&Registers[REGISTER_B]);
    pop(&Registers[REGISTER_A]);

    break;

  case LDP_IMM:
    PageRegister = fetch();
    break;
  case LDP_ABS:
    build_address_abs(&HB, &LB, &address);
    load(&PageRegister, address);
    break;
  case LDP_ZPG:
    build_address_zpg(&HB, &LB, &address);
    load(&PageRegister, address);
    break;
  case LDP_IND:
    build_address_ind(&HB, &LB, &address);
    load(&PageRegister, address);
    break;
  case LDP_PAG:
    build_address_pag(&HB, &LB, &address);
    load(&PageRegister, address);
    break;
  case LDP_BAS:
    build_address_bas(&HB, &LB, &address);
    load(&PageRegister, address);
    break;

  case STP_ABS:
    build_address_abs(&HB, &LB, &address);
    store(PageRegister, address);
    break;
  case STP_ZPG:
    build_address_zpg(&HB, &LB, &address);
    store(PageRegister, address);
    break;
  case STP_IND:
    build_address_ind(&HB, &LB, &address);
    store(PageRegister, address);
    break;
    break;
  case STP_PAG:
    build_address_pag(&HB, &LB, &address);
    store(PageRegister, address);
    break;
  case STP_BAS:
    build_address_bas(&HB, &LB, &address);
    store(PageRegister, address);
    break;

  case TAP:
    PageRegister = Registers[REGISTER_A];
    break;

  case TPA:
    Registers[REGISTER_A] = PageRegister;
    break;

  case LDZ_IMM:
    BaseRegister = ((WORD)fetch()) | ((WORD)fetch() << 8);
    testw(BaseRegister);
    break;
  case LDZ_ABS:
    build_address_abs(&HB, &LB, &address);
    loadw(&BaseRegister, address);
    break;
  case LDZ_ZPG:
    build_address_zpg(&HB, &LB, &address);
    loadw(&BaseRegister, address);
    break;
  case LDZ_IND:
    build_address_ind(&HB, &LB, &address);
    loadw(&BaseRegister, address);
    break;
  case LDZ_PAG:
    build_address_pag(&HB, &LB, &address);
    loadw(&BaseRegister, address);
    break;
  case LDZ_BAS:
    build_address_bas(&HB, &LB, &address);
    loadw(&BaseRegister, address);
    break;

  case STZ_ABS:
    build_address_abs(&HB, &LB, &address);
    storew(BaseRegister, address);
    break;

  case STZ_ZPG:
    build_address_zpg(&HB, &LB, &address);
    storew(BaseRegister, address);
    break;
  case STZ_IND:
    build_address_ind(&HB, &LB, &address);
    storew(BaseRegister, address);
    break;
  case STZ_PAG:
    build_address_pag(&HB, &LB, &address);
    storew(BaseRegister, address);
    break;
  case STZ_BAS:
    build_address_bas(&HB, &LB, &address);
    storew(BaseRegister, address);
    break;

  case DEZ:
    if (BaseRegister > 0) {
      BaseRegister--;
      ztestw(BaseRegister);
    }
    break;

  case INZ:
    if (BaseRegister > 0) {
      BaseRegister++;
      ztestw(BaseRegister);
    }
    break;

  case DPE:
    if (PageRegister > 0) {
      PageRegister--;
      ztest(PageRegister);
    }
    break;

  case INP:
    if (PageRegister > 0) {
      PageRegister++;
      ztest(PageRegister);
    }
    break;
  }
}

// Handles MOV X,Y
//  Where X and Y are any of the registers A-F
void Group_2_Move(BYTE opcode) {

  BYTE HN = opcode >> 4;
  BYTE LN = opcode & 0xf;

  // Normalize HN and LN
  HN -= 0xA;
  LN -= 0x1;
  // Both HN and LN will appear from the range 0-5

  // The source and destination gets mapped as such
  // NIBBLE REGISTER INDEX
  //  0       A       5
  //  1       B       4
  //  2       C       0
  //  3       D       1
  //  4       E       2
  //  5       F       3
  BYTE source = HN < 0x2 ? 0x5 - HN : HN - 0x2;
  BYTE dest = LN < 0x2 ? 0x5 - LN : LN - 0x2;

  assert(dest >= 0 && dest <= 5);
  assert(source >= 0 && dest <= 5);

  Registers[dest] = Registers[source];
}

void execute(BYTE opcode) {
  // We're using bytes to store both the nibbles
  // of the opcode
  BYTE HN = opcode >> 4;
  BYTE LN = opcode & 0xF;

  // If the high nibble is between A-F and the low nibble
  // is between 1-6, it's a MOV opcode
  if (HN >= 0xA && HN <= 0xF && LN >= 0x1 && LN <= 0x6) {
    Group_2_Move(opcode);
  } else {
    Group_1(opcode);
  }
}

void emulate() {
  BYTE opcode;
  int sanity;

  ProgramCounter = 0;
  halt = false;
  memory_in_range = true;
  sanity = 0;

  printf("                    A  B  C  D  E  F  P  Z    SP\n");

  while ((!halt) && (memory_in_range) && (sanity < 200)) {
    printf("%04X ", ProgramCounter); // Print current address
    opcode = fetch();
    execute(opcode);

    printf("%s  ", opcode_mneumonics[opcode]); // Print current opcode

    printf("%02X ", Registers[REGISTER_A]);
    printf("%02X ", Registers[REGISTER_B]);
    printf("%02X ", Registers[REGISTER_C]);
    printf("%02X ", Registers[REGISTER_D]);
    printf("%02X ", Registers[REGISTER_E]);
    printf("%02X ", Registers[REGISTER_F]);
    printf("%02X ", PageRegister);
    printf("%04X ", BaseRegister);
    printf("%04X ", StackPointer); // Print Stack Pointer

    if ((Flags & FLAG_N) == FLAG_N) {
      printf("N=1 ");
    } else {
      printf("N=0 ");
    }
    if ((Flags & FLAG_Z) == FLAG_Z) {
      printf("Z=1 ");
    } else {
      printf("Z=0 ");
    }
    if ((Flags & FLAG_I) == FLAG_I) {
      printf("I=1 ");
    } else {
      printf("I=0 ");
    }
    if ((Flags & FLAG_C) == FLAG_C) {
      printf("C=1 ");
    } else {
      printf("C=0 ");
    }

    printf("\n"); // New line
    sanity++;
  }

  printf("\n"); // New line
}

////////////////////////////////////////////////////////////////////////////////
//                            Simulator/Emulator (End)                        //
////////////////////////////////////////////////////////////////////////////////

void initialise_filenames() {
  int i;

  for (i = 0; i < MAX_FILENAME_SIZE; i++) {
    hex_file[i] = '\0';
    trc_file[i] = '\0';
  }
}

int find_dot_position(char *filename) {
  int dot_position;
  int i;
  char chr;

  dot_position = 0;
  i = 0;
  chr = filename[i];

  while (chr != '\0') {
    if (chr == '.') {
      dot_position = i;
    }
    i++;
    chr = filename[i];
  }

  return (dot_position);
}

int find_end_position(char *filename) {
  int end_position;
  int i;
  char chr;

  end_position = 0;
  i = 0;
  chr = filename[i];

  while (chr != '\0') {
    end_position = i;
    i++;
    chr = filename[i];
  }

  return (end_position);
}

bool file_exists(char *filename) {
  bool exists;
  FILE *ifp;

  exists = false;

  if ((ifp = fopen(filename, "r")) != NULL) {
    exists = true;

    fclose(ifp);
  }

  return (exists);
}

void create_file(char *filename) {
  FILE *ofp;

  if ((ofp = fopen(filename, "w")) != NULL) {
    fclose(ofp);
  }
}

bool getline(FILE *fp, char *buffer) {
  bool rc;
  bool collect;
  char c;
  int i;

  rc = false;
  collect = true;

  i = 0;
  while (collect) {
    c = getc(fp);

    switch (c) {
    case EOF:
      if (i > 0) {
        rc = true;
      }
      collect = false;
      break;

    case '\n':
      if (i > 0) {
        rc = true;
        collect = false;
        buffer[i] = '\0';
      }
      break;

    default:
      buffer[i] = c;
      i++;
      break;
    }
  }

  return (rc);
}

void load_and_run(int args, _TCHAR **argv) {
  char chr;
  int ln;
  int dot_position;
  int end_position;
  long i;
  FILE *ifp;
  long address;
  long load_at;
  int code;

  // Prompt for the .hex file

  printf("\n");
  printf("Enter the hex filename (.hex): ");

  if (args == 2) {
    ln = 0;
    chr = argv[1][ln];
    while (chr != '\0') {
      if (ln < MAX_FILENAME_SIZE) {
        hex_file[ln] = chr;
        trc_file[ln] = chr;
        ln++;
      }
      chr = argv[1][ln];
    }
  } else {
    ln = 0;
    chr = '\0';
    while (chr != '\n') {
      chr = getchar();

      switch (chr) {
      case '\n':
        break;
      default:
        if (ln < MAX_FILENAME_SIZE) {
          hex_file[ln] = chr;
          trc_file[ln] = chr;
          ln++;
        }
        break;
      }
    }
  }
  // Tidy up the file names

  dot_position = find_dot_position(hex_file);
  if (dot_position == 0) {
    end_position = find_end_position(hex_file);

    hex_file[end_position + 1] = '.';
    hex_file[end_position + 2] = 'h';
    hex_file[end_position + 3] = 'e';
    hex_file[end_position + 4] = 'x';
    hex_file[end_position + 5] = '\0';
  } else {
    hex_file[dot_position + 0] = '.';
    hex_file[dot_position + 1] = 'h';
    hex_file[dot_position + 2] = 'e';
    hex_file[dot_position + 3] = 'x';
    hex_file[dot_position + 4] = '\0';
  }

  dot_position = find_dot_position(trc_file);
  if (dot_position == 0) {
    end_position = find_end_position(trc_file);

    trc_file[end_position + 1] = '.';
    trc_file[end_position + 2] = 't';
    trc_file[end_position + 3] = 'r';
    trc_file[end_position + 4] = 'c';
    trc_file[end_position + 5] = '\0';
  } else {
    trc_file[dot_position + 0] = '.';
    trc_file[dot_position + 1] = 't';
    trc_file[dot_position + 2] = 'r';
    trc_file[dot_position + 3] = 'c';
    trc_file[dot_position + 4] = '\0';
  }

  if (file_exists(hex_file)) {
    // Clear Registers and Memory

    Registers[REGISTER_A] = 0;
    Registers[REGISTER_B] = 0;
    Registers[REGISTER_C] = 0;
    Registers[REGISTER_D] = 0;
    Registers[REGISTER_E] = 0;
    Registers[REGISTER_F] = 0;
    PageRegister = 0;
    BaseRegister = 0;
    Flags = 0;
    ProgramCounter = 0;
    StackPointer = 0;

    for (i = 0; i < MEMORY_SIZE; i++) {
      Memory[i] = 0x00;
    }

    // Load hex file

    if ((ifp = fopen(hex_file, "r")) != NULL) {
      printf("Loading file...\n\n");

      load_at = 0;

      while (getline(ifp, InputBuffer)) {
        if (sscanf(InputBuffer, "L=%x", &address) == 1) {
          load_at = address;
        } else if (sscanf(InputBuffer, "%x", &code) == 1) {
          if ((load_at >= 0) && (load_at <= MEMORY_SIZE)) {
            Memory[load_at] = (BYTE)code;
          }
          load_at++;
        } else {
          printf("ERROR> Failed to load instruction: %s \n", InputBuffer);
        }
      }

      fclose(ifp);
    }

    // Emulate

    emulate();
  } else {
    printf("\n");
    printf("ERROR> Input file %s does not exist!\n", hex_file);
    printf("\n");
  }
}

void building(int args, _TCHAR **argv) {
  char buffer[1024];
  load_and_run(args, argv);
  sprintf(buffer,
          "0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%"
          "02X,0x%02X,0x%02X",
          Memory[TEST_ADDRESS_1], Memory[TEST_ADDRESS_2],
          Memory[TEST_ADDRESS_3], Memory[TEST_ADDRESS_4],
          Memory[TEST_ADDRESS_5], Memory[TEST_ADDRESS_6],
          Memory[TEST_ADDRESS_7], Memory[TEST_ADDRESS_8],
          Memory[TEST_ADDRESS_9], Memory[TEST_ADDRESS_10],
          Memory[TEST_ADDRESS_11], Memory[TEST_ADDRESS_12]);
  sendto(sock, buffer, strlen(buffer), 0, (SOCKADDR *)&server_addr,
         sizeof(SOCKADDR));
}

void test_and_mark() {
  char buffer[1024];
  bool testing_complete;
  // TODO: Remove cast
  socklen_t len = (socklen_t)sizeof(SOCKADDR);
  char chr;
  int i;
  int j;
  bool end_of_program;
  long address;
  long load_at;
  int code;
  int mark;
  int passed;

  printf("\n");
  printf("Automatic Testing and Marking\n");
  printf("\n");

  testing_complete = false;

  sprintf(buffer, "Test Student %s", STUDENT_NUMBER);
  sendto(sock, buffer, strlen(buffer), 0, (SOCKADDR *)&server_addr,
         sizeof(SOCKADDR));

  while (!testing_complete) {
    memset(buffer, '\0', sizeof(buffer));

    if (recvfrom(sock, buffer, sizeof(buffer) - 1, 0, (SOCKADDR *)&client_addr,
                 &len) != SOCKET_ERROR) {
      printf("Incoming Data: %s \n", buffer);

      // if (strcmp(buffer, "Testing complete") == 1)
      if (sscanf(buffer, "Testing complete %d", &mark) == 1) {
        testing_complete = true;
        printf("Current mark = %d\n", mark);

      } else if (sscanf(buffer, "Tests passed %d", &passed) == 1) {
        // testing_complete = true;
        printf("Passed = %d\n", passed);

      } else if (strcmp(buffer, "Error") == 0) {
        printf("ERROR> Testing abnormally terminated\n");
        testing_complete = true;
      } else {
        // Clear Registers and Memory

        Registers[REGISTER_A] = 0;
        Registers[REGISTER_B] = 0;
        Registers[REGISTER_C] = 0;
        Registers[REGISTER_D] = 0;
        Registers[REGISTER_E] = 0;
        Registers[REGISTER_F] = 0;
        PageRegister = 0;
        BaseRegister = 0;
        Flags = 0;
        ProgramCounter = 0;
        StackPointer = 0;
        for (i = 0; i < MEMORY_SIZE; i++) {
          Memory[i] = 0;
        }

        // Load hex file

        i = 0;
        j = 0;
        load_at = 0;
        end_of_program = false;
        FILE *ofp;
        fopen_s(&ofp, "branch.txt", "a");

        while (!end_of_program) {
          chr = buffer[i];
          switch (chr) {
          case '\0':
            end_of_program = true;

          case ',':
            if (sscanf(InputBuffer, "L=%x", &address) == 1) {
              load_at = address;
            } else if (sscanf(InputBuffer, "%x", &code) == 1) {
              if ((load_at >= 0) && (load_at <= MEMORY_SIZE)) {
                Memory[load_at] = (BYTE)code;
                fprintf(ofp, "%02X\n", (BYTE)code);
              }
              load_at++;
            } else {
              printf("ERROR> Failed to load instruction: %s \n", InputBuffer);
            }
            j = 0;
            break;

          default:
            InputBuffer[j] = chr;
            j++;
            break;
          }
          i++;
        }
        fclose(ofp);
        // Emulate

        if (load_at > 1) {
          emulate();
          // Send and store results
          sprintf(buffer,
                  "%02X%02X %02X%02X %02X%02X %02X%02X %02X%02X %02X%02X",
                  Memory[TEST_ADDRESS_1], Memory[TEST_ADDRESS_2],
                  Memory[TEST_ADDRESS_3], Memory[TEST_ADDRESS_4],
                  Memory[TEST_ADDRESS_5], Memory[TEST_ADDRESS_6],
                  Memory[TEST_ADDRESS_7], Memory[TEST_ADDRESS_8],
                  Memory[TEST_ADDRESS_9], Memory[TEST_ADDRESS_10],
                  Memory[TEST_ADDRESS_11], Memory[TEST_ADDRESS_12]);
          sendto(sock, buffer, strlen(buffer), 0, (SOCKADDR *)&server_addr,
                 sizeof(SOCKADDR));
        }
      }
    }
  }
}

int _tmain(int argc, _TCHAR *argv[]) {
  char chr;
  char dummy;

  printf("\n");
  printf("Microprocessor Emulator\n");
  printf("UWE Computer and Network Systems Assignment 1\n");
  printf("\n");

  initialise_filenames();

  if (WSAStartup(MAKEWORD(2, 2), &data) != 0)
    return (0);

  sock = socket(AF_INET, SOCK_DGRAM,
                IPPROTO_UDP); // Here we create our socket, which will be a UDP
                              // socket (SOCK_DGRAM).
  if (!sock) {
    // Creation failed!
  }

  memset(&server_addr, 0, sizeof(SOCKADDR_IN));
  server_addr.sin_family = AF_INET;
  server_addr.sin_addr.s_addr = inet_addr(IP_ADDRESS_SERVER);
  server_addr.sin_port = htons(PORT_SERVER);

  memset(&client_addr, 0, sizeof(SOCKADDR_IN));
  client_addr.sin_family = AF_INET;
  client_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
  client_addr.sin_port = htons(PORT_CLIENT);

  chr = '\0';
  while ((chr != 'e') && (chr != 'E')) {
    printf("\n");
    printf("Please select option\n");
    printf("L - Load and run a hex file\n");
    printf("T - Have the server test and mark your emulator\n");
    printf("E - Exit\n");
    if (argc == 2) {
      building(argc, argv);
      exit(0);
    }
    printf("Enter option: ");
    chr = getchar();
    if (chr != 0x0A) {
      dummy = getchar(); // read in the <CR>
    }
    printf("\n");

    switch (chr) {
    case 'L':
    case 'l':
      load_and_run(argc, argv);
      break;

    case 'T':
    case 't':
      test_and_mark();
      break;

    default:
      break;
    }
  }

  closesocket(sock);
  WSACleanup();

  return 0;
}

int main(int argc, char *argv[]) { return _tmain(argc, argv); }
