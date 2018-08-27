
#include <stdio.h>

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
void set_flag_z(BYTE inReg) {
  BYTE reg;
  reg = inReg;

  if ((reg & 0x80) != 0) // msbit set
  {
    Flags = Flags | FLAG_N;
  } else {
    Flags = Flags & (0xFF - FLAG_N);
  }
}

void Group_1(BYTE opcode) {
  BYTE LB = 0;
  BYTE HB = 0;
  WORD address = 0;
  WORD data = 0;
  switch (opcode) {
  case 0x41: // LDAA Immidiate

    break;
  }
}

void Group_2_Move(BYTE opcode) {
  switch (opcode) {}
}
void execute(BYTE opcode) {

  if ((opcode >= 0xA1) % (opcode <= 0xF6)) {
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
  int len = sizeof(SOCKADDR);
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
