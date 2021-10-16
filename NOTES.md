# NES Notes

- CPU can access the Memory Map and CPU Registers.
- Continuous 1 byte cell array, uses 16-bit addressing, 65536 memory cells.
- NES has 2 KiB RAM.

### Memory allocation
- [0x0000 .. 0x2000] RAM accessible
- [0x4020 .. 0x6000] Hardware modules: PPU, APU, GamePads etc.
- [0x6000 .. 0x8000] RAM space on cartridge, things like game state
- [0x8000 … 0x10000] Mapped to Program ROM, PRG ROM, on cartridge

### CPU Registers
- Program Counter (PC), holds address for next machine language instruction to be executed
- Stack Pointer (SP), Memory space [0x0100 .. 0x1FF], use for stack
  - holds address of the top of the space
  - grows from top to bottom
  - when byte gets pushed to stack, SP register decrements
  - when byte retrieved from stack, SP register increments
- Accumulator (AC), stores results of math, logic, and memory access operations
  - used as input parameter for some operations
- Index Register X (X), used as an offset in specific memory addressing modes
  - can be used for auxiliary storage (holds temp values, used as counter, etc.)
- Index Register Y (Y), similar to register X
- Processor status (P), 8-bit register represents 7 status flags that can be set or unset depending on result of last executed instruction
  - for example, Z flag is set (1) if result of operation is 0 and unset/erased (0) otherwise

[6502 instructions](http://www.6502.org/tutorials/6502opcodes.html)

### 6502
- supports 6 types of commands
- 64 unique commands
- some instructions have multiple versions for different memory addressing modes so 150 machine code operations
- 150 machine code operations to implement

### NES is custom 2A03 based on 6502
- 110 unofficial additional opcodes, 1/3 are No-OPs
- audio processing unit on-board
- no support for decimal mode for arithmetic
- need to implement 256 different machine instructions
- several instructions are similar so can reuse them

#### Von Neumann Architecture
- data and instructions stored in memory
- executed coed is data from CPU, any data can be interpreted as executable code
- CPU can't tell difference
- CPU only has program counter register keeping track of position in the instructions stream.

#### Memory Addressing Modes
- CPU only has 2 KiB memory and rest of 64 KiB is for memory mapping.
- load code into 0x8000 address
  - [0x8000 .. 0xFFFF] reserved for Program ROM
  - doesn't have to start exactly on 0x8000
- NES CPU can address 65536 memory cells
  - 2 bytes to store an address
  - uses Little-Endian addressing
    - 8 least significant bits of an address stored before the 8 most significant bits