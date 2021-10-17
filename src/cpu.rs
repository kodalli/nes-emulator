/* CPU Emulation */

use std::collections::HashMap;
use crate::opcodes;

bitflags! {
    /// Processor status, aka flag register, aka P
    ///  7 6 5 4 3 2 1 0
    ///  N V _ B D I Z C
    ///  | |   | | | | +--- Carry Flag
    ///  | |   | | | +----- Zero Flag
    ///  | |   | | +------- Interrupt Disable
    ///  | |   | +--------- Decimal Mode (not used on NES)
    ///  | |   +----------- Break Command
    ///  | +--------------- Overflow Flag
    ///  +----------------- Negative Flag
    ///
    pub struct CpuFlags: u8 {
        const CARRY             = 0b00000001;
        const ZERO              = 0b00000010;
        const INTERRUPT_DISABLE = 0b00000100;
        const DECIMAL_MODE      = 0b00001000;
        const BREAK             = 0b00010000;
        const BREAK2            = 0b00100000;
        const OVERFLOW          = 0b01000000;
        const NEGATIV           = 0b10000000;
    }
}

const STACK: u16 = 0x0100;
const STACK_RESET: u8 = 0xfd;

pub struct CPU {
    pub register_a: u8,
    pub register_x: u8,
    pub register_y: u8,
    pub status: CpuFlags,
    pub program_counter: u16,
    pub stack_pointer: u8,
    memory: [u8; 0xFFFF],
}

/* Memory Addressing Modes */

#[derive(Debug)]
#[allow(non_camel_case_types)]
pub enum AddressingMode {
    Immediate,
    ZeroPage,
    ZeroPage_X,
    ZeroPage_Y,
    Absolute,
    Absolute_X,
    Absolute_Y,
    Indirect_X,
    Indirect_Y,
    NoneAddressing,
}

trait Mem {
    fn mem_read(&self, address: u16) -> u8;

    fn mem_write(&mut self, address: u16, data: u8);

    fn mem_read_u16(&self, pos: u16) -> u16 {
        let lo = self.mem_read(pos) as u16;
        let hi = self.mem_read(pos + 1) as u16;
        hi << 8 | (lo as u16)
    }

    fn mem_write_16(&mut self, pos: u16, data: u16) {
        let hi = (data >> 8) as u8;
        let lo = (data & 0xff) as u8;
        self.mem_write(pos, lo);
        self.mem_write(pos + 1, hi);
    }
}

impl Mem for CPU {
    fn mem_read(&self, address: u16) -> u8 {
        self.memory[address as usize]
    }

    fn mem_write(&mut self, address: u16, data: u8) {
        self.memory[address as usize] = data;
    }
}

impl CPU {
    pub fn new() -> Self {
        CPU {
            register_a: 0,
            register_x: 0,
            register_y: 0,
            stack_pointer: STACK_RESET,
            program_counter: 0,
            status: CpuFlags::from_bits_truncate(0b100100),
            memory: [0; 0xFFFF],
        }
    }

    // Restores the state of all registers, initialize program counter by 2 byte value stored at 0xFFFC
    pub fn reset(&mut self) {
        self.register_a = 0;
        self.register_x = 0;
        self.register_y = 0;
        self.stack_pointer = STACK_RESET;
        self.status = CpuFlags::from_bits_truncate(0b100100);
        self.memory = [0; 0xFFFF];
        self.program_counter = self.mem_read_u16(0xFFFC);
    }

    // Loads program into PRG ROM and saves reference to that code in 0xFFFC
    pub fn load(&mut self, program: Vec<u8>) {
        self.memory[0x8000..(0x8000 + program.len())].copy_from_slice(&program[..]);
        self.mem_write_16(0xFFFC, 0x8000);
    }

    pub fn load_and_run(&mut self, program: Vec<u8>) {
        self.load(program);
        self.reset();
        self.run()
    }

    fn get_operand_address(&self, mode: &AddressingMode) -> u16 {
        match mode {
            AddressingMode::Immediate => self.program_counter,

            AddressingMode::ZeroPage => self.mem_read(self.program_counter) as u16,

            AddressingMode::Absolute => self.mem_read_u16(self.program_counter),

            AddressingMode::ZeroPage_X => {
                let pos = self.mem_read(self.program_counter);
                let address = pos.wrapping_add(self.register_x) as u16;
                address
            }
            AddressingMode::ZeroPage_Y => {
                let pos = self.mem_read(self.program_counter);
                let address = pos.wrapping_add(self.register_y) as u16;
                address
            }

            AddressingMode::Absolute_X => {
                let base = self.mem_read_u16(self.program_counter);
                let address = base.wrapping_add(self.register_x as u16);
                address
            }
            AddressingMode::Absolute_Y => {
                let base = self.mem_read_u16(self.program_counter);
                let address = base.wrapping_add(self.register_y as u16);
                address
            }

            AddressingMode::Indirect_X => {
                let base = self.mem_read(self.program_counter);

                let ptr: u8 = (base as u8).wrapping_add(self.register_x);
                let lo = self.mem_read(ptr as u16);
                let hi = self.mem_read(ptr.wrapping_add(1) as u16);
                (hi as u16) << 8 | (lo as u16)
            }
            AddressingMode::Indirect_Y => {
                let base = self.mem_read(self.program_counter);

                let lo = self.mem_read(base as u16);
                let hi = self.mem_read((base as u8).wrapping_add(1) as u16);
                let deref_base = (hi as u16) << 8 | (lo as u16);
                let deref = deref_base.wrapping_add(self.register_y as u16);
                deref
            }

            AddressingMode::NoneAddressing => {
                panic!("mode {:?} is not supported", mode);
            }
        }
    }

    pub fn run(&mut self) {
        let ref opcodes: HashMap<u8, &'static opcodes::OpCode> = *opcodes::OPCODES_MAP;

        loop {
            let code = self.mem_read(self.program_counter);
            self.program_counter += 1;
            let program_counter_state = self.program_counter;

            let opcode = opcodes.get(&code).expect(&format!("OpCode {:x} is not recognized", code));

            match code {

                // ADC (Add with Carry)
                0x69 | 0x65 | 0x75 | 0x6D | 0x7D | 0x79 | 0x61 | 0x71 => {
                    self.adc(&opcode.mode);
                }

                // AND (bitwise AND with accumulator)
                0x29 | 0x25 | 0x35 | 0x2D | 0x3D | 0x39 | 0x21 | 0x31 => {
                    self.and(&opcode.mode);
                }

                // ASL (Arithmetic Shift Left)
                0x06 | 0x16 | 0x0E | 0x1E => {
                    self.asl(&opcode.mode);
                }

                // ASL (Arithmetic Shift Left) with Accumulator
                0x0A => {
                    self.asl_accumulator();
                }

                // BIT (test BITs)
                0x24 | 0x2C => {
                    self.bit(&opcode.mode);
                }

                // Branch Instructions:
                // BPL (Branch on Plus)
                0x10 => {
                    self.branch(!self.status.contains(CpuFlags::NEGATIV));
                }

                // BMI (Branch on Minus)
                0x30 => {
                    self.branch(self.status.contains(CpuFlags::NEGATIV));
                }

                // BVC (Branch on Overflow Clear)
                0x50 => {
                    self.branch(!self.status.contains(CpuFlags::OVERFLOW));
                }

                // BVS (Branch on Overflow Set
                0x70 => {
                    self.branch(self.status.contains(CpuFlags::OVERFLOW));
                }

                // BCC (Branch on Carry Clear)
                0x90 => {
                    self.branch(!self.status.contains(CpuFlags::CARRY));
                }

                // BCS (Branch on Carry Set)
                0xB0 => {
                    self.branch(self.status.contains(CpuFlags::CARRY));
                }

                // BNE (Branch on Not Equals)
                0xD0 => {
                    self.branch(!self.status.contains(CpuFlags::ZERO));
                }

                // BEQ (Branch on Equal)
                0xF0 => {
                    self.branch(self.status.contains(CpuFlags::ZERO));
                }

                // BRK (0x00)
                0x00 => return,

                // CMP (Compare Accumulator)
                0xC9 | 0xC5 | 0xD5 | 0xCD | 0xDD | 0xD9 | 0xC1 | 0xD1 => {
                    self.compare(&opcode.mode, self.register_a);
                }

                // CPX (Compare X register)
                0xE0 | 0xE4 | 0xEC => {
                    self.compare(&opcode.mode, self.register_x);
                }

                // CPY (Compare Y register)
                0xC0 | 0xC4 | 0xCC => {
                    self.compare(&opcode.mode, self.register_y);
                }

                // DEC (Decrement memory)
                0xC6 | 0xD6 | 0xCE | 0xDE => {
                    self.dec(&opcode.mode);
                }

                // EOR (Bitwise Exclusive OR)
                0x49 | 0x45 | 0x55 | 0x4D | 0x5D | 0x59 | 0x41 | 0x51 => {
                    self.eor(&opcode.mode);
                }

                // FLag (Processor Status) Instructions
                // CLC (Clear Carry)
                0x18 => {
                    self.clear_carry_flag();
                }

                // SEC (Set Carry)
                0x38 => {
                    self.set_carry_flag();
                }

                // CLI (Clear Interrupt)
                0x58 => {
                    self.status.remove(CpuFlags::INTERRUPT_DISABLE);
                }

                // SEI (Set Interrupt)
                0x78 => {
                    self.status.insert(CpuFlags::INTERRUPT_DISABLE);
                }

                // CLV (Clear Overflow)
                0xB8 => {
                    self.status.remove(CpuFlags::OVERFLOW);
                }

                // CLD (Clear Decimal)
                0xD8 => {
                    self.status.remove(CpuFlags::DECIMAL_MODE);
                }

                // SED (Set Decimal)
                0xF8 => {
                    self.status.insert(CpuFlags::DECIMAL_MODE);
                }

                // INC (Increment memory)
                0xE6 | 0xF6 | 0xEE | 0xFE => {
                    self.inc(&opcode.mode);
                }

                // JMP (Jump Absolute)
                0x4C => {
                    let mem_address = self.mem_read_u16(self.program_counter);
                    self.program_counter = mem_address;
                }

                // JMP (Jump Indirect)
                0x6C => {
                    let mem_address = self.mem_read_u16(self.program_counter);

                    let indirect_ref = if mem_address & 0x00FF == 0x00FF {
                        let lo = self.mem_read(mem_address);
                        let hi = self.mem_read(mem_address & 0x00FF);
                        (hi as u16) << 8 | (lo as u16)
                    } else {
                        self.mem_read_u16(mem_address)
                    };

                    self.program_counter = indirect_ref;
                }

                // JSR (Jump to Subroutine)
                0x20 => {
                    self.stack_push_u16(self.program_counter);
                    let target_address = self.mem_read_u16(self.program_counter);
                    self.program_counter = target_address;
                }

                // LDA Load Accumulator
                0xa9 | 0xa5 | 0xb5 | 0xad | 0xbd | 0xb9 | 0xa1 | 0xb1 => {
                    self.lda(&opcode.mode);
                }

                // LDX (Load X register)
                0xA2 | 0xA6 | 0xB6 | 0xAE | 0xBE => {
                    self.ldx(&opcode.mode);
                }

                // LDY (Load Y register)
                0xA0 | 0xA4 | 0xB4 | 0xAC | 0xBC => {
                    self.ldy(&opcode.mode);
                }

                // LSR (Logical Shift Right)
                0x46 | 0x56 | 0x4E | 0x5E => {
                    self.lsr(&opcode.mode);
                }

                // LSR with Accumulator
                0x4A => {
                    self.lsr_accumulator();
                }

                // Wrap Around, Program Counter, Execution Times

                // NOP (No Operation)
                0xEA => {
                    // do nothing
                }

                // ORA (Bitwise OR with Accumulator)
                0x09 | 0x05 | 0x15 | 0x0D | 0x1D | 0x19 | 0x01 | 0x11 => {
                    self.ora(&opcode.mode);
                }

                // Register Instructions
                // TAX (0xAA) Transfer Accumulator to X, Copy value from A to X, and update status register
                0xAA => self.tax(),

                // TXA (Transfer X to A)
                0x8A => {
                    self.register_a = self.register_x;
                    self.update_zero_and_negative_flags(self.register_a);
                }

                // DEX (Decrement X)
                0xCA => self.dex(),

                // INX (Increment X)
                0xE8 => self.inx(),

                // TAY (Transfer A to Y)
                0xA8 => {
                    self.register_y = self.register_a;
                    self.update_zero_and_negative_flags(self.register_y);
                }

                // TYA (Transfer Y to A)
                0x98 => {
                    self.register_a = self.register_y;
                    self.update_zero_and_negative_flags(self.register_a);
                }

                // DEY (Decrement Y)
                0x88 => self.dey(),

                // INY (Increment Y)
                0xC8 => self.iny(),

                // ROL (Rotate Left) with Accumulator
                0x2A => {
                    self.rol_accumulator();
                }

                // ROL (Rotate Left)
                0x26 | 0x36 | 0x2E | 0x3E => {
                    self.rol(&opcode.mode);
                }

                // ROR (Rotate Right) with Accumulator
                0x6A => {
                    self.ror_accumulator();
                }

                // ROR (Rotate Right)
                0x66 | 0x76 | 0x6E | 0x7E => {
                    self.ror(&opcode.mode);
                }

                // RTI (Return from Interrupt)
                0x40 => {
                    self.status.bits = self.stack_pop();
                    self.status.remove(CpuFlags::BREAK);
                    self.status.insert(CpuFlags::BREAK2);
                    self.program_counter = self.stack_pop_u16();
                }

                // RTS (Return from subroutine)
                0x60 => {
                    self.program_counter = self.stack_pop_u16() + 1;
                }

                // SBC (Subtract with Carry)
                0xE9 | 0xE5 | 0xF5 | 0xED | 0xFD | 0xF9 | 0xE1 | 0xF1 => {
                    self.sbc(&opcode.mode);
                }

                // STA (Store Accumulator) copies the value from register A to memory
                0x85 | 0x95 | 0x8d | 0x9d | 0x99 | 0x81 | 0x91 => {
                    self.sta(&opcode.mode);
                }

                // Stack Instructions
                // TXS (Transfer X to Stack ptr)
                0x9A => {
                    self.stack_pointer = self.register_x;
                }

                // TSX (Transfer Stack ptr to X)
                0xBA => {
                    self.register_x = self.stack_pointer;
                }

                // PHA (Push Accumulator)
                0x48 => {
                    self.stack_push(self.register_a);
                }

                // PLA (Pull Accumulator)
                0x68 => {
                    self.pla();
                }

                // PHP (Push Processor status)
                0x08 => {
                    self.php();
                }

                // PLP (Pull Processor status)
                0x28 => {
                    self.plp();
                }

                // STX (Store X register)
                0x86 | 0x96 | 0x8e => {
                    let address = self.get_operand_address(&opcode.mode);
                    self.mem_write(address, self.register_x);
                }

                // STY (Store Y register)
                0x84 | 0x94 | 0x8c => {
                    let address = self.get_operand_address(&opcode.mode);
                    self.mem_write(address, self.register_y);
                }

                _ => todo!(),
            }

            if program_counter_state == self.program_counter {
                self.program_counter += (opcode.len - 1) as u16;
            }
        }
    }
    /* Helper functions */

    fn update_zero_and_negative_flags(&mut self, result: u8) {
        if result == 0 {
            self.status.insert(CpuFlags::ZERO);
        } else {
            self.status.remove(CpuFlags::ZERO);
        }

        if result & 0b1000_0000 != 0 {
            self.status.insert(CpuFlags::NEGATIV);
        } else {
            self.status.remove(CpuFlags::NEGATIV);
        }
    }

    fn set_register_a(&mut self, value: u8) {
        self.register_a = value;
        self.update_zero_and_negative_flags(self.register_a);
    }

    // Ignoring decimal mode
    fn add_to_register_a(&mut self, data: u8) {
        let sum = self.register_a as u16
            + data as u16
            + (if self.status.contains(CpuFlags::CARRY) {
            1
        } else {
            0
        }) as u16;

        let carry = sum > 0xff;

        if carry {
            self.status.insert(CpuFlags::CARRY);
        } else {
            self.status.remove(CpuFlags::CARRY);
        }

        let result = sum as u8;

        if (data ^ result) & (result ^ self.register_a) & 0x80 != 0 {
            self.status.insert(CpuFlags::OVERFLOW);
        } else {
            self.status.remove(CpuFlags::OVERFLOW);
        }

        self.set_register_a(result);
    }

    fn set_carry_flag(&mut self) {
        self.status.insert(CpuFlags::CARRY)
    }

    fn clear_carry_flag(&mut self) {
        self.status.remove(CpuFlags::CARRY)
    }

    fn stack_push(&mut self, data: u8) {
        self.mem_write((STACK as u16) + self.stack_pointer as u16, data);
        self.stack_pointer = self.stack_pointer.wrapping_sub(1);
    }

    fn stack_push_u16(&mut self, data: u16) {
        let hi = (data >> 8) as u8;
        let lo = (data & 0xff) as u8;
        self.stack_push(hi);
        self.stack_push(lo);
    }

    fn stack_pop(&mut self) -> u8 {
        self.stack_pointer = self.stack_pointer.wrapping_add(1);
        self.mem_read((STACK as u16) + self.stack_pointer as u16)
    }

    fn stack_pop_u16(&mut self) -> u16 {
        let lo = self.stack_pop() as u16;
        let hi = self.stack_pop() as u16;
        hi << 8 | lo
    }

    /* Op Codes */

    fn adc(&mut self, mode: &AddressingMode) {
        let address = self.get_operand_address(mode);
        let value = self.mem_read(address);
        self.add_to_register_a(value);
    }

    fn and(&mut self, mode: &AddressingMode) {
        let address = self.get_operand_address(mode);
        let data = self.mem_read(address);
        self.set_register_a(data & self.register_a);
    }

    fn asl(&mut self, mode: &AddressingMode) -> u8 {
        let address = self.get_operand_address(mode);
        let mut data = self.mem_read(address);
        if data >> 7 == 1 {
            self.set_carry_flag();
        } else {
            self.clear_carry_flag();
        }
        data = data << 1;
        self.mem_write(address, data);
        self.update_zero_and_negative_flags(data);
        data
    }

    fn asl_accumulator(&mut self) {
        let mut data = self.register_a;
        if data >> 7 == 1 {
            self.set_carry_flag();
        } else {
            self.clear_carry_flag();
        }
        data = data << 1;
        self.set_register_a(data)
    }

    fn bit(&mut self, mode: &AddressingMode) {
        let address = self.get_operand_address(mode);
        let data = self.mem_read(address);
        let and = self.register_a & data;
        if and == 0 {
            self.status.insert(CpuFlags::ZERO);
        } else {
            self.status.remove(CpuFlags::ZERO);
        }

        self.status.set(CpuFlags::NEGATIV, data & 0b10000000 > 0);
        self.status.set(CpuFlags::OVERFLOW, data & 0b01000000 > 0);
    }

    fn branch(&mut self, condition: bool) {
        if condition {
            let jump: i8 = self.mem_read(self.program_counter) as i8;
            let jump_address = self
                .program_counter
                .wrapping_add(1)
                .wrapping_add(jump as u16);

            self.program_counter = jump_address;
        }
    }

    fn compare(&mut self, mode: &AddressingMode, compare_with: u8) {
        let address = self.get_operand_address(mode);
        let data = self.mem_read(address);
        if data <= compare_with {
            self.status.insert(CpuFlags::CARRY);
        } else {
            self.status.remove(CpuFlags::CARRY);
        }
        self.update_zero_and_negative_flags(compare_with.wrapping_sub(data));
    }

    fn dec(&mut self, mode: &AddressingMode) -> u8 {
        let address = self.get_operand_address(&mode);
        let mut data = self.mem_read(address);
        data = data.wrapping_sub(1);
        self.mem_write(address, data);
        self.update_zero_and_negative_flags(data);
        data
    }

    fn dey(&mut self) {
        self.register_y = self.register_y.wrapping_sub(1);
        self.update_zero_and_negative_flags(self.register_y);
    }

    fn dex(&mut self) {
        self.register_x = self.register_x.wrapping_sub(1);
        self.update_zero_and_negative_flags(self.register_x);
    }

    fn eor(&mut self, mode: &AddressingMode) {
        let address = self.get_operand_address(&mode);
        let data = self.mem_read(address);
        self.set_register_a(data ^ self.register_a);
    }

    fn inc(&mut self, mode: &AddressingMode) -> u8 {
        let address = self.get_operand_address(mode);
        let mut data = self.mem_read(address);
        data = data.wrapping_add(1);
        self.mem_write(address, data);
        self.update_zero_and_negative_flags(data);
        data
    }

    fn inx(&mut self) {
        self.register_x = self.register_x.wrapping_add(1);
        self.update_zero_and_negative_flags(self.register_x);
    }

    fn iny(&mut self) {
        self.register_y = self.register_y.wrapping_add(1);
        self.update_zero_and_negative_flags(self.register_y);
    }

    fn lda(&mut self, mode: &AddressingMode) {
        let address = self.get_operand_address(&mode);
        let value = self.mem_read(address);

        self.register_a = value;
        self.update_zero_and_negative_flags(self.register_a);
    }

    fn ldx(&mut self, mode: &AddressingMode) {
        let address = self.get_operand_address(&mode);
        let value = self.mem_read(address);

        self.register_x = value;
        self.update_zero_and_negative_flags(self.register_x);
    }

    fn ldy(&mut self, mode: &AddressingMode) {
        let address = self.get_operand_address(&mode);
        let value = self.mem_read(address);

        self.register_y = value;
        self.update_zero_and_negative_flags(self.register_y);
    }

    fn lsr(&mut self, mode: &AddressingMode) -> u8 {
        let address = self.get_operand_address(&mode);
        let mut data = self.mem_read(address);
        if data & 1 == 1 {
            self.set_carry_flag();
        } else {
            self.clear_carry_flag();
        }
        data = data >> 1;
        self.mem_write(address, data);
        self.update_zero_and_negative_flags(data);
        data
    }

    fn lsr_accumulator(&mut self) {
        let mut data = self.register_a;
        if data & 1 == 1 {
            self.set_carry_flag();
        } else {
            self.clear_carry_flag();
        }
        data = data >> 1;
        self.set_register_a(data);
    }

    fn ora(&mut self, mode: &AddressingMode) {
        let address = self.get_operand_address(&mode);
        let data = self.mem_read(address);
        self.set_register_a(data | self.register_a);
    }
    fn ror(&mut self, mode: &AddressingMode) -> u8 {
        let address = self.get_operand_address(&mode);
        let mut data = self.mem_read(address);
        let old_carry = self.status.contains(CpuFlags::CARRY);

        if data & 1 == 1 {
            self.set_carry_flag();
        } else {
            self.clear_carry_flag();
        }

        data = data >> 1;
        if old_carry {
            data = data | 0b10000000;
        }

        self.mem_write(address, data);
        self.update_zero_and_negative_flags(data);
        data
    }

    fn ror_accumulator(&mut self) {
        let mut data = self.register_a;
        let old_carry = self.status.contains(CpuFlags::CARRY);

        if data & 1 == 1 {
            self.set_carry_flag();
        } else {
            self.clear_carry_flag();
        }

        data = data >> 1;
        if old_carry {
            data = data | 0b10000000;
        }

        self.set_register_a(data);
    }

    fn rol(&mut self, mode: &AddressingMode) -> u8 {
        let address = self.get_operand_address(&mode);
        let mut data = self.mem_read(address);
        let old_carry = self.status.contains(CpuFlags::CARRY);

        if data >> 7 == 1 {
            self.set_carry_flag();
        } else {
            self.clear_carry_flag();
        }

        data = data << 1;
        if old_carry {
            data = data | 1;
        }

        self.mem_write(address, data);
        self.update_zero_and_negative_flags(data);
        data
    }

    fn rol_accumulator(&mut self) {
        let mut data = self.register_a;
        let old_carry = self.status.contains(CpuFlags::CARRY);

        if data >> 7 == 1 {
            self.set_carry_flag();
        } else {
            self.clear_carry_flag();
        }

        data = data << 1;
        if old_carry {
            data = data | 1;
        }

        self.set_register_a(data);
    }

    fn pla(&mut self) {
        let data = self.stack_pop();
        self.set_register_a(data);
    }

    fn plp(&mut self) {
        self.status.bits = self.stack_pop();
        self.status.remove(CpuFlags::BREAK);
        self.status.insert(CpuFlags::BREAK2);
    }

    fn php(&mut self) {
        let mut flags = self.status.clone();
        flags.insert(CpuFlags::BREAK);
        flags.insert(CpuFlags::BREAK2);
        self.stack_push(flags.bits());
    }

    fn sbc(&mut self, mode: &AddressingMode) {
        let address = self.get_operand_address(&mode);
        let data = self.mem_read(address);
        self.add_to_register_a((data as i8).wrapping_neg().wrapping_sub(1) as u8);
    }

    fn sta(&mut self, mode: &AddressingMode) {
        let address = self.get_operand_address(mode);
        self.mem_write(address, self.register_a);
    }

    fn tax(&mut self) {
        self.register_x = self.register_a;
        self.update_zero_and_negative_flags(self.register_x);
    }
}

/* Tests */

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_0xa9_lda_immediate_load_data() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xa9, 0x05, 0x00]);
        assert_eq!(cpu.register_a, 5);
        assert_eq!(cpu.status.bits() & 0b0000_0010, 0b00);
        assert_eq!(cpu.status.bits() & 0b1000_0000, 0);
    }

    #[test]
    fn test_0xaa_tax_move_a_to_x() {
        let mut cpu = CPU::new();
        cpu.register_a = 10;
        cpu.load_and_run(vec![0xaa, 0x00]);

        assert_eq!(cpu.register_x, 10)
    }

    #[test]
    fn test_5_ops_working_together() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xa9, 0xc0, 0xaa, 0xe8, 0x00]);

        assert_eq!(cpu.register_x, 0xc1)
    }

    #[test]
    fn test_inx_overflow() {
        let mut cpu = CPU::new();
        cpu.register_x = 0xff;
        cpu.load_and_run(vec![0xe8, 0xe8, 0x00]);

        assert_eq!(cpu.register_x, 1)
    }

    #[test]
    fn test_lda_from_memory() {
        let mut cpu = CPU::new();
        cpu.mem_write(0x10, 0x55);

        cpu.load_and_run(vec![0xa5, 0x10, 0x00]);

        assert_eq!(cpu.register_a, 0x55);
    }
}