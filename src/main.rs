use std::ffi::CStr;
use std::fs::File;
use std::io::Read;
use std::ops::{Index, RangeFrom};

#[derive(Debug)]
enum Region {
    NTSC,
    PAL,
}

macro_rules! debug {
    ($($arg:tt)*) => {
        if true {
            println!($($arg)*);
        }
    };
}

struct MappedMemory {
    memory: [u8; 65536],
}

impl MappedMemory {
    fn new() -> Self {
        Self { memory: [0; 65536] }
    }

    fn get(&self, index: u16) -> &u8 {
        // APU ranges
        // $4000–$4013, $4015–$4017

        let apu_ranges = [0x4000..=0x4013, 0x4015..=0x4017];
        for apu_range in apu_ranges {
            if apu_range.contains(&index) {
                panic!("APU range read: 0x{:04x}", index);
            }
        }

        &self.memory[index as usize]
    }

    fn get_range(&self, index: std::ops::Range<u16>) -> &[u8] {
        let apu_ranges = [0x4000..=0x4013, 0x4015..=0x4017];
        for apu_range in apu_ranges {
            if apu_range.contains(&index.start) || apu_range.contains(&index.end) {
                panic!("APU range read: 0x{:04x}..0x{:04x}", index.start, index.end);
            }
        }

        &self.memory[index.start as usize..index.end as usize]
    }

    fn get_range_from(&self, index: RangeFrom<u16>) -> &[u8] {
        let apu_ranges = [0x4000..=0x4013, 0x4015..=0x4017];
        for apu_range in apu_ranges {
            if apu_range.contains(&index.start) {
                panic!("APU range read: 0x{:04x}..", index.start);
            }
        }

        &self.memory[index.start as usize..]
    }

    fn set(&mut self, index: u16, value: u8) {
        let apu_ranges = [0x4000..=0x4013, 0x4015..=0x4017];
        for apu_range in apu_ranges {
            if apu_range.contains(&index) {
                panic!("APU range write: 0x{:04x} = 0x{:02x}", index, value);
            }
        }

        self.memory[index as usize] = value;
    }
}

impl Index<u16> for MappedMemory {
    type Output = u8;

    fn index(&self, index: u16) -> &Self::Output {
        self.get(index)
    }
}

impl Index<std::ops::Range<u16>> for MappedMemory {
    type Output = [u8];

    fn index(&self, index: std::ops::Range<u16>) -> &Self::Output {
        self.get_range(index)
    }
}

impl Index<RangeFrom<u16>> for MappedMemory {
    type Output = [u8];

    fn index(&self, index: RangeFrom<u16>) -> &Self::Output {
        self.get_range_from(index)
    }
}

fn main() {
    let mut nsf_rom = Vec::new();
    File::open("./mm2.nsf")
        .unwrap()
        .read_to_end(&mut nsf_rom)
        .unwrap();
    let mut i = 0;
    i += expect_bytes(&nsf_rom[i..], b"NESM\x1A");
    let version_number = nsf_rom[i];
    if ![1, 2].contains(&version_number) {
        panic!("expected version 1 or 2, got {}", version_number);
    }
    if version_number != 1 {
        panic!("only working with version 1 for now");
    }
    i += 1;

    let total_num_songs = grab_u8(&nsf_rom, &mut i);
    // note: this is one indexed
    let starting_song = grab_u8(&nsf_rom, &mut i);

    let load_addr = grab_u16(&nsf_rom, &mut i);
    let init_addr = grab_u16(&nsf_rom, &mut i);
    let play_addr = grab_u16(&nsf_rom, &mut i);

    let song_name = CStr::from_bytes_until_nul(grab_n_bytes(&nsf_rom, &mut i, 32))
        .unwrap()
        .to_str()
        .unwrap();
    let artist_name = CStr::from_bytes_until_nul(grab_n_bytes(&nsf_rom, &mut i, 32))
        .unwrap()
        .to_str()
        .unwrap();
    let copyright_holder = CStr::from_bytes_until_nul(grab_n_bytes(&nsf_rom, &mut i, 32))
        .unwrap()
        .to_str()
        .unwrap();

    let play_speed_ticks_ntsc = grab_u16(&nsf_rom, &mut i);
    let bankswitch_init_values = grab_n_bytes(&nsf_rom, &mut i, 8);
    let play_speed_ticks_pal = grab_u16(&nsf_rom, &mut i);

    let pal_ntsc_flags = grab_u8(&nsf_rom, &mut i);
    let region = match pal_ntsc_flags & 0x00 {
        0 => Region::NTSC,
        1 => Region::PAL,
        _ => unreachable!(),
    };
    let dual_pal_ntsc = pal_ntsc_flags & 0x01 != 0;
    assert_eq!(0, 0b11111100 & pal_ntsc_flags);

    assert_eq!(0x7b, i);
    let extra_sound_chip_support = grab_u8(&nsf_rom, &mut i);
    assert_eq!(
        0, extra_sound_chip_support,
        "no external sound chips support yet"
    );

    // reserved for nsf2
    i += expect_bytes(&nsf_rom[i..], &[0]);

    // 24 bit length also reserved for nsf2
    i += expect_bytes(&nsf_rom[i..], &[0, 0, 0]);

    assert_eq!(0x80, i);

    // end of header
    // here's where the CPU code starts

    dbg!(version_number);
    dbg!(total_num_songs);
    dbg!(starting_song);
    dbg!(load_addr);
    dbg!(init_addr);
    dbg!(play_addr);

    dbg!(song_name);
    dbg!(artist_name);
    dbg!(copyright_holder);

    dbg!(play_speed_ticks_ntsc);
    dbg!(bankswitch_init_values);
    dbg!(play_speed_ticks_pal);

    dbg!(region);
    dbg!(dual_pal_ntsc);

    // copy the NSF rom into memory
    // let mut memory = [0; 65536];
    let mut memory = MappedMemory::new();
    let rest_of_rom = &nsf_rom[i..];

    // start emulating CPU and doing playback
    memory.memory[load_addr as usize..(load_addr as usize) + rest_of_rom.len()]
        .copy_from_slice(&nsf_rom[i..]);
    let mut pc = init_addr as u16;
    let mut a = 0;
    let mut carry = false;
    let mut zero = false;
    let mut negative = false;
    let mut overflow = false;
    let mut x = 0;
    let mut y = 0;
    let mut sp = 0xff;

    let mut is_done_with_init = false;
    let mut is_init_just_finished = false;

    loop {
        if is_init_just_finished {
            debug!(
                "init just finished, setting pc to play_addr (0x{:04x})",
                play_addr
            );
            pc = play_addr;
            is_done_with_init = true;
            is_init_just_finished = false;
        }

        match &memory[pc..] {
            // pha: push a to stack
            &[0x48, ..] => {
                debug!("pha: push a to stack");
                push_u8(&mut memory.memory, &mut sp, a);
                pc += 1;
            }

            // pla: pull a from stack
            &[0x68, ..] => {
                debug!("pla: pull a from stack");
                a = pop_u8(&mut memory.memory, &mut sp);
                zero = a == 0;
                negative = a & (1 << 7) != 0;
                pc += 1;
            }

            // jsr: jump to subroutine
            &[0x20, lo, hi, ..] => {
                debug!("jsr: jump to subroutine (0x{:02x}{:02x})", hi, lo);
                let addr = u16::from_le_bytes([lo, hi]);
                push_u16(&mut memory.memory, &mut sp, pc + 2);
                pc = addr;
            }

            // rts: return from subroutine
            &[0x60, ..] => {
                debug!("rts: return from subroutine");
                if sp == 0xff {
                    is_init_just_finished = true;
                } else {
                    pc = pop_u16(&mut memory.memory, &mut sp) + 1;
                }
            }

            // lda: load a
            // immediate
            &[0xa9, val, ..] => {
                debug!("lda: load a (immediate) 0x{:02x}", val);
                a = val;
                pc += 2;
            }
            // zero page
            &[0xa5, addr, ..] => {
                debug!("lda: load a (zero page) 0x{:02x}", addr);
                a = memory[addr as u16];
                pc += 2;
            }
            // absolute +x
            &[0xbd, lo, hi, ..] => {
                debug!("lda: load a (absolute +x) 0x{:02x}{:02x}", hi, lo);
                let addr = u16::from_le_bytes([lo, hi]);
                a = memory[(addr + x as u16)];
                pc += 3;
            }
            // indirect +y
            &[0xb1, addr, ..] => {
                debug!("lda: load a (indirect +y) 0x{:02x}", addr);
                let addr =
                    u16::from_le_bytes([memory[addr as u16], memory[addr.wrapping_add(1) as u16]]);
                a = memory[addr + y as u16];
                pc += 2;
            }

            // sta: store a
            // absolute
            &[0x85, lo, ..] => {
                debug!("sta: store a (absolute) 0x{:02x}", lo);
                memory.set(lo as u16, a);
                pc += 2;
            }
            // absolute
            &[0x8d, lo, hi, ..] => {
                debug!("sta: store a (absolute) 0x{:02x}{:02x}", hi, lo);
                let addr = u16::from_le_bytes([lo, hi]);
                memory.set(addr, a);
                pc += 3;
            }
            // absolute +x
            &[0x9d, lo, hi, ..] => {
                debug!("sta: store a (absolute +x) 0x{:02x}{:02x}", hi, lo);
                let addr = u16::from_le_bytes([lo, hi]);
                memory.set(addr + x as u16, a);
                pc += 3;
            }

            // ldx: load x
            // zero page
            &[0xa6, addr, ..] => {
                debug!("ldx: load x (zero page) 0x{:02x}", addr);
                x = memory[addr as u16];
                pc += 2;
            }

            // stx: store x
            // zero page
            &[0x86, addr, ..] => {
                debug!("stx: store x (zero page) 0x{:02x}", addr);
                memory.set(addr as u16, x);
                pc += 2;
            }

            // tax: transfer a to x
            &[0xaa, ..] => {
                debug!("tax: transfer a to x");
                x = a;
                zero = x == 0;
                negative = x & (1 << 7) != 0;
                pc += 1;
            }

            // ldy: load y
            // immediate
            &[0xa0, val, ..] => {
                debug!("ldy: load y (immediate) 0x{:02x}", val);
                y = val;
                zero = y == 0;
                negative = y & (1 << 7) != 0;
                pc += 2;
            }

            // jmp
            // absolute
            &[0x4c, lo, hi, ..] => {
                debug!("jmp: jump to 0x{:02x}{:02x}", hi, lo);
                pc = u16::from_le_bytes([lo, hi]);
            }

            // beq: branch if equal
            &[0xf0, offset, ..] => {
                debug!("beq: branch if equal");
                if zero {
                    let offset = i8::from_le_bytes([offset]);
                    pc = pc.wrapping_add_signed(2 + offset as i16);
                } else {
                    pc += 2;
                }
            }

            // bne: branch if not equal
            &[0xd0, offset, ..] => {
                debug!("bne: branch if not equal");
                if !zero {
                    let offset = i8::from_le_bytes([offset]);
                    pc = pc.wrapping_add_signed(2 + offset as i16);
                } else {
                    pc += 2;
                }
            }

            // bcc: branch if carry clear
            &[0x90, offset, ..] => {
                debug!("bcc: branch if carry clear");
                if !carry {
                    let offset = i8::from_le_bytes([offset]);
                    pc = pc.wrapping_add_signed(2 + offset as i16);
                } else {
                    pc += 2;
                }
            }

            // bcs: branch if carry set
            &[0xb0, offset, ..] => {
                debug!("bcs: branch if carry set");
                if carry {
                    let offset = i8::from_le_bytes([offset]);
                    pc = pc.wrapping_add_signed(2 + offset as i16);
                } else {
                    pc += 2;
                }
            }

            // cmp
            // immediate
            &[0xc9, val, ..] => {
                debug!("cmp: compare (immediate) 0x{:02x}", val);
                carry = a >= val;
                zero = a == val;
                let result = a.wrapping_sub(val);
                negative = result & (1 << 7) != 0;
                pc += 2;
            }

            // cpx
            // zero page
            &[0xe4, addr, ..] => {
                debug!("cpx: compare x (zero page) 0x{:02x}", addr);
                carry = x >= memory[addr as u16];
                zero = x == memory[addr as u16];
                let result = x.wrapping_sub(memory[addr as u16]);
                negative = result & (1 << 7) != 0;
                pc += 2;
            }

            // cpy
            // immediate
            &[0xc0, val, ..] => {
                debug!("cpy: compare y (immediate) 0x{:02x}", val);
                carry = y >= val;
                zero = y == val;
                let result = y.wrapping_sub(val);
                negative = result & (1 << 7) != 0;
                pc += 2;
            }

            // dec: decrement memory
            // zero page
            &[0xc6, addr, ..] => {
                debug!("dec: decrement memory (zero page) 0x{:02x}", addr);
                memory.set(addr as u16, memory[addr as u16].wrapping_sub(1));
                zero = memory[addr as u16] == 0;
                negative = memory[addr as u16] & (1 << 7) != 0;
                pc += 2;
            }

            // dey: decrement y
            &[0x88, ..] => {
                debug!("dey: decrement y");
                y = y.wrapping_sub(1);
                zero = y == 0;
                negative = y & (1 << 7) != 0;
                pc += 1;
            }

            // inx: increment x
            &[0xe8, ..] => {
                debug!("inx: increment x");
                x = x.wrapping_add(1);
                zero = x == 0;
                negative = x & (1 << 7) != 0;
                pc += 1;
            }

            // iny: increment y
            &[0xc8, ..] => {
                debug!("iny: increment y");
                y = y.wrapping_add(1);
                zero = y == 0;
                negative = y & (1 << 7) != 0;
                pc += 1;
            }

            // adc: add with carry
            // zero page
            &[0x65, addr, ..] => {
                debug!("adc: add with carry (zero page) 0x{:02x}", addr);
                let value_to_add = memory[addr as u16] as u16;
                let result = (a as u16) + value_to_add + carry as u16;

                let a_prev = a;
                a = result as u8;
                carry = result > 0xFF;
                zero = a == 0;
                negative = a & (1 << 7) != 0;
                // "If the result's sign is different from both A's and memory's, signed overflow (or underflow) occurred."
                overflow = ((a_prev ^ a) & (a_prev ^ value_to_add as u8) & 0x80) != 0;
                pc += 2;
            }

            // lsr: logical shift right
            // accumulator
            &[0x4a, ..] => {
                debug!("lsr: logical shift right (accumulator)");
                carry = a & 1 != 0;
                a >>= 1;
                zero = a == 0;
                negative = a & (1 << 7) != 0;
                pc += 1;
            }

            // asl: arithmetic shift left
            // accumulator
            &[0x0a, ..] => {
                debug!("asl: arithmetic shift left (accumulator)");
                carry = a & (1 << 7) != 0;
                a <<= 1;
                zero = a == 0;
                negative = a & (1 << 7) != 0;
                pc += 1;
            }

            // zero page
            &[0x46, addr, ..] => {
                debug!("lsr: logical shift right (zero page) 0x{:02x}", addr);
                carry = memory[addr as u16] & 1 != 0;
                memory.set(addr as u16, memory[addr as u16] >> 1);
                zero = memory[addr as u16] == 0;
                negative = memory[addr as u16] & (1 << 7) != 0;
                pc += 2;
            }

            // and
            // immediate
            &[0x29, val, ..] => {
                debug!("and: and (immediate) 0x{:02x}", val);
                a &= val;
                zero = a == 0;
                negative = a & (1 << 7) != 0;
                pc += 2;
            }

            // ora: or with accumulator
            // zero page
            &[0x05, addr, ..] => {
                debug!("ora: or with accumulator (zero page) 0x{:02x}", addr);
                a |= memory[addr as u16];
                zero = a == 0;
                negative = a & (1 << 7) != 0;
                pc += 2;
            }

            // clc: clear carry
            &[0x18, ..] => {
                debug!("clc: clear carry");
                carry = false;
                pc += 1;
            }

            // nop: no-op
            &[0xea, ..] => {
                debug!("nop: no-op");
                pc += 1;
            }

            _ => panic!(
                "unhandled cpu code: 0x{:02x} ({:?})",
                &memory[pc as u16], &memory[pc as u16]
            ),
        }
    }
}

fn pop_u16(memory: &mut [u8], sp: &mut u8) -> u16 {
    *sp += 1;
    let lo = memory[stack_addr(*sp) as usize];
    *sp += 1;
    let hi = memory[stack_addr(*sp) as usize];
    u16::from_le_bytes([lo, hi])
}

fn push_u16(memory: &mut [u8], sp: &mut u8, val: u16) {
    let [lo, hi] = val.to_le_bytes();
    memory[stack_addr(*sp) as usize] = hi;
    *sp -= 1;
    memory[stack_addr(*sp) as usize] = lo;
    *sp -= 1;
}

fn pop_u8(memory: &mut [u8], sp: &mut u8) -> u8 {
    *sp += 1;
    memory[stack_addr(*sp) as usize]
}

fn push_u8(memory: &mut [u8], sp: &mut u8, val: u8) {
    memory[stack_addr(*sp) as usize] = val;
    *sp -= 1;
}

fn stack_addr(sp: u8) -> u16 {
    0x0100 + sp as u16
}

fn grab_u16(buf: &[u8], i: &mut usize) -> u16 {
    let val = u16::from_le_bytes([buf[*i], buf[*i + 1]]);
    *i += 2;
    val
}

fn grab_n_bytes<'a>(buf: &'a [u8], i: &mut usize, n: usize) -> &'a [u8] {
    let val = &buf[*i..*i + n];
    *i += n;
    val
}

fn grab_u8(buf: &[u8], i: &mut usize) -> u8 {
    let val = buf[*i];
    *i += 1;
    val
}

fn expect_bytes(buf: &[u8], prefix: &[u8]) -> usize {
    if buf.starts_with(prefix) {
        prefix.len()
    } else {
        panic!("expected {:?}, got {:?}", prefix, &buf[..prefix.len()]);
    }
}
