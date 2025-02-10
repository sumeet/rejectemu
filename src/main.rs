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

/// Each sub-array is the 8-step waveform for a given duty setting.
///  0 -> 12.5%  (0,1,0,0,0,0,0,0)
///  1 -> 25%    (0,1,1,0,0,0,0,0)
///  2 -> 50%    (0,1,1,1,1,0,0,0)
///  3 -> 25%neg (1,0,0,1,1,1,1,1)
static DUTY_TABLE: [[u8; 8]; 4] = [
    [0, 1, 0, 0, 0, 0, 0, 0],
    [0, 1, 1, 0, 0, 0, 0, 0],
    [0, 1, 1, 1, 1, 0, 0, 0],
    [1, 0, 0, 1, 1, 1, 1, 1],
];

struct PulseChannel {
    enabled: bool,

    timer: u16,
    length_counter: u8,

    duty_index: u8,
    length_counter_disable: bool,
    envelope_disable: bool,
    volume_envelope_period: u8,
}

impl PulseChannel {
    fn new() -> Self {
        Self {
            enabled: false,
            timer: 0,
            length_counter: 0,

            duty_index: 0,
            length_counter_disable: false,
            envelope_disable: false,
            volume_envelope_period: 0,
        }
    }
}

struct TriangleChannel {
    enabled: bool,

    timer: u16,
    length_counter: u8,

    length_counter_halt: bool,
    linear_counter_reload: u8,
}

impl TriangleChannel {
    fn new() -> Self {
        Self {
            enabled: false,
            timer: 0,
            length_counter: 0,
            length_counter_halt: false,
            linear_counter_reload: 0,
        }
    }
}

static NOISE_PERIOD_TABLE_NTSC: [u16; 16] = [
    4, 8, 16, 32, 64, 96, 128, 160, 202, 254, 380, 508, 762, 1016, 2034, 4068,
];

/// These values correspond to the 5-bit length index (0..31).
/// The hardware does this lookup internally on writes to the length registers.
static LENGTH_TABLE: [u8; 32] = [
    10, 254, 20, 2, 40, 4, 80, 6, 160, 8, 60, 10, 14, 12, 26, 14, 12, 16, 28, 18, 24, 20, 48, 22,
    96, 24, 192, 26, 72, 28, 160, 30,
];

struct NoiseChannel {
    enabled: bool,

    length_counter_halt: bool,
    constant_volume: bool,
    envelope_period: u8,

    short_mode: bool,
    period_index: u8,

    length_index: u8,
}

impl NoiseChannel {
    fn new() -> Self {
        Self {
            enabled: false,
            length_counter_halt: false,
            constant_volume: false,
            envelope_period: 0,
            short_mode: false,
            period_index: 0,
            length_index: 0,
        }
    }
}

struct APU {
    pulse_channel_1: PulseChannel,
    pulse_channel_2: PulseChannel,
    triangle_channel: TriangleChannel,

    noise_channel: NoiseChannel,
    dmc_enabled: bool,
}

impl APU {
    fn new() -> Self {
        Self {
            pulse_channel_1: PulseChannel::new(),
            pulse_channel_2: PulseChannel::new(),
            triangle_channel: TriangleChannel::new(),

            // noise_enabled: false,
            noise_channel: NoiseChannel::new(),
            dmc_enabled: false,
        }
    }
}

struct MappedMemory {
    apu: APU,
    memory: [u8; 65536],
}

const BANKSWITCH_RANGES: std::ops::RangeInclusive<u16> = 0x5FF8..=0x5FFF;

impl MappedMemory {
    fn new() -> Self {
        Self {
            memory: [0; 65536],
            apu: APU::new(),
        }
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

        if BANKSWITCH_RANGES.contains(&index) {
            panic!("bankswitch range read: 0x{:04x}", index);
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

        if BANKSWITCH_RANGES.contains(&index.start) || BANKSWITCH_RANGES.contains(&index.end) {
            panic!(
                "bankswitch range read: 0x{:04x}..0x{:04x}",
                index.start, index.end
            );
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

        if BANKSWITCH_RANGES.contains(&index.start) {
            panic!("bankswitch range read: 0x{:04x}..", index.start);
        }

        &self.memory[index.start as usize..]
    }

    fn set(&mut self, index: u16, value: u8) {
        match index {
            0x4000 => {
                debug!("pulse channel 1 control: 0x{:02x}", value);
                self.apu.pulse_channel_1.duty_index = (value >> 6) & 0b11; // 0..3
                self.apu.pulse_channel_1.length_counter_disable = (value & 0b0010_0000) != 0;
                self.apu.pulse_channel_1.envelope_disable = (value & 0b0001_0000) != 0;
                self.apu.pulse_channel_1.volume_envelope_period = value & 0b0000_1111;
            }
            0x4002 => {
                debug!("pulse channel 1 timer low: 0x{:02x}", value);
                self.apu.pulse_channel_1.timer =
                    (self.apu.pulse_channel_1.timer & 0xFF00) | (value as u16);
            }
            0x4003 => {
                debug!("pulse channel 1 timer high + counter: 0x{:02x}", value);
                let timer_high_3 = (value & 0b0000_0111) as u16; // lower 3 bits
                let length_counter_5 = (value >> 3) & 0b0001_1111; // upper 5 bits

                self.apu.pulse_channel_1.timer =
                    (self.apu.pulse_channel_1.timer & 0x00FF) | (timer_high_3 << 8);

                self.apu.pulse_channel_1.length_counter = length_counter_5;
            }
            0x4004 => {
                debug!("pulse channel 2 control: 0x{:02x}", value);
                self.apu.pulse_channel_2.duty_index = (value >> 6) & 0b11; // 0..3
                self.apu.pulse_channel_2.length_counter_disable = (value & 0b0010_0000) != 0;
                self.apu.pulse_channel_2.envelope_disable = (value & 0b0001_0000) != 0;
                self.apu.pulse_channel_2.volume_envelope_period = value & 0b0000_1111;
            }
            0x4006 => {
                debug!("pulse channel 2 timer low: 0x{:02x}", value);
                self.apu.pulse_channel_2.timer =
                    (self.apu.pulse_channel_2.timer & 0xFF00) | (value as u16);
            }
            0x4007 => {
                debug!("pulse channel 2 timer high + counter: 0x{:02x}", value);
                let timer_high_3 = (value & 0b0000_0111) as u16; // lower 3 bits
                let length_counter_5 = (value >> 3) & 0b0001_1111; // upper 5 bits

                self.apu.pulse_channel_2.timer =
                    (self.apu.pulse_channel_2.timer & 0x00FF) | (timer_high_3 << 8);

                self.apu.pulse_channel_2.length_counter = length_counter_5;
            }
            0x4008 => {
                debug!("triangle channel control: 0x{:02x}", value);
                // bit 7
                self.apu.triangle_channel.length_counter_halt = (value & 0b1000_0000) != 0;
                // bits 0..6
                self.apu.triangle_channel.linear_counter_reload = value & 0b0111_1111;
            }
            0x400a => {
                debug!("triangle channel timer low: 0x{:02x}", value);
                self.apu.triangle_channel.timer =
                    (self.apu.triangle_channel.timer & 0xFF00) | (value as u16);
            }
            0x400b => {
                debug!("triangle channel timer high + counter: 0x{:02x}", value);
                let timer_high_3 = (value & 0b0000_0111) as u16; // lower 3 bits
                let length_counter_5 = (value >> 3) & 0b0001_1111; // upper 5 bits

                self.apu.triangle_channel.timer =
                    (self.apu.triangle_channel.timer & 0x00FF) | (timer_high_3 << 8);

                self.apu.triangle_channel.length_counter = length_counter_5;
            }
            0x400C => {
                self.apu.noise_channel.length_counter_halt = (value & 0x80) != 0; // bit 7
                self.apu.noise_channel.constant_volume = (value & 0x40) != 0; // bit 6
                self.apu.noise_channel.envelope_period = value & 0x0F; // bits 3..0

                // Bits 5..4 get ignored by the hardware
            }
            0x400E => {
                debug!("noise period/mode: 0x{:02x}", value);
                // bit 7
                self.apu.noise_channel.short_mode = (value & 0b1000_0000) != 0;
                // bits 3..0
                self.apu.noise_channel.period_index = value & 0b0000_1111;

                // bits 6..4 are ignored
            }
            0x400F => {
                debug!("noise channel length load: 0x{:02x}", value);

                // Bits 7..3 = length counter index
                let length_val = (value >> 3) & 0b1_1111;
                self.apu.noise_channel.length_index = length_val;

                // Bits 2..0 are unused for noise
                // If you implement the envelope fully, you also reset the envelope here.
            }
            0x4015 => {
                debug!("APU status: 0x{:02x}", value);
                self.apu.pulse_channel_1.enabled = (value & 0b0000_0001) != 0;
                self.apu.pulse_channel_2.enabled = (value & 0b0000_0010) != 0;
                self.apu.triangle_channel.enabled = (value & 0b0000_0100) != 0;
                self.apu.noise_channel.enabled = (value & 0b0000_1000) != 0;
                self.apu.dmc_enabled = (value & 0b0001_0000) != 0;
                // bits 5..=7 are unused
            }
            _ => {
                let apu_ranges = [0x4000..=0x4013, 0x4015..=0x4017];
                for apu_range in apu_ranges {
                    if apu_range.contains(&index) {
                        panic!("APU range write: 0x{:04x} = 0x{:02x}", index, value);
                    }
                }
                if BANKSWITCH_RANGES.contains(&index) {
                    panic!("bankswitch range write: 0x{:04x} = 0x{:02x}", index, value);
                }
                self.memory[index as usize] = value;
            }
        }
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
            // absolute
            &[0xad, lo, hi, ..] => {
                debug!("lda: load a (absolute) 0x{:02x}{:02x}", hi, lo);
                let addr = u16::from_le_bytes([lo, hi]);
                a = memory[addr];
                pc += 3;
            }
            // absolute +x
            &[0xbd, lo, hi, ..] => {
                debug!("lda: load a (absolute +x) 0x{:02x}{:02x}", hi, lo);
                let addr = u16::from_le_bytes([lo, hi]);
                a = memory[addr + x as u16];
                pc += 3;
            }
            // absolute +y
            &[0xb9, lo, hi, ..] => {
                debug!("lda: load a (absolute +y) 0x{:02x}{:02x}", hi, lo);
                let addr = u16::from_le_bytes([lo, hi]);
                a = memory[addr + y as u16];
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
            // indirect +x
            &[0xa1, addr, ..] => {
                debug!("lda: load a (indirect +x) 0x{:02x}", addr);
                let addr =
                    u16::from_le_bytes([memory[addr as u16], memory[addr.wrapping_add(1) as u16]]);
                a = memory[addr + x as u16];
                pc += 2;
            }

            // sta: store a
            // zero page
            &[0x85, lo, ..] => {
                debug!("sta: store a (zero page) 0x{:02x}", lo);
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
                debug!("sta: store a (absolute +x) 0x{hi:02x}{lo:02x} + {x:02x}");
                let addr = u16::from_le_bytes([lo, hi]);
                memory.set(addr + x as u16, a);
                pc += 3;
            }
            // indirect +y
            &[0x91, addr, ..] => {
                debug!("sta: store a (indirect +y) 0x{:02x}", addr);
                let addr =
                    u16::from_le_bytes([memory[addr as u16], memory[addr.wrapping_add(1) as u16]]);
                memory.set(addr + y as u16, a);
                pc += 2;
            }

            // ldx: load x
            // immediate
            &[0xa2, val, ..] => {
                debug!("ldx: load x (immediate) 0x{:02x}", val);
                x = val;
                zero = x == 0;
                negative = x & (1 << 7) != 0;
                pc += 2;
            }
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

            // txa: transfer x to a
            &[0x8a, ..] => {
                debug!("txa: transfer x to a");
                a = x;
                zero = a == 0;
                negative = a & (1 << 7) != 0;
                pc += 1;
            }

            // tay: transfer a to y
            &[0xa8, ..] => {
                debug!("tay: transfer a to y");
                y = a;
                zero = y == 0;
                negative = y & (1 << 7) != 0;
                pc += 1;
            }

            // tya: transfer y to a
            &[0x98, ..] => {
                debug!("tya: transfer y to a");
                a = y;
                zero = a == 0;
                negative = a & (1 << 7) != 0;
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
            // zero page
            &[0xa4, addr, ..] => {
                debug!("ldy: load y (zero page) 0x{:02x}", addr);
                y = memory[addr as u16];
                pc += 2;
            }

            // sty: store y
            // zero page
            &[0x84, addr, ..] => {
                debug!("sty: store y (zero page) 0x{:02x}", addr);
                memory.set(addr as u16, y);
                pc += 2;
            }

            // jmp
            // absolute
            &[0x4c, lo, hi, ..] => {
                debug!("jmp: jump to 0x{:02x}{:02x}", hi, lo);
                pc = u16::from_le_bytes([lo, hi]);
            }
            // indirect
            &[0x6c, lo, hi, ..] => {
                debug!("jmp: jump to 0x{:02x}{:02x} (indirect)", hi, lo);
                let addr = u16::from_le_bytes([lo, hi]);
                let low_byte = *memory.get(addr);
                // Emulate CPU bug:
                // If the low byte of the address is 0xFF, the high byte is read from the beginning of the same page.
                let high_addr = if addr & 0x00FF == 0x00FF {
                    addr & 0xFF00 // wraps around to the same page
                } else {
                    addr + 1
                };
                let high_byte = *memory.get(high_addr);
                pc = u16::from_le_bytes([low_byte, high_byte]);
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

            // bpl: branch if plus (relative)
            &[0x10, offset, ..] => {
                debug!("bpl: branch if plus (relative)");
                if !negative {
                    let offset = i8::from_le_bytes([offset]);
                    pc = pc.wrapping_add_signed(2 + offset as i16);
                } else {
                    pc += 2;
                }
            }

            // bmi: branch if minus (relative)
            &[0x30, offset, ..] => {
                debug!("bmi: branch if minus (relative)");
                if negative {
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
            // zero page
            &[0xc5, addr, ..] => {
                debug!("cmp: compare (zero page) 0x{:02x}", addr);
                carry = a >= memory[addr as u16];
                zero = a == memory[addr as u16];
                let result = a.wrapping_sub(memory[addr as u16]);
                negative = result & (1 << 7) != 0;
                pc += 2;
            }
            // indirect +y
            &[0xd1, addr, ..] => {
                debug!("cmp: compare (indirect +y) 0x{:02x}", addr);
                let addr =
                    u16::from_le_bytes([memory[addr as u16], memory[addr.wrapping_add(1) as u16]]);
                carry = a >= memory[addr + y as u16];
                zero = a == memory[addr + y as u16];
                let result = a.wrapping_sub(memory[addr + y as u16]);
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
            // zero page
            &[0xc4, addr, ..] => {
                debug!("cpy: compare y (zero page) 0x{:02x}", addr);
                carry = y >= memory[addr as u16];
                zero = y == memory[addr as u16];
                let result = y.wrapping_sub(memory[addr as u16]);
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

            // dex: decrement x
            &[0xca, ..] => {
                debug!("dex: decrement x");
                x = x.wrapping_sub(1);
                zero = x == 0;
                negative = x & (1 << 7) != 0;
                pc += 1;
            }

            // inc: increment memory
            // zero page
            &[0xe6, addr, ..] => {
                debug!("inc: increment memory (zero page) 0x{:02x}", addr);
                memory.set(addr as u16, memory[addr as u16].wrapping_add(1));
                zero = memory[addr as u16] == 0;
                negative = memory[addr as u16] & (1 << 7) != 0;
                pc += 2;
            }
            // absolute
            &[0xee, lo, hi, ..] => {
                debug!("inc: increment memory (absolute) 0x{:02x}{:02x}", hi, lo);
                let addr = u16::from_le_bytes([lo, hi]);
                memory.set(addr, memory[addr].wrapping_add(1));
                zero = memory[addr] == 0;
                negative = memory[addr] & (1 << 7) != 0;
                pc += 3;
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
            // immediate
            &[0x69, val, ..] => {
                debug!("adc: add with carry (immediate) 0x{:02x}", val);
                let value_to_add = val as u16;
                let result = (a as u16)
                    .wrapping_add(value_to_add)
                    .wrapping_add(carry as u16);

                let a_prev = a;
                a = result as u8;
                carry = result > 0xFF;
                zero = a == 0;
                negative = a & (1 << 7) != 0;
                // "If the result's sign is different from both A's and memory's, signed overflow (or underflow) occurred."
                overflow = ((a_prev ^ a) & ((value_to_add as u8) ^ a) & 0x80) != 0;
                pc += 2;
            }
            // zero page
            &[0x65, addr, ..] => {
                debug!("adc: add with carry (zero page) 0x{:02x}", addr);
                let value_to_add = memory[addr as u16] as u16;
                let result = (a as u16)
                    .wrapping_add(value_to_add)
                    .wrapping_add(carry as u16);

                let a_prev = a;
                a = result as u8;
                carry = result > 0xFF;
                zero = a == 0;
                negative = a & (1 << 7) != 0;
                // "If the result's sign is different from both A's and memory's, signed overflow (or underflow) occurred."
                overflow = ((a_prev ^ a) & ((value_to_add as u8) ^ a) & 0x80) != 0;
                pc += 2;
            }
            // indirect +y
            &[0x71, addr, ..] => {
                debug!("adc: add with carry (indirect +y) 0x{:02x}", addr);
                let addr =
                    u16::from_le_bytes([memory[addr as u16], memory[addr.wrapping_add(1) as u16]]);
                let value_to_add = memory[addr + y as u16] as u16;
                let result = (a as u16)
                    .wrapping_add(value_to_add)
                    .wrapping_add(carry as u16);

                let a_prev = a;
                a = result as u8;
                carry = result > 0xFF;
                zero = a == 0;
                negative = a & (1 << 7) != 0;
                // "If the result's sign is different from both A's and memory's, signed overflow (or underflow) occurred."
                overflow = ((a_prev ^ a) & ((value_to_add as u8) ^ a) & 0x80) != 0;
                pc += 2;
            }
            // absolute
            &[0x6d, lo, hi, ..] => {
                debug!("adc: add with carry (absolute) 0x{:02x}{:02x}", hi, lo);
                let addr = u16::from_le_bytes([lo, hi]);
                let value_to_add = memory[addr] as u16;
                let result = (a as u16) + value_to_add + carry as u16;

                let a_prev = a;
                a = result as u8;
                carry = result > 0xFF;
                zero = a == 0;
                negative = a & (1 << 7) != 0;
                // "If the result's sign is different from both A's and memory's, signed overflow (or underflow) occurred."
                overflow = ((a_prev ^ a) & ((value_to_add as u8) ^ a) & 0x80) != 0;
                pc += 3;
            }

            // sbc: subtract with carry
            // immediate
            &[0xe9, val, ..] => {
                debug!("sbc: subtract with carry (immediate) 0x{:02x}", val);

                let a_prev = a;

                // Perform the actual subtraction in 16 bits
                let result = (a as u16)
                    .wrapping_sub(val as u16)
                    .wrapping_sub(!carry as u16);

                // Store the 8-bit result
                a = result as u8;

                // Carry is set if the result didn't require a borrow
                carry = result <= 0xFF;

                // Standard flags
                zero = a == 0;
                negative = (a & 0x80) != 0;

                // --- Correct overflow calculation ---
                // Even though we "subtracted," internally the CPU does A + (~val) + carry.
                // So the effective 8-bit operand is:
                let operand_8 = (!val).wrapping_add(carry as u8);
                // Now use the standard 6502 overflow formula:
                overflow = ((a_prev ^ a) & (operand_8 ^ a) & 0x80) != 0;

                pc += 2;
            }
            // zero page
            &[0xe5, addr, ..] => {
                debug!("sbc: subtract with carry (zero page) 0x{:02x}", addr);

                let a_prev = a;

                // Perform the actual subtraction in 16 bits
                let result = (a as u16)
                    .wrapping_sub(memory[addr as u16] as u16)
                    .wrapping_sub(!carry as u16);

                // Store the 8-bit result
                a = result as u8;

                // Carry is set if the result didn't require a borrow
                carry = result <= 0xFF;

                // Standard flags
                zero = a == 0;
                negative = (a & 0x80) != 0;

                // --- Correct overflow calculation ---
                // Even though we "subtracted," internally the CPU does A + (~val) + carry.
                // So the effective 8-bit operand is:
                let operand_8 = (!memory[addr as u16]).wrapping_add(carry as u8);
                // Now use the standard 6502 overflow formula:
                overflow = ((a_prev ^ a) & (operand_8 ^ a) & 0x80) != 0;

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
            // zero page
            &[0x46, addr, ..] => {
                debug!("lsr: logical shift right (zero page) 0x{:02x}", addr);
                carry = memory[addr as u16] & 1 != 0;
                memory.set(addr as u16, memory[addr as u16] >> 1);
                zero = memory[addr as u16] == 0;
                negative = memory[addr as u16] & (1 << 7) != 0;
                pc += 2;
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

            // rol: rotate left
            // ROL shifts a memory value or the accumulator to the left, moving the value of each bit into the next bit and treating the carry flag as though it is both above bit 7 and below bit 0. Specifically, the value in carry is shifted into bit 0, and bit 7 is shifted into carry. Rotating left 9 times simply returns the value and carry back to their original state.
            // accumulator
            &[0x2a, ..] => {
                debug!("rol: rotate left (accumulator)");
                let carry_bit = carry as u8;
                carry = a & (1 << 7) != 0;
                a = (a << 1) | carry_bit;
                zero = a == 0;
                negative = a & (1 << 7) != 0;
                pc += 1;
            }

            // ror: rotate right
            // ROR shifts a memory value or the accumulator to the right, moving the value of each bit into the next bit and treating the carry flag as though it is both above bit 7 and below bit 0. Specifically, the value in carry is shifted into bit 7, and bit 0 is shifted into carry. Rotating right 9 times simply returns the value and carry back to their original state.
            // accumulator
            &[0x6a, ..] => {
                debug!("ror: rotate right (accumulator)");
                let carry_bit = carry as u8;
                carry = a & 1 != 0;
                a = (a >> 1) | (carry_bit << 7);
                zero = a == 0;
                negative = a & (1 << 7) != 0;
                pc += 1;
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
            // immediate
            &[0x09, val, ..] => {
                debug!("ora: or with accumulator (immediate) 0x{:02x}", val);
                a |= val;
                zero = a == 0;
                negative = a & (1 << 7) != 0;
                pc += 2;
            }
            // zero page
            &[0x05, addr, ..] => {
                debug!("ora: or with accumulator (zero page) 0x{:02x}", addr);
                a |= memory[addr as u16];
                zero = a == 0;
                negative = a & (1 << 7) != 0;
                pc += 2;
            }
            // indirect +y
            &[0x11, addr, ..] => {
                debug!("ora: or with accumulator (indirect +y) 0x{:02x}", addr);
                let addr =
                    u16::from_le_bytes([memory[addr as u16], memory[addr.wrapping_add(1) as u16]]);
                a |= memory[addr + y as u16];
                zero = a == 0;
                negative = a & (1 << 7) != 0;
                pc += 2;
            }

            // sec: set carry
            &[0x38, ..] => {
                debug!("sec: set carry");
                carry = true;
                pc += 1;
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
