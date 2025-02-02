use std::ffi::CStr;
use std::fs::File;
use std::io::Read;

#[derive(Debug)]
enum Region {
    NTSC,
    PAL,
}

fn main() {
    let mut buf = Vec::new();
    File::open("./mm2.nsf")
        .unwrap()
        .read_to_end(&mut buf)
        .unwrap();
    let mut i = 0;
    i += expect_bytes(&buf[i..], b"NESM\x1A");
    let version_number = buf[i];
    if ![1, 2].contains(&version_number) {
        panic!("expected version 1 or 2, got {}", version_number);
    }
    if version_number != 1 {
        panic!("only working with version 1 for now");
    }
    i += 1;

    let total_num_songs = grab_u8(&buf, &mut i);
    // note: this is one indexed
    let starting_song = grab_u8(&buf, &mut i);

    let load_addr = grab_u16(&buf, &mut i);
    let init_addr = grab_u16(&buf, &mut i);
    let play_addr = grab_u16(&buf, &mut i);

    let song_name = CStr::from_bytes_until_nul(grab_n_bytes(&buf, &mut i, 32))
        .unwrap()
        .to_str()
        .unwrap();
    let artist_name = CStr::from_bytes_until_nul(grab_n_bytes(&buf, &mut i, 32))
        .unwrap()
        .to_str()
        .unwrap();
    let copyright_holder = CStr::from_bytes_until_nul(grab_n_bytes(&buf, &mut i, 32))
        .unwrap()
        .to_str()
        .unwrap();

    let play_speed_ticks_ntsc = grab_u16(&buf, &mut i);
    let bankswitch_init_values = grab_n_bytes(&buf, &mut i, 8);
    let play_speed_ticks_pal = grab_u16(&buf, &mut i);

    let pal_ntsc_flags = grab_u8(&buf, &mut i);
    let region = match pal_ntsc_flags & 0x00 {
        0 => Region::NTSC,
        1 => Region::PAL,
        _ => unreachable!(),
    };
    let dual_pal_ntsc = pal_ntsc_flags & 0x01 != 0;
    assert_eq!(0, 0b11111100 & pal_ntsc_flags);

    assert_eq!(0x7b, i);
    let extra_sound_chip_support = grab_u8(&buf, &mut i);
    assert_eq!(
        0, extra_sound_chip_support,
        "no external sound chips support yet"
    );

    // reserved for nsf2
    i += expect_bytes(&buf[i..], &[0]);

    // 24 bit length also reserved for nsf2
    i += expect_bytes(&buf[i..], &[0, 0, 0]);

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

    // start of cpu code
    match &buf[i..] {
        // jmp
        &[0x4c, ..] => {
            i += 1;
            let addr = grab_u16(&buf, &mut i);
            panic!("unhandled jmp to {:x}", addr);
        }
        _ => panic!("unhandled cpu code: {:?}", &buf[i]),
    }
    println!("{:x}", buf[i]);
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
