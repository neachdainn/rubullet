use anyhow::Result;
use std::fs::File;
use std::io::{BufRead, BufReader, Read};
use std::path::PathBuf;
use structopt::StructOpt;

/// A tool to view logging files created by RuBullet, PyBullet or Bullet3.
#[derive(StructOpt, Debug)]
#[structopt(name = "dump_log")]
struct Arguments {
    /// Logging file
    #[structopt(parse(from_os_str))]
    file: PathBuf,
}
fn calc_size<'a>(fmt: &str) -> usize {
    let mut size = 0;
    for c in fmt.chars() {
        size += match c {
            'I' | 'i' | 'f' => 4,
            'B' => 1,
            _ => {
                panic!("can not determine data type")
            }
        };
    }
    size
}
fn main() -> Result<()> {
    let args: Arguments = Arguments::from_args();
    let file = File::open(args.file.clone())?;
    println!("Opened");
    println!("{:?}", args.file);
    let mut reader = BufReader::new(file);
    let mut key_buf = String::new();
    reader
        .read_line(&mut key_buf)
        .expect("error while reading file");
    let keys: Vec<&str> = key_buf.strip_suffix("\n").unwrap().split(",").collect();
    let mut fmt_buf = String::new();
    reader
        .read_line(&mut fmt_buf)
        .expect("error while reading file");
    let fmt = fmt_buf.strip_suffix("\n").unwrap();
    let sz = calc_size(fmt);
    let ncols = fmt.len();

    let verbose = true;
    if verbose {
        println!("Keys:");
        println!("{:?}", keys);
        println!("Format:");
        println!("{}", fmt);
        println!("Size:");
        println!("{:?}", sz);
        println!("Columns:");
        println!("{}", ncols);
    }
    let mut chunk_index = 0;
    loop {
        let mut check_buf = [0_u8; 2];
        match reader.read_exact(&mut check_buf) {
            Ok(_) => {}
            Err(_) => {
                return Ok(());
            }
        }
        assert_eq!(
            &check_buf,
            &[170_u8, 187_u8],
            "Error, expected aabb terminal"
        );
        println!("chunk # {}", chunk_index);
        chunk_index += 1;
        for (index, data_type) in fmt.chars().enumerate() {
            match data_type {
                'I' => {
                    let mut integer_buf = [0_u8; 4];
                    reader.read_exact(&mut integer_buf).unwrap();
                    let integer = u32::from_le_bytes(integer_buf);
                    if verbose {
                        println!("    {} = {}", keys[index], integer);
                    }
                }
                'i' => {
                    let mut integer_buf = [0_u8; 4];
                    reader.read_exact(&mut integer_buf).unwrap();
                    let integer = i32::from_le_bytes(integer_buf);
                    if verbose {
                        println!("    {} = {}", keys[index], integer);
                    }
                }
                'f' => {
                    let mut float_buf = [0_u8; 4];
                    reader.read_exact(&mut float_buf).unwrap();
                    let float = f32::from_le_bytes(float_buf);
                    if verbose {
                        println!("    {} = {}", keys[index], float);
                    }
                }
                'B' => {
                    let mut char_buf = [0_u8; 1];
                    reader.read_exact(&mut char_buf).unwrap();
                    let char = i8::from_le_bytes(char_buf);
                    if verbose {
                        println!("    {} = {}", keys[index], char);
                    }
                }

                _ => {
                    panic!("can not determine data type")
                }
            }
        }
    }
}
