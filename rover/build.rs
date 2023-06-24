use std::fs;
use std::io::Write;
use esp_idf_part::PartitionTable;

use anyhow::Context;

// Necessary because of this issue: https://github.com/rust-lang/cargo/issues/9641
fn main() -> anyhow::Result<()> {
    embuild::build::CfgArgs::output_propagated("ESP_IDF")?;
    embuild::build::LinkArgs::output_propagated("ESP_IDF")?;
    make_flashfs_sh()
}

fn get_spiffs_size_and_address() -> Option<(u32, u32)> {
    let bin = fs::read("./partitions.csv").unwrap();
    let table = PartitionTable::try_from(bin).unwrap();
    for partition in table.partitions().iter() {
        if partition.name() == "spiffs" {
            return Some((partition.offset(), partition.size()));
        }
    }
    return None;
}

fn make_flashfs_sh() -> anyhow::Result<()> {
    let (address, size) = match get_spiffs_size_and_address() {
        Some(x) => x,
        None => {
            return Err(std::io::Error::new(
                std::io::ErrorKind::NotFound,
                "spiffs partition not found",
            )
            .into());
        }
    };

    let manifest_dir = std::env::var("CARGO_MANIFEST_DIR").context("CARGO_MANIFEST_DIR not set")?;
    let flashfs_var_path = format!("{manifest_dir}/flashfs.var");
    let mut flashfs_var = std::fs::OpenOptions::new()
        .create(true)
        .truncate(true)
        .write(true)
        .open(flashfs_var_path)?;
    writeln!(flashfs_var, "#!/bin/sh")?;
    writeln!(flashfs_var, "spiffs_size={size}")?;
    writeln!(flashfs_var, "spiffs_address=0x{address:0x}")?;
    Ok(())
}
