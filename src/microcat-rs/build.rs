use git2::{Cred, RemoteCallbacks};
use prost_build::Config;
use std::fs::{read_dir, remove_dir_all};
use std::path::{Path, PathBuf};
use std::{env, fs};

fn main() {
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed=.cache");
    println!("cargo:rerun-if-changed=src/serial");

    println!("Starting build...");
    let cache_dir = ".cache/microcat-avr-rs";
    let repo_url = "git@github.com:miloom/microcat-avr-rs.git";
    let proto_dir = format!("{}/proto", cache_dir);

    // Ensure the cache directory exists
    fs::create_dir_all(cache_dir).expect("Failed to create cache directory");

    remove_dir_all(Path::new(cache_dir)).unwrap();

    let mut callbacks = RemoteCallbacks::new();
    callbacks.credentials(|_url, username_from_url, _allowed_types| {
        Cred::ssh_key(
            username_from_url.unwrap(),
            None,
            Path::new(&format!("{}/.ssh/id_ed25519", env::var("HOME").unwrap())),
            None,
        )
    });

    let mut fo = git2::FetchOptions::new();
    fo.remote_callbacks(callbacks);

    let mut builder = git2::build::RepoBuilder::new();
    builder.fetch_options(fo);

    builder
        .clone(repo_url, Path::new(cache_dir))
        .expect("Failed to clone repository");

    // Ensure the protobuf output directory exists
    fs::create_dir_all("src/serial").expect("Failed to create src/serial directory");

    // Gather all the .proto files
    let proto_files = find_proto_files(Path::new(&proto_dir));

    // Compile the protobuf files
    let mut config = Config::new();
    config.type_attribute(".", "#[derive(serde::Serialize, serde::Deserialize)]");
    config.compile_well_known_types();
    config.out_dir("src/serial");
    println!("{proto_files:?}");
    config
        .compile_protos(&proto_files, &[proto_dir.clone()])
        .expect("Failed to compile protobuf files");

    // Tell cargo to re-run if the protobuf files change
    println!("cargo:rerun-if-changed={}", proto_dir);
}

/// Recursively find all .proto files in a given directory
fn find_proto_files(dir: &Path) -> Vec<PathBuf> {
    let mut proto_files = Vec::new();
    if dir.is_dir() {
        for entry in read_dir(dir).expect("Failed to read proto directory") {
            let entry = entry.expect("Failed to read directory entry");
            let path = entry.path();
            if path.is_dir() {
                // Recursively search in subdirectories
                proto_files.extend(find_proto_files(&path));
            } else if path.extension().and_then(|s| s.to_str()) == Some("proto") {
                proto_files.push(path);
            }
        }
    }
    proto_files
}
