use git2::Repository;
use prost_build::Config;
use std::fs;
use std::fs::read_dir;
use std::path::{Path, PathBuf};

fn main() {
    let cache_dir = ".cache/microcat-avr";
    let repo_url = "https://github.com/miloom/microcat-avr.git";
    let proto_dir = format!("{}/proto", cache_dir);

    // Ensure the cache directory exists
    fs::create_dir_all(cache_dir).expect("Failed to create cache directory");

    // Clone or update the Git repository
    if Path::new(&(cache_dir.to_owned() + "/.git")).exists() {
        println!("Resetting the existing repository...");
        let repo = Repository::open(cache_dir).expect("Failed to open repository");
        let mut remote = repo.find_remote("origin").expect("Remote 'origin' not found");
        remote
            .fetch(&["refs/heads/master:refs/remotes/origin/master"], None, None)
            .expect("Failed to fetch updates");
        let obj = repo
            .refname_to_id("refs/remotes/origin/master")
            .expect("Failed to find ref");
        let commit = repo.find_commit(obj).expect("Failed to find commit");
        repo.reset(&commit.as_object(), git2::ResetType::Hard, None)
            .expect("Failed to reset repository");
    } else {
        println!("Cloning the repository...");
        Repository::clone(repo_url, cache_dir).expect("Failed to clone repository");
    }

    // Ensure the protobuf output directory exists
    fs::create_dir_all("src/serial").expect("Failed to create src/serial directory");

    // Gather all the .proto files
    let proto_files = find_proto_files(&Path::new(&proto_dir));

    // Compile the protobuf files
    let mut config = Config::new();
    config.type_attribute(".", "#[derive(serde::Serialize, serde::Deserialize)]"); // Optional: Add Serde attributes if needed
    config.compile_well_known_types();
    config.out_dir("src/serial");

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
