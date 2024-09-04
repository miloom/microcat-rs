CACHE_DIR := ".cache/microcat-avr"
REPO_URL := "https://github.com/miloom/microcat-avr.git"

set ignore-comments

# Generate protobuf code from files
generate:
    mkdir -p .cache
    @if [ -d "{{CACHE_DIR}}" ]; then \
      echo "Resetting the existing repository..."; \
      git -C {{CACHE_DIR}} fetch; \
      git -C {{CACHE_DIR}} reset --hard origin/master; \
    else \
      echo "Cloning the repository..."; \
      git clone {{REPO_URL}} {{CACHE_DIR}}; \
    fi
    mkdir -p src/serial
    protoc --prost_out=src/serial --prost_opt=enable_type_names=true,compile_well_known_types=true -I .cache/microcat-avr/proto .cache/microcat-avr/proto/*.proto


build:
    just generate
    cargo build
