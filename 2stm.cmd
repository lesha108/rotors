cargo build --release
cargo objcopy --bin rotors --target thumbv7em-none-eabihf --release -- -O binary rotors.bin
cargo embed --release --chip STM32F411CEUx

