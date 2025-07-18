cargo build --release --no-default-features

cargo bloat --release --no-default-features --crates --split-std --no-relative-size -n 20

cargo size --release -- -A