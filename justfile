default: check

fmt:
	cargo fmt --all

check: check-host check-embedded

check-host:
	cargo check -p cu-rp-balancebot --all-targets
	cargo check -p cu-flight-controller --all-targets --features textlogs
	cargo check -p cu-human-pose --all-targets
	cargo check -p cu-feetech-demo --all-targets
	cargo check -p cu-gnss-ublox-demo --all-targets --features logexport

check-embedded:
	cargo check -p cu-elrs-bdshot-demo --target thumbv8m.main-none-eabihf
	cargo check -p cu-flight-controller --target thumbv7em-none-eabihf --no-default-features --features firmware,textlogs --bin quad
