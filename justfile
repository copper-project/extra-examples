default: pr-check

pr-check:
	just fmt-check
	just check
	just test

fmt:
	cargo fmt --all

fmt-check:
	cargo fmt --all -- --check

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

# Host-side tests only. The embedded demo is compile-checked separately and the
# balancebot resim/logreader paths pull in local-only or Python-linked flows.
test:
	cargo test -p cu-human-pose
	cargo test -p cu-flight-controller --bin quad-sim --features textlogs
	cargo test -p cu-rp-balancebot --bin balancebot-sim
