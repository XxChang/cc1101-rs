# cc1101-rs
**WIP**

# How to use examples
The two examples use [bluepill development Board](https://components101.com/microcontrollers/stm32f103c8t8-blue-pill-development-board), but this crate is not restricted to specific boards due to the usage of [`embedded-hal`](https://github.com/rust-embedded/embedded-hal). Maybe you can change the `--target` flags and adapt the example to other platform.

The following is the connection between pins:

```
PA4 --> CSn
PA5 --> SCLK
PA6 --> SO
PA7 --> SI
PA8 --> GDO0
```

### For TX
```
cargo embed --release --example bluepill_tx.rs
```
The 
### For RX
```
cargo embed --release --example bluepill_rx.rs
```
