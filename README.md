# `lsm303c`

> no_std driver for the lsm303c (accelerometer + magnetometer/compass).

[![Build Status](https://travis-ci.org/copterust/lsm303c.svg?branch=master)](https://travis-ci.org/copterust/lsm303c)

## What works

- To be tested

## Supported chips

* `LSM303C`;


## Basic usage

Include [library](https://crates.io/crates/lsm303c) as a dependency in your Cargo.toml
[![crates.io](http://meritbadge.herokuapp.com/lsm303c?style=flat-square)](https://crates.io/crates/lsm303c):

```
[dependencies.lsm303c]
version = "<version>"
```

Use embedded-hal implementation to get I2C handle and delay then create lsm303c handle:

```rust
extern crate lsm303c; // or just use lsm303c; if 2018 edition is used.

// to create sensor with default configuration:
let mut lsm = LSM303::default(l2c, &mut delay)?;
// to get all supported measurements:
let all = marg.all()?;
println!("{:?}", all);
```

## More examples

Number of examples can be found in [proving-ground](https://github.com/copterust/proving-ground) repo.

## Documentation

API Docs available on [docs.rs](https://docs.rs/lsm303c).

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
  http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

## Testimonials

Started off as a fork of [japaric's lsm303ldhc repo](https://github.com/japaric/lsm303dlhc).
