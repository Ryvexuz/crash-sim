# Lockstep net messages (n20-25a)

Stage 7 m4: two machines race in lockstep; only inputs and state hashes cross the wire. This
document is the wire format only (`src/net.rs`). The session layer is a later task (n20-25b).

One message is one line of text. `encode(&Msg) -> String` returns the line without a terminator
(the transport adds `\n`); `decode(&str) -> Result<Msg, NetError>` takes the line without one.
No floats cross the wire: inputs are integers in 1/1000 units, so text round-trips exactly.

| Message | Line | Fields |
|---|---|---|
| hello | `hello 1 <seed> <tick_rate>` | `1` is the protocol version; `seed` is u64; `tick_rate` is 1..=1000 ticks per second |
| input | `input <tick> <throttle> <brake> <steer>` | `tick` is u64; brake 0..=1000; throttle and steer -1000..=1000 (a negative throttle is reverse) |
| hash | `hash <tick> <16 lower-case hex>` | `tick` is u64; the hash is a u64 as exactly 16 lower-case hex digits, zero padded |
| bye | `bye` | no fields |

Examples: `hello 1 42 60`, `input 7 1000 0 -250`, `hash 9 00000000000000ab`, `bye`.

## Strict parsing

`decode` refuses, with a `NetError`, anything that is not exactly what `encode` produces:

- empty input (`Empty`) and lines over 128 bytes (`TooLong`, checked before parsing);
- unknown words, including wrong case (`UnknownWord`);
- extra or missing fields (`WrongFieldCount`);
- a version other than 1 in `hello` (`WrongVersion`). The version stays 1: the wire was already signed decimal text, so a v1 peer from before 15499f4 refuses a reverse input with `OutOfRange` mid-session;
- values outside the ranges above, including u64 overflow and tick_rate 0 (`OutOfRange`);
- numbers that are not canonical decimal: a `+`, leading zeros, `-0`, a bare `-`, decimals, exponents,
  `0x`, underscores, non-digits (`BadNumber`); hash text that is not 16 lower-case hex digits (`BadNumber`);
- spacing other than exactly one space between fields: leading, trailing or doubled spaces, tabs,
  `\r` or `\n`, any control character, any non-ASCII byte (`BadSpacing`).

So `decode(s) == Ok(m)` implies `encode(&m) == s`: every accepted line is the one canonical text.
