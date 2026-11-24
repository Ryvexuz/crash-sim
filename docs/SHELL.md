# The shell protocol (n20-26a)

The protocol below is the STAGE7 Milestone 3 text. This task supplies the library;
the binary `--shell` branch is follow-up n20-26a-2.

**The shell protocol (n20-26a, documented in `docs/SHELL.md`).** `crash-sim --shell --data <dir> --shell-track <file> [--shell-track <file> ...] --shell-car <file> [--shell-car <file> ...] --today <YYYY-MM-DD>`. The track and car lists are exactly the repeated flags, in the order given (never scanned, never hard-coded: n20-23's rule), at least one of each, and a track's or car's id is its file stem. `--today` is required (the shell has no clock, so a test is deterministic; the front end passes the UTC date). The data folder holds `settings.txt` (`settings 1`, n20-21) and `best.save` (`save 2`, n20-22), both written canonically and atomically. One line is read from stdin per event: `up`, `down`, `select`, `back`, `pause`, `finished <ms>` or `finished none` (the race ended; the best valid lap in whole milliseconds), and `set <key> <value>` (`width`, `height`, `fullscreen`, `volume`, `units`; only on the settings screen). The shell prints one line per fact, each flushed. At start, after the `settings` and `best` lines and before it reads any event, it prints the initial `state main cursor=0`. After every event except `set` it prints the `state` line (also when the state did not change), then an `action` line if the step returned one, then what carrying the action out printed. A `set` event prints only its `settings` line, or a `warn`, and no `state` line.

```text
shell tracks=<n> cars=<n>                                           first line
settings width=<w> height=<h> fullscreen=<true|false> volume=<v> units=<kmh|mph> source=<defaults|file|unsaved>
best track=t-20261124-162643 seconds=<value> car=t-20261124-162643 date=<YYYY-MM-DD>          one per saved track, sorted by id, at start only
state main cursor=<i>  |  state track_select cursor=<i>  |  state car_select track=t-20261124-162643 cursor=<i>  |  state settings
state race track=t-20261124-162643 car=t-20261124-162643  |  state paused track=t-20261124-162643 car=t-20261124-162643 cursor=<i>  |  state results track=t-20261124-162643 car=t-20261124-162643  |  state quit
action start_race track=t-20261124-162643 track_file=<path as given> car=t-20261124-162643 car_file=<path as given>
action abort_race  |  action save_settings  |  action quit
saved settings                       after action save_settings
saved best track=t-20261124-162643 seconds=<3 decimals>      after `finished <ms>` when the lap beat the saved one (or there was none)
warn settings refused: <settings error text>     the file failed `settings::load`; defaults are in use; the file is not touched
warn save refused: <save error text>             the file failed `Save::parse`; an empty save is in use; the file is not touched
warn settings not saved: ... / warn best not saved: ...    a write was asked for while the file is in the refused state (never overwrites a refused file)
warn set refused: <message>                       a bad key, a bad value (settings::validate's text) or not on the settings screen; nothing changed
warn data folder not writable: <path>             printed once at start when a file cannot be created there; the shell still runs
```

The shell exits 0 after `action quit` or at end of stdin, and 2 on a usage error (empty stdout). A `finished <ms>` outside the race state is ignored with `warn finished ignored: not in a race`. `seconds` in `saved best` and in `best` is the integer milliseconds divided by 1000 with three decimals, made from the integer (milestone 1's rule), so the printed time, the saved time and the on-screen time are one number.

## Decisions in n20-26a-1

1. **Arguments.** `parse_args` excludes the program name. `--shell`, `--data` and
   `--today` must appear exactly once; each list flag appears at least once, with
   order preserved. Unknown flags, missing values (including a following `--`
   flag), repeated singleton flags, invalid or duplicate stems within either
   list, and absent required flags are usage errors naming the flag.
   Values cannot be empty or contain control characters or Unicode line/paragraph
   separators: paths printed as given must not inject protocol lines.
   Native `Path::file_stem` determines ids, including Windows paths on Windows;
   no asset is opened. `unknown` is not accepted for `--today`; all other dates
   are checked by parsing `save 2\nbest t 1 c <today>\n` with `Save::parse`.
   The ids in the verbatim table are illustrative, not hard-coded task ids.
2. **Startup.** The order is the `shell` line, optional folder warning, optional
   settings refusal, settings line, optional save refusal, sorted best lines,
   initial main state. The writability probe creates the data directory, uses
   `save::write_atomic` on `<data>/.shell-probe` with empty bytes, then removes
   that probe. A failure in creation, writing or removal emits the folder warning
   once. Missing data files mean defaults/an empty save without a refusal;
   other read errors and parse errors refuse that file. Read errors use the I/O
   error's Display; parser errors use the existing module's Display.
3. **Times.** `finished` accepts `none`, or 1 to 9 ASCII digits representing a
   positive integer. Thus milliseconds are bounded to 1..=999999999 by the
   protocol, keeping conversion/division finite and away from underflow.
   `saved best` uses integer quotient/remainder text with exactly three decimal
   places; the recorded value is `ms as f64 / 1000.0`. Canonical save text is
   `Save::to_text` (so `70000` prints `70.000` but saves `70`).
   Startup best lines format the saved `f64` with `{:.3}`, including legacy
   values not originating in this shell.
4. **Transitions.** All recognized non-set events go through `menu::step` using
   the list lengths. They print state even if unchanged, followed by action and
   its effects. Both `race` and `paused` accept `finished`, following the menu's
   rule; otherwise print state then `warn finished ignored: not in a race`.
   `finished none` records nothing. `Save::record_lap` determines whether a lap
   beats the old best. No improvement prints no extra line; record/write errors
   print `warn best not saved: <error>`.
5. **Writes.** Settings use `settings::write` followed by `save::write_atomic`;
   best laps use `Save::to_text` followed by the same writer. Refused files are
   never written in the session, even after successful in-memory changes:
   `warn settings not saved: settings.txt was refused at start and is left unchanged`
   or `warn best not saved: best.save was refused at start and is left unchanged`.
   Other write failures use `warn settings not saved: <error>` or
   `warn best not saved: <error>`. Only successful settings writes change source
   to `file`; a successful set changes it to `unsaved`. A failed best write keeps
   the new in-memory best; a subsequent slower lap does not retry that write.
   Concurrent sessions writing one data folder are not supported by the
   underlying atomic writer.
6. **Set.** `set` and lines starting `set ` use the set handler. Outside settings,
   the sole line is `warn set refused: not on the settings screen`. On settings,
   exactly three nonempty tokens separated by single spaces are required;
   malformed shapes give `warn set refused: expected set <key> <value>`.
   Unknown keys give `warn set refused: unknown key <key>`. Numeric values must
   have 1 to 10 ASCII digits and fit `u32`; malformed values give
   `warn set refused: <key> needs an integer, got <value>`. Boolean/unit errors
   replace `an integer` with `true or false`/`kmh or mph`. A copy is validated
   through `settings::validate` before replacing anything; its Display follows
   `warn set refused: ` on failure. Success prints only the settings line.
7. **Unknown input.** Other lines, including invalid finished values, print only
   `warn event refused: unknown event <line as {:?}>`. There is no state line.
   `run` removes one trailing LF, then one trailing CR; it does not trim spaces.
8. **End and I/O.** `run` returns 0 at EOF/read error or immediately after the
   quit event, without reading another line; 2 for usage errors with empty stdout
   and `shell: usage: <message>` on stderr; 1 for output write or flush failure.
   Every output line ends in LF and is individually flushed. `Shell::finished`
   becomes true when its returned event lines include `action quit`.
   There are no threads or shell-level `unwrap`/`expect` calls.

## Library and checks

`crash_sim::shell::{ShellArgs, parse_args, Shell, run}` provides the library API.
`Shell::start` returns initial facts; `Shell::event` returns each event's ordered
facts, without doing stream I/O. `run` connects those facts to `BufRead`/`Write`.
Pass parsed `ShellArgs` to `start` for the argument contract.

`tests/shell_core.rs` contains the seven named in-process checks. The exact
STAGE7 check 3.2 output and canonical files are pinned byte for byte. The three
specified behavioural mutations should fail respectively because a refused
file is rewritten, `70.000` becomes `70`, or the unchanged settings state line
disappears. These mutation checks must be run by the orchestrator, separately,
with restoration and a clean rerun after each.

Follow-up **n20-26a-2** adds the `src/main.rs` `--shell` branch and the seven
`tests/shell.rs` binary-level check 3.1 tests, including two processes on one
data folder, after t-20261124-161051 merges.
