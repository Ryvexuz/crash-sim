//! Menu state machine tests (n20-23). The module is included by path; lib.rs is not touched.
#![allow(dead_code)]

#[path = "../src/menu.rs"]
mod menu;

use menu::{step, Action, Event, State};
use std::collections::{HashSet, VecDeque};

const EVENTS: [Event; 6] = [
    Event::Up,
    Event::Down,
    Event::Select,
    Event::Back,
    Event::Pause,
    Event::RaceFinished,
];

/// Every state shape for lists of the given lengths (plus out-of-range indices).
fn all_states(t: usize, c: usize) -> Vec<State> {
    let mut v = vec![State::Settings, State::Quit];
    for cursor in 0..menu::MAIN_ITEMS {
        v.push(State::Main { cursor });
    }
    for cursor in 0..t + 2 {
        v.push(State::TrackSelect { cursor });
    }
    for track in 0..t + 2 {
        for cursor in 0..c + 2 {
            v.push(State::CarSelect { track, cursor });
        }
    }
    for track in 0..t + 2 {
        for car in 0..c + 2 {
            v.push(State::Race { track, car });
            v.push(State::Results { track, car });
            for cursor in 0..menu::PAUSE_ITEMS {
                v.push(State::Paused { track, car, cursor });
            }
        }
    }
    v
}

#[test]
fn back_from_every_state_reaches_main_or_quit() {
    for (t, c) in [(0, 0), (0, 3), (3, 0), (3, 3)] {
        for s in all_states(t, c) {
            let mut cur = s;
            let mut ok = false;
            for _ in 0..10 {
                if matches!(cur, State::Main { .. } | State::Quit) {
                    ok = true;
                    break;
                }
                cur = step(cur, Event::Back, t, c).0;
            }
            assert!(ok, "Back from {:?} (t={}, c={}) never reached Main/Quit, ended {:?}", s, t, c, cur);
        }
    }
}

#[test]
fn every_state_is_reachable_from_main() {
    let (t, c) = (2, 2);
    let mut seen: HashSet<State> = HashSet::new();
    let mut q = VecDeque::from([menu::start()]);
    seen.insert(menu::start());
    while let Some(s) = q.pop_front() {
        for e in EVENTS {
            let n = step(s, e, t, c).0;
            if seen.insert(n) {
                q.push_back(n);
            }
        }
    }
    let kinds: HashSet<&str> = seen
        .iter()
        .map(|s| match s {
            State::Main { .. } => "Main",
            State::TrackSelect { .. } => "TrackSelect",
            State::CarSelect { .. } => "CarSelect",
            State::Settings => "Settings",
            State::Race { .. } => "Race",
            State::Paused { .. } => "Paused",
            State::Results { .. } => "Results",
            State::Quit => "Quit",
        })
        .collect();
    for k in ["Main", "TrackSelect", "CarSelect", "Settings", "Race", "Paused", "Results", "Quit"] {
        assert!(kinds.contains(k), "{} unreachable from Main; reached {:?}", k, kinds);
    }
}

#[test]
fn paused_back_returns_to_the_race() {
    let race = State::Race { track: 1, car: 2 };
    let (paused, _) = step(race, Event::Pause, 3, 3);
    assert!(matches!(paused, State::Paused { track: 1, car: 2, .. }));
    assert_eq!(step(paused, Event::Back, 3, 3), (race, None));
    assert_eq!(step(paused, Event::Pause, 3, 3), (race, None));
}

#[test]
fn empty_track_list_cannot_start_a_race() {
    // Walk every path from Main with no tracks; no step may ever produce StartRace or Race.
    for cars in [0, 3] {
        let mut seen = HashSet::from([menu::start()]);
        let mut q = VecDeque::from([menu::start()]);
        while let Some(s) = q.pop_front() {
            assert!(!matches!(s, State::Race { .. } | State::CarSelect { .. }), "reached {:?}", s);
            for e in EVENTS {
                let (n, a) = step(s, e, 0, cars);
                assert!(!matches!(a, Some(Action::StartRace { .. })));
                if seen.insert(n) {
                    q.push_back(n);
                }
            }
        }
    }
}

#[test]
fn empty_car_list_cannot_start_a_race() {
    let (s, _) = step(State::TrackSelect { cursor: 0 }, Event::Select, 3, 0);
    assert!(matches!(s, State::CarSelect { .. }));
    let (s2, a) = step(s, Event::Select, 3, 0);
    assert_eq!((s2, a), (s, None));
}

#[test]
fn happy_path_starts_the_chosen_race() {
    let mut s = menu::start();
    s = step(s, Event::Select, 3, 3).0; // Play
    s = step(s, Event::Down, 3, 3).0; // track 1
    s = step(s, Event::Select, 3, 3).0;
    s = step(s, Event::Up, 3, 3).0; // car wraps to 2
    let (s, a) = step(s, Event::Select, 3, 3);
    assert_eq!(a, Some(Action::StartRace { track: 1, car: 2 }));
    assert_eq!(s, State::Race { track: 1, car: 2 });
    let (s, _) = step(s, Event::RaceFinished, 3, 3);
    assert_eq!(s, State::Results { track: 1, car: 2 });
    let (s, a) = step(s, Event::Select, 3, 3);
    assert_eq!((s, a), (State::Race { track: 1, car: 2 }, Some(Action::StartRace { track: 1, car: 2 })));
}

#[test]
fn quit_settings_and_abort_actions() {
    let (s, a) = step(State::Main { cursor: 2 }, Event::Select, 1, 1);
    assert_eq!((s, a), (State::Quit, Some(Action::Quit)));
    assert_eq!(step(State::Quit, Event::Select, 1, 1), (State::Quit, None));
    assert_eq!(step(State::Settings, Event::Select, 1, 1), (State::Settings, Some(Action::SaveSettings)));
    let paused = State::Paused { track: 0, car: 0, cursor: 1 };
    assert_eq!(step(paused, Event::Select, 1, 1), (State::Main { cursor: 0 }, Some(Action::AbortRace)));
    assert_eq!(step(State::Race { track: 0, car: 0 }, Event::Back, 1, 1).1, Some(Action::AbortRace));
}

#[test]
fn lists_shrinking_never_panics_or_starts_a_stale_race() {
    for s in all_states(3, 3) {
        for e in EVENTS {
            let (_, a) = step(s, e, 1, 1);
            if let Some(Action::StartRace { track, car }) = a {
                assert!(track < 1 && car < 1, "{:?} {:?} started {} {}", s, e, track, car);
            }
        }
    }
}
