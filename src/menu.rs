//! Game menus as a pure state machine (Stage 7 m3, n20-23). No drawing, no I/O, std only.
//!
//! The renderer draws whatever `State` says; the caller feeds it `Event`s and carries out the
//! returned `Action`. The track and car lists are passed in on every step (never hard-coded),
//! and the state only holds indices into them. Indices that no longer fit a list (it shrank
//! since) never start a race: the step is a no-op or falls back, it never panics.
//!
//! Back always makes progress toward `Main`: TrackSelect and Settings go to Main, CarSelect to
//! TrackSelect, Results to Main, Paused resumes the race, and Race abandons it to Main (with
//! `Action::AbortRace`). So repeated Back from any state reaches Main (or Quit), and Main stays.

/// Number of items on the main menu: Play, Settings, Quit.
pub const MAIN_ITEMS: usize = 3;
/// Number of items on the pause menu: Resume, Quit to menu.
pub const PAUSE_ITEMS: usize = 2;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum State {
    Main { cursor: usize },
    TrackSelect { cursor: usize },
    CarSelect { track: usize, cursor: usize },
    Settings,
    Race { track: usize, car: usize },
    Paused { track: usize, car: usize, cursor: usize },
    Results { track: usize, car: usize },
    Quit,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Event {
    Up,
    Down,
    Select,
    Back,
    Pause,
    RaceFinished,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Action {
    /// Start a race on tracks[track] with cars[car]. Always in range of the lists passed in.
    StartRace { track: usize, car: usize },
    /// The race was left before it finished (Race+Back, or Quit to menu from Paused).
    AbortRace,
    SaveSettings,
    Quit,
}

pub fn start() -> State {
    State::Main { cursor: 0 }
}

fn move_cursor(cursor: usize, len: usize, event: Event) -> usize {
    if len == 0 {
        return 0;
    }
    let cursor = cursor.min(len - 1);
    match event {
        Event::Up => (cursor + len - 1) % len,
        Event::Down => (cursor + 1) % len,
        _ => cursor,
    }
}

/// One transition. `track_count` and `car_count` are the lengths of the lists the caller shows.
pub fn step(
    state: State,
    event: Event,
    track_count: usize,
    car_count: usize,
) -> (State, Option<Action>) {
    use Event::*;
    match state {
        State::Main { cursor } => match event {
            Up | Down => (State::Main { cursor: move_cursor(cursor, MAIN_ITEMS, event) }, None),
            Select => match cursor {
                0 => (State::TrackSelect { cursor: 0 }, None),
                1 => (State::Settings, None),
                _ => (State::Quit, Some(Action::Quit)),
            },
            _ => (state, None),
        },
        State::TrackSelect { cursor } => match event {
            Up | Down => (State::TrackSelect { cursor: move_cursor(cursor, track_count, event) }, None),
            // An empty track list (or a stale cursor) cannot go on to start a race.
            Select if cursor < track_count => (State::CarSelect { track: cursor, cursor: 0 }, None),
            Back => (State::Main { cursor: 0 }, None),
            _ => (state, None),
        },
        State::CarSelect { track, cursor } => match event {
            Up | Down => {
                (State::CarSelect { track, cursor: move_cursor(cursor, car_count, event) }, None)
            }
            Select if cursor < car_count && track < track_count => (
                State::Race { track, car: cursor },
                Some(Action::StartRace { track, car: cursor }),
            ),
            Back => (State::TrackSelect { cursor: track }, None),
            _ => (state, None),
        },
        State::Settings => match event {
            Select => (State::Settings, Some(Action::SaveSettings)),
            Back => (State::Main { cursor: 1 }, None),
            _ => (state, None),
        },
        State::Race { track, car } => match event {
            Pause => (State::Paused { track, car, cursor: 0 }, None),
            RaceFinished => (State::Results { track, car }, None),
            Back => (State::Main { cursor: 0 }, Some(Action::AbortRace)),
            _ => (state, None),
        },
        State::Paused { track, car, cursor } => match event {
            Up | Down => (State::Paused { track, car, cursor: move_cursor(cursor, PAUSE_ITEMS, event) }, None),
            Pause | Back => (State::Race { track, car }, None),
            Select => {
                if cursor == 0 {
                    (State::Race { track, car }, None)
                } else {
                    (State::Main { cursor: 0 }, Some(Action::AbortRace))
                }
            }
            RaceFinished => (State::Results { track, car }, None),
        },
        State::Results { track, car } => match event {
            // Select is a rematch, only if the same track and car are still in the lists.
            Select if track < track_count && car < car_count => (
                State::Race { track, car },
                Some(Action::StartRace { track, car }),
            ),
            Back => (State::Main { cursor: 0 }, None),
            _ => (state, None),
        },
        State::Quit => (State::Quit, None),
    }
}
