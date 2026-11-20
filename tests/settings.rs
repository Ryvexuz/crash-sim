#[path = "../src/settings.rs"]
mod settings;

use settings::{load, validate, write, Key, Settings, SettingsError, Units, ACTIONS, KEY_NAMES};

fn fails_at(text: &str, line: usize) -> SettingsError {
    let e = load(text).expect_err(text);
    assert_eq!(e.line, line, "{text:?} -> {e}");
    e
}

#[test]
fn header_only_gives_defaults_and_canonical_text() {
    let s = load("settings 1\n").unwrap();
    assert_eq!(s, Settings::default());
    assert_eq!(load("settings 1").unwrap(), s);
    assert_eq!(
        write(&s).unwrap(),
        "settings 1\nwidth = 1280\nheight = 720\nfullscreen = false\nvolume = 80\nunits = kmh\n\
         key.throttle = up\nkey.brake = down\nkey.left = left\nkey.right = right\n\
         key.pause = escape\nkey.reset = r\n"
    );
}

#[test]
fn round_trip_is_byte_identical() {
    let text =
        "settings 1\r\n# my setup\n\n  width = 1920 \nheight=1080\nfullscreen = true\nvolume = 0\n\
                units = mph\nkey.throttle = w\nkey.brake = s\nkey.left = a\nkey.right = d\n\
                key.pause = space\nkey.reset = tab\n";
    let s = load(text).unwrap();
    assert_eq!(
        (s.width, s.height, s.fullscreen, s.volume, s.units),
        (1920, 1080, true, 0, Units::Mph)
    );
    let w = write(&s).unwrap();
    assert!(!w.contains('#') && !w.contains('\r'));
    assert_eq!(load(&w).unwrap(), s);
    assert_eq!(write(&load(&w).unwrap()).unwrap(), w);
}

#[test]
fn every_value_in_range_round_trips() {
    // Every field varies at once; the six keys are six consecutive (so distinct) list entries.
    for i in 0..KEY_NAMES.len() {
        let mut s = Settings {
            width: 320 + (i as u32 * 251) % 7361,
            height: 7680 - (i as u32 * 97) % 7361,
            volume: (i as u32 * 7) % 101,
            fullscreen: i % 2 == 0,
            units: if i % 3 == 0 { Units::Mph } else { Units::Kmh },
            ..Settings::default()
        };
        for j in 0..6 {
            s.keys[j] = Key::from_name(KEY_NAMES[(i + j) % KEY_NAMES.len()]).unwrap();
        }
        let w = write(&s).unwrap();
        let back = load(&w).unwrap_or_else(|e| panic!("{w:?}: {e}"));
        assert_eq!(back, s);
        assert_eq!(write(&back).unwrap(), w);
    }
    // The extremes.
    for (w, h, v) in [(320, 320, 0), (7680, 7680, 100)] {
        let s = Settings {
            width: w,
            height: h,
            volume: v,
            ..Settings::default()
        };
        assert_eq!(load(&write(&s).unwrap()).unwrap(), s);
    }
}

#[test]
fn write_refuses_what_load_would_refuse() {
    let d = Settings::default();
    let bad = [
        (
            Settings {
                volume: 101,
                ..d.clone()
            },
            "volume",
        ),
        (
            Settings {
                volume: u32::MAX,
                ..d.clone()
            },
            "volume",
        ),
        (
            Settings {
                width: 319,
                ..d.clone()
            },
            "width",
        ),
        (
            Settings {
                width: 0,
                ..d.clone()
            },
            "width",
        ),
        (
            Settings {
                width: 7681,
                ..d.clone()
            },
            "width",
        ),
        (
            Settings {
                height: 100,
                ..d.clone()
            },
            "height",
        ),
        (
            Settings {
                height: 7681,
                ..d.clone()
            },
            "height",
        ),
    ];
    for (s, field) in bad {
        let e = write(&s).expect_err(field);
        assert!(e.0.contains(field), "{e}");
        assert_eq!(validate(&s).unwrap_err(), e);
    }
    // Two actions on one key.
    let mut clash = Settings::default();
    clash.keys[0] = clash.keys[1];
    let e = write(&clash).unwrap_err();
    assert!(
        e.0.contains("key.throttle") && e.0.contains("key.brake") && e.0.contains("both"),
        "{e}"
    );
    // The same rules through load give the same message text: one rule, not two copies.
    let e = load("settings 1\nkey.throttle = down\n").unwrap_err();
    assert_eq!(e.message, validate(&clash).unwrap_err().0);
    let e = load("settings 1\nvolume = 150\n").unwrap_err();
    assert_eq!(
        e.message,
        validate(&Settings {
            volume: 150,
            ..d.clone()
        })
        .unwrap_err()
        .0
    );
    // The defaults are valid.
    assert!(validate(&d).is_ok());
}

#[test]
fn bad_header_is_refused() {
    assert!(fails_at("", 1).message.contains("empty"));
    // The first of several problems is the one reported.
    fails_at("settings 1\nvolume = 150\nwidth 5\n", 2);
    fails_at("settings 2\n", 1);
    fails_at("width = 800\nsettings 1\n", 1);
    fails_at("# c\nsettings 1\n", 1);
}

#[test]
fn bad_syntax_unknown_and_duplicate_keys_name_their_line() {
    fails_at("settings 1\n\n# c\nwidth 800\n", 4);
    fails_at("settings 1\nspeed = 3\n", 2);
    fails_at("settings 1\nkey.jump = space\n", 2);
    fails_at("settings 1\nkey.throttle = banana\n", 2);
    fails_at("settings 1\nvolume = 5\n# x\nvolume = 6\n", 4);
    fails_at("settings 1\nkey.pause = p\nkey.pause = q\n", 3);
}

#[test]
fn out_of_range_and_malformed_values_name_their_line() {
    fails_at("settings 1\nwidth = 319\n", 2);
    fails_at("settings 1\nheight = 7681\n", 2);
    fails_at("settings 1\n\nheight = 99999999999\n", 3);
    fails_at("settings 1\nwidth = +800\n", 2);
    fails_at("settings 1\nwidth = -800\n", 2);
    fails_at("settings 1\nwidth = 8.5\n", 2);
    fails_at("settings 1\nwidth =\n", 2);
    fails_at("settings 1\nvolume = 101\n", 2);
    fails_at("settings 1\nfullscreen = yes\n", 2);
    fails_at("settings 1\nunits = knots\n", 2);
    // Bounds themselves are accepted.
    assert!(load("settings 1\nwidth = 320\nheight = 7680\nvolume = 100\n").is_ok());
}

#[test]
fn two_actions_on_one_key_are_refused_at_the_explicit_line() {
    // Both explicit: the later line.
    let e = fails_at("settings 1\nkey.brake = w\nkey.throttle = w\n", 3);
    assert!(e.message.contains("key.throttle") && e.message.contains("key.brake"));
    // One explicit, the other a default (reset defaults to r).
    fails_at("settings 1\nwidth = 800\nkey.pause = r\n", 3);
    // Swapping two defaults is fine.
    assert!(load("settings 1\nkey.throttle = down\nkey.brake = up\n").is_ok());
}

#[test]
fn every_action_name_is_settable() {
    for (i, a) in ACTIONS.iter().enumerate() {
        let s = load(&format!("settings 1\nkey.{a} = z\n")).unwrap();
        assert_eq!(s.keys[i].name(), "z");
    }
}
