#[path = "../src/settings.rs"]
mod settings;

use settings::{load, write, Key, Settings, SettingsError, Units, ACTIONS, KEY_NAMES};

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
        write(&s),
        "settings 1\nwidth = 1280\nheight = 720\nfullscreen = false\nvolume = 80\nunits = kmh\n\
         key.throttle = up\nkey.brake = down\nkey.left = left\nkey.right = right\n\
         key.pause = escape\nkey.reset = r\n"
    );
}

#[test]
fn round_trip_is_byte_identical() {
    let text = "settings 1\r\n# my setup\n\n  width = 1920 \nheight=1080\nfullscreen = true\nvolume = 0\n\
                units = mph\nkey.throttle = w\nkey.brake = s\nkey.left = a\nkey.right = d\n\
                key.pause = space\nkey.reset = tab\n";
    let s = load(text).unwrap();
    assert_eq!((s.width, s.height, s.fullscreen, s.volume, s.units), (1920, 1080, true, 0, Units::Mph));
    let w = write(&s);
    assert!(!w.contains('#') && !w.contains('\r'));
    assert_eq!(load(&w).unwrap(), s);
    assert_eq!(write(&load(&w).unwrap()), w);
}

#[test]
fn every_value_in_range_round_trips() {
    for (i, name) in KEY_NAMES.iter().enumerate().take(30) {
        let mut s = Settings::default();
        s.width = 320 + (i as u32 * 251) % 7361;
        s.height = 7680 - (i as u32 * 97) % 7361;
        s.volume = (i as u32 * 7) % 101;
        s.fullscreen = i % 2 == 0;
        s.units = if i % 3 == 0 { Units::Mph } else { Units::Kmh };
        s.keys[i % 6] = Key::from_name(name).unwrap();
        // Skip draws that put two actions on one key (those are refused, tested below).
        let w = write(&s);
        match load(&w) {
            Ok(back) => {
                assert_eq!(back, s);
                assert_eq!(write(&back), w);
            }
            Err(e) => assert!(e.message.contains("both"), "{e}"),
        }
    }
}

#[test]
fn bad_header_is_refused() {
    fails_at("", 1);
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
