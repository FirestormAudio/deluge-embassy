//! Golden wire-vector generator + drift guard. The committed JSON at
//! `tools/deluge-web-sim/src/protocol-golden.json` is the single source the TS
//! codec is pinned against. Regenerate with:
//!   UPDATE_GOLDEN=1 cargo test -p deluge-protocol --features std --test golden
#![cfg(feature = "std")]

use deluge_protocol::{FromDeluge, ToDeluge, cdc_button_id};
use serde_json::json;
use std::path::PathBuf;

/// Body bytes = full frame minus the 2-byte length prefix (WS transport form).
fn body(frame: Vec<u8>) -> Vec<u8> {
    frame[2..].to_vec()
}

fn vectors() -> serde_json::Value {
    let mut display = vec![0u8; 768];
    for (i, b) in display.iter_mut().enumerate() {
        *b = (i % 251) as u8;
    }
    let mut pads = vec![0u8; 432];
    for (i, b) in pads.iter_mut().enumerate() {
        *b = (i % 253) as u8;
    }

    let to: Vec<(&str, Vec<u8>)> = vec![
        ("UpdateDisplay", body(ToDeluge::UpdateDisplay(&display).to_frame())),
        ("ClearDisplay", body(ToDeluge::ClearDisplay.to_frame())),
        ("SetPadRgb", body(ToDeluge::SetPadRgb { col: 3, row: 4, rgb: [10, 20, 30] }.to_frame())),
        ("ClearAllPads", body(ToDeluge::ClearAllPads.to_frame())),
        ("SetLed", body(ToDeluge::SetLed { index: 35, on: true }.to_frame())),
        ("SetCv", body(ToDeluge::SetCv { channel: 1, value: 0x1234 }.to_frame())),
        ("SetGate", body(ToDeluge::SetGate { channel: 2, on: true }.to_frame())),
        ("SetAllPads", body(ToDeluge::SetAllPads(&pads).to_frame())),
        ("SetKnobIndicator", body(ToDeluge::SetKnobIndicator { which: 2, levels: [1, 2, 3, 4] }.to_frame())),
        ("SetSyncedLed", body(ToDeluge::SetSyncedLed(true).to_frame())),
        ("ClearAllLeds", body(ToDeluge::ClearAllLeds.to_frame())),
        ("SetBrightness", body(ToDeluge::SetBrightness(200).to_frame())),
        ("GetVersion", body(ToDeluge::GetVersion.to_frame())),
        ("Ping", body(ToDeluge::Ping.to_frame())),
    ];
    let from: Vec<(&str, Vec<u8>)> = vec![
        ("PadPressed", body(FromDeluge::PadPressed { col: 5, row: 2 }.to_frame())),
        ("PadReleased", body(FromDeluge::PadReleased { col: 5, row: 2 }.to_frame())),
        ("ButtonPressed", body(FromDeluge::ButtonPressed { id: cdc_button_id(0) }.to_frame())),
        ("ButtonReleased", body(FromDeluge::ButtonReleased { id: cdc_button_id(35) }.to_frame())),
        ("EncoderRotated", body(FromDeluge::EncoderRotated { id: 4, delta: -1 }.to_frame())),
        ("Version", body(FromDeluge::Version { major: 1, minor: 0, patch: 0 }.to_frame())),
        ("Pong", body(FromDeluge::Pong.to_frame())),
        ("Ready", body(FromDeluge::Ready.to_frame())),
    ];

    let mut arr = Vec::new();
    for (name, b) in to {
        arr.push(json!({ "name": name, "dir": "to", "body": b }));
    }
    for (name, b) in from {
        arr.push(json!({ "name": name, "dir": "from", "body": b }));
    }
    serde_json::Value::Array(arr)
}

fn golden_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../tools/deluge-web-sim/src/protocol-golden.json")
}

#[test]
fn golden_vectors_up_to_date() {
    let generated = serde_json::to_string_pretty(&vectors()).unwrap() + "\n";
    let path = golden_path();
    if std::env::var("UPDATE_GOLDEN").is_ok() {
        std::fs::write(&path, &generated).unwrap();
        return;
    }
    let committed = std::fs::read_to_string(&path)
        .expect("protocol-golden.json missing — run with UPDATE_GOLDEN=1 to create it");
    assert_eq!(
        committed, generated,
        "protocol-golden.json is stale — regenerate: UPDATE_GOLDEN=1 cargo test -p deluge-protocol --features std --test golden"
    );
}
