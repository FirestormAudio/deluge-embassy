// Physical Deluge faceplate layout — control positions in the Deluge.svg
// coordinate space (viewBox 2178 x 1482), transcribed from the native simulator
// (tools/deluge-simulator/src/renderer.rs BUTTON_POSITIONS/ENCODER_POSITIONS +
// pad geometry) and bridged to the wren id space via link.rs:
//   raw button id = button_to_id() - 144   (Led index == raw button id)
//   encoder index = encoder_to_id()         (Enc.onTurn index 0..5)
//   knob shaft-click = encoder_push_id() - 144   (a button id)
// Names are the canonical controls.rs / printed-legend labels.

export const FACE_W = 2178;
export const FACE_H = 1482;

/// OLED screen rect (SVG units): <rect x y width height/> from draw_oled.
export const OLED_RECT = { x: 1105.9, y: 274.347, w: 275.943, h: 88.191 };

/// Pad grid geometry (18 cols = 16 main + 2 sidebar; 8 rows). Per-col/row offsets
/// reproduce the sidebar gap; row 0 here is the top SVG row.
export const PAD = {
  baseX: 135.153,
  baseY: 575.641,
  size: 69.5,
  radius: 6.9,
  colOffsets: [
    0.0, 106.47, 213.0, 319.47, 425.0, 531.47, 638.0, 744.47, 851.0, 957.47, 1064.0, 1170.47,
    1276.0, 1382.47, 1489.0, 1595.47, 1764.0, 1870.47,
  ],
  rowOffsets: [-11.4482, 94.9539, 201.356, 307.758, 414.16, 520.562, 626.964, 733.366],
};

export interface ButtonSpec {
  rawId: number; // wren Buttons/Led id
  name: string;
  cx: number;
  cy: number;
  r: number;
  fn?: boolean; // an encoder-function (small) button
}

const R = 22.5;
export const BUTTONS: ButtonSpec[] = [
  // Far-right transport
  { rawId: 35, name: "PLAY", cx: 1934.6, cy: 332.13, r: R },
  { rawId: 26, name: "REC", cx: 1934.6, cy: 413.93, r: R },
  { rawId: 8, name: "SHIFT", cx: 1934.6, cy: 495.06, r: R },
  // Tempo cluster
  { rawId: 34, name: "TAP", cx: 1701.66, cy: 332.13, r: R },
  { rawId: 25, name: "SYNC", cx: 1701.66, cy: 413.93, r: R },
  { rawId: 17, name: "TRIPLET", cx: 1701.66, cy: 495.06, r: R },
  // Load/Save/Back column
  { rawId: 16, name: "BACK", cx: 1449.21, cy: 252.47, r: R },
  { rawId: 15, name: "LOAD", cx: 1449.21, cy: 332.13, r: R },
  { rawId: 33, name: "SAVE", cx: 1449.21, cy: 413.93, r: R },
  { rawId: 7, name: "LEARN", cx: 1449.21, cy: 495.06, r: R },
  // Under-screen row
  { rawId: 5, name: "SYNTH", cx: 1143.22, cy: 414.04, r: R },
  { rawId: 14, name: "KIT", cx: 1216.34, cy: 414.04, r: R },
  { rawId: 23, name: "MIDI", cx: 1289.45, cy: 414.04, r: R },
  { rawId: 32, name: "CV", cx: 1362.57, cy: 414.04, r: R },
  // Session / Clip
  { rawId: 12, name: "SESSION", cx: 856.35, cy: 413.93, r: R },
  { rawId: 21, name: "CLIP", cx: 856.35, cy: 495.74, r: R },
  { rawId: 3, name: "AFFECT", cx: 694.4, cy: 453.37, r: R },
  // Bottom misc
  { rawId: 30, name: "KEYBD", cx: 1017.07, cy: 495.74, r: R },
  { rawId: 6, name: "SCALE", cx: 1159.26, cy: 495.74, r: R },
  { rawId: 24, name: "CROSS", cx: 1289.45, cy: 495.74, r: R },
  // Encoder-function row (8 small buttons under the top encoders), left→right
  { rawId: 1, name: "1", cx: 370.13, cy: 332.13, r: R, fn: true },
  { rawId: 10, name: "2", cx: 451.13, cy: 332.13, r: R, fn: true },
  { rawId: 19, name: "3", cx: 532.45, cy: 332.13, r: R, fn: true },
  { rawId: 28, name: "4", cx: 614.06, cy: 332.13, r: R, fn: true },
  { rawId: 2, name: "5", cx: 694.02, cy: 332.13, r: R, fn: true },
  { rawId: 11, name: "6", cx: 775.33, cy: 332.13, r: R, fn: true },
  { rawId: 20, name: "7", cx: 856.35, cy: 332.13, r: R, fn: true },
  { rawId: 29, name: "8", cx: 937.66, cy: 332.13, r: R, fn: true },
];

export interface EncoderSpec {
  index: number; // Enc.onTurn index 0..5, or -1 for the non-wren Volume knob
  name: string;
  cx: number;
  cy: number;
  r: number;
  pushId: number; // shaft-click button id (-1 if none)
  gold?: boolean;
}

export const ENCODERS: EncoderSpec[] = [
  { index: 4, name: "▲▼", cx: 166.65, cy: 454.27, r: 61, pushId: 0 },
  { index: 0, name: "◀▶", cx: 370.71, cy: 210.28, r: 61, pushId: 9 },
  { index: 2, name: "GOLD", cx: 573.04, cy: 454.25, r: 61, pushId: 18, gold: true },
  { index: 3, name: "GOLD", cx: 776.04, cy: 210.26, r: 61, pushId: 27, gold: true },
  { index: 5, name: "SELECT", cx: 1036.32, cy: 330.58, r: 56, pushId: 31 },
  { index: 1, name: "TEMPO", cx: 1701.66, cy: 210.28, r: 61, pushId: 13 },
  { index: -1, name: "LEVEL", cx: 1936.03, cy: 210.26, r: 61, pushId: -1 }, // not in the wren binding
];
