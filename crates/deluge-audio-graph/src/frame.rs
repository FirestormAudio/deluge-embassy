//! One stereo output sample. The engine renders blocks of these.

#[derive(Clone, Copy, Default, PartialEq, Debug)]
pub struct StereoFrame {
    pub l: f32,
    pub r: f32,
}
