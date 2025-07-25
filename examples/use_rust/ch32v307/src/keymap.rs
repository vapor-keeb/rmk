use rmk::action::KeyAction;
use rmk::{a, k, layer, mo};

pub(crate) const COL: usize = 3;
pub(crate) const ROW: usize = 3;
pub(crate) const NUM_LAYER: usize = 1;

#[rustfmt::skip]
pub const fn get_default_keymap() -> [[[KeyAction; COL]; ROW]; NUM_LAYER] {
    [
        layer!([
            [k!(Kp7), k!(Kp8), k!(Kp9)],
            [k!(Kp4), k!(Kp5), k!(Kp6)],
            [k!(Kp1), k!(Kp2), k!(Kp3)]
        ]),
    ]
}
