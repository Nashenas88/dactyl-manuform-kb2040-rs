use keyberon::action;
use keyberon::action::Action;
// use keyberon::action::{k, l, HoldTapConfig};
use keyberon::key_code::KeyCode;
use keyberon::key_code::KeyCode::*;

#[allow(dead_code)]
const CT_T: Action = action::m(&[KeyCode::LCtrl, KeyCode::Tab]);
#[allow(dead_code)]
const SC_T: Action = action::m(&[KeyCode::LShift, KeyCode::LCtrl, KeyCode::Tab]);
#[allow(dead_code)]
const CA_D: Action = action::m(&[LCtrl, LAlt, Delete]);
#[allow(dead_code)]
const AL_T: Action = action::m(&[LAlt, Tab]);

const COLEMAK: Action = action::d(0);
const QWERTY: Action = action::d(1);

pub static LAYERS: keyberon::layout::Layers = keyberon::layout::layout! {
    { // (0) Colemak Dh
        [F1     F2     F3     F4     F5     F6      F7       F8     F9     F10   F11   F12]
        ['`'    1      2      3      4      5       6        7      8      9     0     -  ]
        [Tab    Q      W      F      P      G       J        L      U      Y     ;     =  ]
        [LShift A      R      S      T      D       H        N      E      I     O     ;  ]
        [LAlt   Z      X      C      V      B       K        M      ,      .     Quote /  ]
        [n      n      Space  BSpace t      t       '{'      '}'    Delete Enter n     n  ]
        [n      n      t      (3)    (2)    n       {QWERTY} {AL_T} t      t     n     n  ]
    }
    { // (1) Qwerty
        [F1     F2     F3     F4     F5     F6           F7  F8     F9     F10   F11  F12 ]
        ['`'    1      2      3      4      5            6   7      8      9     0    -   ]
        [Tab    Q      W      E      R      T            Y   U      I      O     P    =   ]
        [LShift A      S      D      F      G            H   J      K      L     ;   Quote]
        [LAlt   Z      X      C      V      B            N   M      ,      .     /   '}'  ]
        [n      n      Space  BSpace t      t            '{' '}'    Delete Enter n    n   ]
        [n      n      t      (3)    (2)    {COLEMAK}    n   {AL_T} t      t     n    n   ]
    }
    { // (2) Numpad
        [t t t t t t      t t    t       t          t       t]
        [t t t t t t      t t    KpSlash KpAsterisk t       t]
        [t t t t t t      t Kp7  Kp8     Kp9        KpMinus t]
        [t t t t t t      t Kp4  Kp5     Kp6        KpPlus  t]
        [t t t t t t      t Kp1  Kp2     Kp3        KpEnter t]
        [n n t t t t      t t    Kp0     KpDot      n       n]
        [n n t t t t      t t    t       t          n       n]
    }
    { // (3) Arrows
        [t t t t t t      t t    t      t      t t]
        [t t t t t t      t t    t      t      t t]
        [t t t t t t      t PgUp Up     PgDown t t]
        [t t t t t t      t Left Down   Right  t t]
        [t t t t t t      t Home Insert End    t t]
        [n n t t t t      t t    t      t      n n]
        [n n t t t t      t t    t      t      n n]
    }
    { // (4)
        [t t t t t t      t t t t t t]
        [t t t t t t      t t t t t t]
        [t t t t t t      t t t t t t]
        [t t t t t t      t t t t t t]
        [t t t t t t      t t t t t t]
        [n n t t t t      t t t t n n]
        [n n t t t t      t t t t n n]
    }
};
