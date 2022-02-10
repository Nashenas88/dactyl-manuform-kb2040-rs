use keyberon::action;
use keyberon::action::Action;

const COLEMAK: Action = action::d(0);
const QWERTY: Action = action::d(1);
const NUM_PAD: Action = action::l(2);

pub static LAYERS: keyberon::layout::Layers = keyberon::layout::layout! {
    { // (0) Colemak Dh
        [F1     F2 F3   F4     F5        F6        F7       F8      F9     F10   F11 F12   ]
        ['`'    1  2    3      4         5         6        7       8      9     0   =     ]
        [Tab    Q  W    F      P         B         J        L       U      Y     ;   -     ]
        [LShift A  R    S      T         G         M        N       E      I     O   Quote ]
        [LCtrl  Z  X    C      D         V         K        H       ,      .     /   Bslash]
        [n      n  LGui LAlt   BSpace    Space     Enter    Delete '['    ']'    n   n     ]
        [n      n  t    Escape {NUM_PAD} n         {QWERTY} RAlt    RShift RCtrl n   n     ]
    }
    { // (1) Qwerty
        [F1     F2 F3   F4     F5        F6           F7    F8      F9     F10   F11 F12   ]
        ['`'    1  2    3      4         5            6     7       8      9     0   =     ]
        [Tab    Q  W    E      R         T            Y     U       I      O     P   -     ]
        [LShift A  S    D      F         G            H     J       K      L     ;   Quote ]
        [LCtrl  Z  X    C      V         B            N     M       ,      .     /   Bslash]
        [n      n  LGui LAlt   BSpace    Space        Enter Delete '['    ']'    n   n     ]
        [n      n  t    Escape {NUM_PAD} {COLEMAK}    n     RAlt    RShift RCtrl n   n     ]
    }
    { // (2) NumpadNav
        [t t t    t      t      t                     t t    t       t          t       t]
        [t t t    t      t      t                     t t    KpSlash KpAsterisk t       t]
        [t t PgUp Up     PgDown t                     t Kp7  Kp8     Kp9        KpMinus t]
        [t t Left Down   Right  t                     t Kp4  Kp5     Kp6        KpPlus  t]
        [t t Home Insert End    t                     t Kp1  Kp2     Kp3        KpEnter t]
        [n n t    t      t      t                     t t    Kp0     KpDot      n       n]
        [n n t    t      t      t                     t t    t       t          n       n]
    }
    { // (3) ??
        [t t t t t t      t t t t t t]
        [t t t t t t      t t t t t t]
        [t t t t t t      t t t t t t]
        [t t t t t t      t t t t t t]
        [t t t t t t      t t t t t t]
        [n n t t t t      t t t t n n]
        [n n t t t t      t t t t n n]
    }
};
