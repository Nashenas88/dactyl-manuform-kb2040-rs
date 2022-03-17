use keyberon::action::{self, d, l};

pub enum CustomAction {
    BootReset,
    // QwertyLedMode,
    // ColemakLedMode,
    // SymbolLedMode,
}

pub(crate) type Action = action::Action<CustomAction>;

const UF2: Action = action::Action::Custom(CustomAction::BootReset);

// Colemak DH
const COLEMAK: Action = d(0);
const QWERTY: Action = d(1);
const LAYER: Action = l(2);
const NUM_PAD: Action = d(3);
const NAV: Action = d(4);
const SYM: Action = l(5);
const MMO: Action = d(6);

pub(crate) static LAYERS: keyberon::layout::Layers<CustomAction> = keyberon::layout::layout! {
    { // (0) Colemak Dh
        [F1       F2 F3    F4    F5     F6        F7     F8      F9     F10   F11 F12   ]
        [CapsLock 1  2     3     4      5         6      7       8      9     0   =     ]
        [Escape   Q  W     F     P      B         J      L       U      Y     ;   -     ]
        [Tab      A  R     S     T      G         M      N       E      I     O   Quote ]
        [LShift   Z  X     C     D      V         K      H       ,      .     /   RShift]
        [n        n  LCtrl LAlt  BSpace Space     Enter  Delete  RAlt   RCtrl n   n     ]
        [n        n  t    {SYM}  LGui  {LAYER}   {LAYER} RGui   {SYM}   t     n   n     ]
    }
    { // (1) Qwerty
        [F1       F2 F3    F4    F5     F6        F7      F8      F9     F10   F11 F12   ]
        [CapsLock 1  2     3     4      5         6       7       8      9     0   =     ]
        [Escape   Q  W     E     R      T         Y       U       I      O     P   -     ]
        [Tab      A  S     D     F      G         H       J       K      L     ;   Quote ]
        [LShift   Z  X     C     V      B         N       M       ,      .     /   RShift]
        [n        n  LCtrl LAlt  BSpace Space     Enter   Delete  RAlt   RCtrl n   n     ]
        [n        n  t    {SYM}  LGui  {LAYER}   {LAYER}  RGui   {SYM}   t     n   n     ]
    }
    { // (2) Left Layer Selector
        [ n        n     n      n         n        n        n  n         n         n      n     n      ]
        [ n        n     n      n         n        n        n  n         n         n      n     n      ]
        [ n        n     n      n         n        n        n  n         n         n      n     n      ]
        [{QWERTY} {MMO} {NAV}  {NUM_PAD} {COLEMAK} n        n {COLEMAK} {NUM_PAD} {NAV}  {MMO} {QWERTY}]
        [ n        n     n      n         n        n        n  n         n         n      n     n      ]
        [ n        n     n      n         n        n        n  n         n         n      n     n      ]
        [ n        n     n      n         n        n        n  n         n         n      n     n      ]
    }
    { // (3) Numpad
        [t      t t     t     t      t         t       t       t       t          t       t     ]
        [t      t t     t     t      t         t       NumLock KpSlash KpAsterisk t       t     ]
        [t      t t     t     t      t         t       Kp7     Kp8     Kp9        KpMinus t     ]
        [t      t t     t     t      t         t       Kp4     Kp5     Kp6        KpPlus  t     ]
        [LShift t t     t     t      t         t       Kp1     Kp2     Kp3        KpEnter RShift]
        [n      n LCtrl LAlt  BSpace Space     KpEnter Kp0     Kp0     KpDot      n       n     ]
        [n      n t    {SYM}  LGui  {LAYER}   {LAYER}  RGui   {SYM}    t          n       n     ]
    }
    { // (4) Nav
        [t      t          t     t      t      t        t       t      t    t     t t     ]
        [t      t          t     t      t      t        t       t      t    t     t t     ]
        [t      Pause      PgUp  Up     PgDown t        t       t      t    t     t t     ]
        [t      PScreen    Left  Down   Right  t        t       t      t    t     t t     ]
        [LShift ScrollLock Home  Insert End    t        t       t      t    t     t RShift]
        [n      n          LCtrl LAlt   BSpace Space    Enter   Delete RAlt RCtrl n n     ]
        [n      n          t    {SYM}   LGui  {LAYER}  {LAYER}  RGui  {SYM} t     n n     ]
    }
    { // (5) Symbol
        [{UF2}  n  n     n    n      n       n      n      n    n     n n     ]
        [n      n  n     n    n      n       n      n      n    n     n n     ]
        [n      1  2     3    4      5       6      7      8    9     0 n     ]
        [n      = '_'    -    +      n       n     '('    '{'  '['    n n     ]
        [LShift ~ '`'    |    Bslash n       n     ')'    '}'  ']'    n RShift]
        [n      n  LCtrl LAlt BSpace Space   Enter  Delete RAlt RCtrl n n     ]
        [n      n  t    {SYM} LGui  {LAYER} {LAYER} RGui  {SYM} t     n n     ]
    }
    { // (6) MMO
        [Escape F1 F2   F3     F4     F5       F6      F7    F8   F9         F10     n]
        [M      J  C    R      U      -        n       n     n    KpAsterisk n       n]
        [N      1  2    3      4      5        n       Kp7   Kp8  Kp9        KpMinus n]
        [Tab    I  A    W      D      =        n       Kp4   Kp5  Kp6        KpPlus  n]
        [B      6  7    8      9      0        n       Kp1   Kp2  Kp3        KpEnter n]
        [n      n  LAlt LCtrl LShift Space     Kp0     RCtrl RAlt RShift     n       n]
        [n      n  t   {SYM}  LGui  {LAYER}   {LAYER}  RGui {SYM} t          n       n]
    }
};
