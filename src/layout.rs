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

#[cfg(feature = "home-mods")]
pub(crate) static LAYERS: keyberon::layout::Layers<CustomAction> = {
    use keyberon::action::{k, m, HoldTapConfig};
    use keyberon::key_code::KeyCode::{
        Equal, Kb9, LAlt, LBracket, LCtrl, LGui, LShift, Minus, Quote, RAlt, RCtrl, RGui, RShift,
        SColon, A, D, E, F, I, J, K, L, N, O, R, S, T,
    };

    const fn home_mod(tap: &'static Action, hold: &'static Action) -> Action {
        Action::HoldTap {
            timeout: 200,
            hold,
            tap,
            config: HoldTapConfig::PermissiveHold,
            tap_hold_interval: 0,
        }
    }

    // Colemak home row mods
    const A_SFT: Action = home_mod(&k(A), &k(LShift));
    const R_CTL: Action = home_mod(&k(R), &k(LCtrl));
    const S_ALT: Action = home_mod(&k(S), &k(LAlt));
    const T_GUI: Action = home_mod(&k(T), &k(LGui));

    const N_GUI: Action = home_mod(&k(N), &k(RGui));
    const E_ALT: Action = home_mod(&k(E), &k(RAlt));
    const I_CTL: Action = home_mod(&k(I), &k(RCtrl));
    const O_SFT: Action = home_mod(&k(O), &k(RShift));

    // Qwerty home row mods
    // const A_SFT: Action = home_mod(&k(A), &k(LShift));
    const S_CTL: Action = home_mod(&k(S), &k(LCtrl));
    const D_ALT: Action = home_mod(&k(D), &k(LAlt));
    const F_GUI: Action = home_mod(&k(F), &k(LGui));

    const J_GUI: Action = home_mod(&k(J), &k(RGui));
    const K_ALT: Action = home_mod(&k(K), &k(RAlt));
    const L_CTL: Action = home_mod(&k(L), &k(RCtrl));
    const SC_SFT: Action = home_mod(&k(SColon), &k(RShift));

    // Symbol home row mods
    const EQ_SFT: Action = home_mod(&k(Equal), &k(LShift));
    const US_CTL: Action = home_mod(&m(&[LShift, Minus]), &k(LCtrl));
    const MN_ALT: Action = home_mod(&k(Minus), &k(LAlt));
    const PL_GUI: Action = home_mod(&m(&[LShift, Equal]), &k(LGui));

    const LP_GUI: Action = home_mod(&m(&[LShift, Kb9]), &k(RGui));
    const LB_ALT: Action = home_mod(&m(&[LShift, LBracket]), &k(RAlt));
    const LS_CTL: Action = home_mod(&k(LBracket), &k(RCtrl));
    const QT_SFT: Action = home_mod(&k(Quote), &k(RShift));

    keyberon::layout::layout! {
        { // (0) Colemak Dh
            [F1     F2     F3     F4       F5     F6      F7     F8     F9     F10        F11    F12  ]
            ['`'    1      2      3        4      5       6      7      8      9          0      =    ]
            [Escape Q      W      F        P      B       J      L      U      Y          ;      -    ]
            [Tab   {A_SFT}{R_CTL}{S_ALT}  {T_GUI} G       M     {N_GUI}{E_ALT}{I_CTL}    {O_SFT} Quote]
            [t      Z      X      C        D      V       K      H      ,      .          /      t    ]
            [n      n      t      CapsLock BSpace Space   Enter  Delete t      ScrollLock n      n    ]
            [n      n      t      t       {SYM}  {LAYER} {LAYER}{SYM}   t      t          n      n    ]
        }
        { // (1) Qwerty
            [F1     F2     F3     F4       F5     F6        F7     F8     F9     F10        F11     F12  ]
            ['`'    1      2      3        4      5         6      7      8      9          0       =    ]
            [Escape Q      W      E        R      T         Y      U      I      O          P       -    ]
            [Tab   {A_SFT}{S_CTL}{D_ALT}  {F_GUI} G         H     {J_GUI}{K_ALT}{L_CTL}    {SC_SFT} Quote]
            [t      Z      X      C        V      B         N      M      ,      .          /       t    ]
            [n      n      t      CapsLock BSpace Space     Enter  Delete t      ScrollLock n       n    ]
            [n      n      Up     Down    {SYM}  {LAYER}   {LAYER}{SYM}   Left   Right      n       n    ]
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
            [t t      t       t    t      t         t       t       t       t          t       t]
            [t t      t       t    t      t         t       NumLock KpSlash KpAsterisk t       t]
            [t t      t       t    t      t         t       Kp7     Kp8     Kp9        KpMinus t]
            [t LShift LCtrl   LAlt LGui   t         t       Kp4     Kp5     Kp6        KpPlus  t]
            [t t      t       t    t      t         t       Kp1     Kp2     Kp3        KpEnter t]
            [n n      NumLock t    BSpace Space     KpEnter Kp0     Kp0     KpDot      n       n]
            [n n      t       t    t     {LAYER}   {LAYER}  t       t       t          n       n]
        }
        { // (4) Nav
            [t      t          t     t      t      t        t       t      t    t          t      t]
            [t      t          t     t      t      t        t       t      t    t          t      t]
            [t      Pause      PgUp  Up     PgDown t        t       t      t    t          t      t]
            [t      PScreen    Left  Down   Right  t        t       RGui   RAlt RCtrl      RShift t]
            [t      ScrollLock Home  Insert End    t        t       t      t    t          t      t]
            [n      n          t     t      BSpace Space    Enter   Delete t    ScrollLock n      n]
            [n      n          t     t      t     {LAYER}  {LAYER}  t      t    t          n      n]
        }
        { // (5) Symbol
            [{UF2} n       n       n       n       n       n      n       n       n       n       n]
            [n     n       n       n       n       n       n      n       n       n       n       n]
            [n     1       2       3       4       5       6      7       8       9       0       n]
            [n    {EQ_SFT}{US_CTL}{MN_ALT}{PL_GUI} n       n     {LP_GUI}{LB_ALT}{LS_CTL}{QT_SFT} n]
            [n     ~      '`'      |       Bslash  n       n     ')'     '}'     ']'     '"'      n]
            [n     n       t       t       BSpace  Space   Enter  Delete  t       t       n       n]
            [n     n       t       t       t      {LAYER} {LAYER} t       t       t       n       n]
        }
        { // (6) MMO
            [Escape F1 F2   F3     F4     F5       F6      F7    F8      F9         F10     n]
            [M      J  C    R      U      -        n       n     KpSlash KpAsterisk n       n]
            [N      1  2    3      4      5        n       Kp7   Kp8     Kp9        KpMinus n]
            [Tab    I  A    W      D      =        n       Kp4   Kp5     Kp6        KpPlus  n]
            [B      6  7    8      9      0        n       Kp1   Kp2     Kp3        KpEnter n]
            [n      n  LAlt LCtrl LShift Space     Kp0     RCtrl RAlt    RShift     n       n]
            [n      n  t    t     LGui  {LAYER}   {LAYER}  RGui  t       t          n       n]
        }
    }
};

#[cfg(not(feature = "home-mods"))]
pub(crate) static LAYERS: keyberon::layout::Layers<CustomAction> = {
    keyberon::layout::layout! {
        { // (0) Colemak Dh
            [F1       F2 F3 F4   F5     F6        F7     F8     F9   F10 F11 F12  ]
            [CapsLock 1  2  3    4      5         6      7      8    9   0   =    ]
            [Escape   Q  W  F    P      B         J      L      U    Y   ;   -    ]
            [Tab      A  R  S    T      G         M      N      E    I   O   Quote]
            [t        Z  X  C    D      V         K      H      ,    .   /   t    ]
            [n        n  t  t    BSpace Space     Enter  Delete t    t   n   n    ]
            [n        n  t {SYM} t     {LAYER}   {LAYER} t     {SYM} t   n   n    ]
        }
        { // (1) Qwerty
            [F1       F2 F3 F4   F5     F6        F7     F8     F9   F10 F11 F12  ]
            [CapsLock 1  2  3    4      5         6      7      8    9   0   =    ]
            [Escape   Q  W  E    R      T         Y      U      I    O   P   -    ]
            [Tab      A  S  D    F      G         H      J      K    L   ;   Quote]
            [t        Z  X  C    V      B         N      M      ,    .   /   t    ]
            [n        n  t  t    BSpace Space     Enter  Delete t    t   n   n    ]
            [n        n  t {SYM} t     {LAYER}   {LAYER} t     {SYM} t   n   n    ]
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
            [n      n  t     n    LGui  {LAYER} {LAYER} RGui   n    t     n n     ]
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
    }
};
