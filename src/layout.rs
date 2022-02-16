use keyberon::{
    action::{self, d, k, l, m, HoldTapConfig},
    key_code::KeyCode::*,
};

pub enum CustomAction {
    BootReset,
    // QwertyLedMode,
    // ColemakLedMode,
    // SymbolLedMode,
}

pub(crate) type Action = action::Action<CustomAction>;

const fn make_holdtap(hold: &'static Action, tap: &'static Action) -> Action {
    make_holdtap_with_timeout(hold, tap, 300)
}

const fn make_holdtap_with_timeout(
    hold: &'static Action,
    tap: &'static Action,
    timeout: u16,
) -> Action {
    Action::HoldTap {
        timeout,
        hold,
        tap,
        config: HoldTapConfig::Default,
        tap_hold_interval: 0,
    }
}

const UF2: Action = action::Action::Custom(CustomAction::BootReset);

// Colemak DH
const COLEMAK: Action = d(0);
const QWERTY: Action = d(1);
const NUM_PAD: Action = l(2);
const NAV: Action = l(3);
const SYM: Action = d(4);

// Colemak DH Hold-Tap configs.
const CTRL_A: Action = make_holdtap(&k(LCtrl), &k(A));
const SHIFT_R: Action = make_holdtap(&k(LShift), &k(R));
const ALT_S: Action = make_holdtap(&k(LAlt), &k(S));
const GUI_T: Action = make_holdtap(&k(LGui), &k(T));

const CTRL_O: Action = make_holdtap(&k(RCtrl), &k(O));
const SHIFT_I: Action = make_holdtap(&k(RShift), &k(I));
const ALT_E: Action = make_holdtap(&k(RAlt), &k(E));
const GUI_N: Action = make_holdtap(&k(RGui), &k(N));

// Qwerty Hold-Tap configs. Shares Ctrl A from Colemak DH config.
const SHIFT_S: Action = make_holdtap(&k(LShift), &k(S));
const ALT_D: Action = make_holdtap(&k(LAlt), &k(D));
const GUI_F: Action = make_holdtap(&k(LGui), &k(F));

const CTRL_SEMI: Action = make_holdtap(&k(RCtrl), &k(SColon));
const SHIFT_L: Action = make_holdtap(&k(RShift), &k(L));
const ALT_K: Action = make_holdtap(&k(RAlt), &k(K));
const GUI_J: Action = make_holdtap(&k(RGui), &k(J));

// Symbol layer Hold-Tap configs.
const CTRL_EQ: Action = make_holdtap(&k(LCtrl), &k(Equal));
const SFT_US: Action = make_holdtap(&k(LShift), &m(&[LShift, Minus]));
const ALT_MIN: Action = make_holdtap(&k(LAlt), &k(Minus));
const GUI_PL: Action = make_holdtap(&k(LGui), &m(&[LShift, Equal]));
const CTRL_PIPE: Action = make_holdtap(&k(RCtrl), &m(&[LShift, Bslash]));
const SFT_RB: Action = make_holdtap(&k(RShift), &k(RBracket));
const ALT_LB: Action = make_holdtap(&k(RAlt), &k(LBracket));
const GUI_DQ: Action = make_holdtap(&k(RGui), &m(&[LShift, Bslash]));

const MU: Action = k(VolUp);
const MD: Action = k(VolDown);
const MP: Action = k(MediaPlayPause);

pub(crate) static LAYERS: keyberon::layout::Layers<CustomAction> = keyberon::layout::layout! {
    { // (0) Colemak Dh
        [F1       F2       F3        F4      F5        F6           F7    F8       F9      F10       F11     F12   ]
        [CapsLock 1        2         3       4         5            6     7        8       9         0       =     ]
        ['`'      Q        W         F       P         B            J     L        U       Y         ;       -     ]
        [Tab     {CTRL_A} {SHIFT_R} {ALT_S} {GUI_T}    G            M    {GUI_N}  {ALT_E} {SHIFT_I} {CTRL_O} Quote ]
        [Escape   Z        X         C       D         V            K     H        ,       .         /       Bslash]
        [n        n        t         t       BSpace    Space        Enter Delete  '['     ']'        n       n     ]
        [n        n        {SYM}     {MP}    n         {NUM_PAD}    {NAV} {QWERTY} {MD}    {MU}      n       n     ]
    }
    { // (1) Qwerty
        [F1       F2       F3        F4      F5        F6           F7    F8      F9      F10       F11        F12   ]
        [CapsLock 1        2         3       4         5            6     7       8       9         0          =     ]
        ['`'      Q        W         E       R         T            Y     U       I       O         P          -     ]
        [Tab     {CTRL_A} {SHIFT_S} {ALT_D} {GUI_F}    G            H    {GUI_J} {ALT_K} {SHIFT_L} {CTRL_SEMI} Quote ]
        [Escape   Z        X         C       V         B            N     M       ,       .         /          Bslash]
        [n        n        t         t       BSpace    Space        Enter Delete '['     ']'        n          n     ]
        [n        n        {SYM}     {MP}    {COLEMAK} {NUM_PAD}    {NAV} n       {MD}    {MU}      n          n     ]
    }
    { // (2) Numpad
        [t t t t t t        t       t       t       t          t       t]
        [t t t t t t        t       NumLock KpSlash KpAsterisk t       t]
        [t t t t t t        t       Kp7     Kp8     Kp9        KpMinus t]
        [t t t t t t        t       Kp4     Kp5     Kp6        KpPlus  t]
        [t t t t t t        t       Kp1     Kp2     Kp3        KpEnter t]
        [n n t t t t        KpEnter Kp0     Kp0     KpDot      n       n]
        [n n t t t t        t       t       t       t          n       n]
    }
    { // (3) Nav
        [t t          t    t      t      t       t t t t t t]
        [t t          t    t      t      t       t t t t t t]
        [t Pause      PgUp Up     PgDown t       t t t t t t]
        [t PScreen    Left Down   Right  t       t t t t t t]
        [t ScrollLock Home Insert End    t       t t t t t t]
        [n n          t    t      t      t       t t t t n n]
        [n n          t    t      t      t       t t t t n n]
    }
    { // (4) Symbols
        [{UF2} n         n         n         n         n            n     n        n       n         n           n]
        [n     1         2         3         4         5            6     7        8       9         0           n]
        [n     !         @         #         $         %            ^     &       '('     ')'        *           n]
        [n     {CTRL_EQ} {SFT_US}  {ALT_MIN} {GUI_PL}  n            n    {GUI_DQ} {ALT_LB} {SFT_RB}  {CTRL_PIPE} n]
        [n     n         n         <         >         n            n     Quote   '['     ']'        Bslash      n]
        [n     n         t         t         BSpace    Space        Enter Delete   n       n         n           n]
        [n     n         n         {MP}      {COLEMAK} {NUM_PAD}    {NAV} {QWERTY} {MD}    {MU}      n           n]
    }
};
