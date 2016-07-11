/*!
 A non-cryptographically secure PRNG.
 [http://burtleburtle.net/bob/rand/smallprng.html]
*/

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Prng {
    a: u32,
    b: u32,
    c: u32,
    d: u32,
}

impl Prng {
    pub fn seed(val: u32) -> Prng {
        let mut state = Prng {
            a: 0xF1_EA_5E_ED,
            b: val,
            c: val,
            d: val
        };
        state.advance(20);
        state
    }

    pub fn advance(&mut self, n: u32) {
        for _ in 1..n {
            self.step();
        }
    }
    
    pub fn step(&mut self) {
        let e = self.a.wrapping_sub(self.b.wrapping_shl(27));
        self.a = self.b ^ self.c.wrapping_shl(17);
        self.b = self.c.wrapping_add(self.d);
        self.c = self.d.wrapping_add(e);
        self.d = e.wrapping_add(self.a);
    }

    pub fn val(&self) -> u32 {
        self.d
    }

    pub fn scaled_val(&self, lower: u32, upper: u32) -> u32 {
        let range = upper - lower;
        lower + self.val() % (range + 1)
    }

    pub fn scaled_next(&mut self, lower: u32, upper: u32) -> u32 {
        self.step();
        self.scaled_val(lower, upper)
    }
}

//use core::iter;

impl Iterator for Prng {
    type Item = u32;
   
    fn next(&mut self) -> Option<u32> {
       self.step();
       Some(self.scaled_val(0, 16))
   }
}
