//! Minimal 3-vector, just what the sim needs.
use std::ops::{Add, AddAssign, Mul, Neg, Sub};

#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub struct Vec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vec3 {
    pub const ZERO: Vec3 = Vec3 { x: 0.0, y: 0.0, z: 0.0 };

    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Vec3 { x, y, z }
    }
    pub fn dot(self, o: Vec3) -> f64 {
        self.x * o.x + self.y * o.y + self.z * o.z
    }
    pub fn length(self) -> f64 {
        self.dot(self).sqrt()
    }
    /// The cross product self x o (right-handed: x cross y = z).
    pub fn cross(self, o: Vec3) -> Vec3 {
        Vec3::new(
            self.y * o.z - self.z * o.y,
            self.z * o.x - self.x * o.z,
            self.x * o.y - self.y * o.x,
        )
    }
    /// This vector scaled to length 1, or None when it is zero or has a non-finite component.
    /// It is divided by its largest component before the length is taken, so a huge or tiny
    /// vector cannot overflow or underflow (as `collide::Plane::new` does for a normal).
    pub fn normalized(self) -> Option<Vec3> {
        let finite = self.x.is_finite() && self.y.is_finite() && self.z.is_finite();
        let m = self.x.abs().max(self.y.abs()).max(self.z.abs());
        if !(finite && m > 0.0) {
            return None;
        }
        let s = Vec3::new(self.x / m, self.y / m, self.z / m);
        Some(s * (1.0 / s.length()))
    }
}

impl Add for Vec3 {
    type Output = Vec3;
    fn add(self, o: Vec3) -> Vec3 {
        Vec3::new(self.x + o.x, self.y + o.y, self.z + o.z)
    }
}
impl Sub for Vec3 {
    type Output = Vec3;
    fn sub(self, o: Vec3) -> Vec3 {
        Vec3::new(self.x - o.x, self.y - o.y, self.z - o.z)
    }
}
impl Mul<f64> for Vec3 {
    type Output = Vec3;
    fn mul(self, s: f64) -> Vec3 {
        Vec3::new(self.x * s, self.y * s, self.z * s)
    }
}
impl Neg for Vec3 {
    type Output = Vec3;
    fn neg(self) -> Vec3 {
        Vec3::new(-self.x, -self.y, -self.z)
    }
}
impl AddAssign for Vec3 {
    fn add_assign(&mut self, o: Vec3) {
        *self = *self + o;
    }
}
