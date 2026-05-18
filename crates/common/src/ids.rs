//! Strongly-typed indices into vertex / triangle / polygon arrays.

macro_rules! id_newtype {
    ($name:ident) => {
        #[derive(Copy, Clone, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
        pub struct $name(pub u32);

        impl $name {
            pub const INVALID: Self = Self(u32::MAX);

            #[inline]
            pub const fn new(value: u32) -> Self {
                Self(value)
            }

            #[inline]
            pub const fn get(self) -> u32 {
                self.0
            }

            #[inline]
            pub const fn index(self) -> usize {
                self.0 as usize
            }

            #[inline]
            pub const fn is_valid(self) -> bool {
                self.0 != u32::MAX
            }
        }

        impl From<u32> for $name {
            #[inline]
            fn from(v: u32) -> Self {
                Self(v)
            }
        }

        impl From<$name> for u32 {
            #[inline]
            fn from(v: $name) -> Self {
                v.0
            }
        }

        impl From<$name> for usize {
            #[inline]
            fn from(v: $name) -> Self {
                v.0 as usize
            }
        }
    };
}

id_newtype!(VertexId);
id_newtype!(TriangleId);
id_newtype!(PolygonId);
