#pragma once


namespace msr {
    namespace airlib {
        struct DamageCoeff {
        public:
            float propeller_damage_coefficients[4] = { 0.5, 0.7, 0.9, 1.0 };
        };
    };
} //namespace