#pragma once
#include <boost/units/base_units/metric/hour.hpp>
#include <boost/units/base_units/us/mile.hpp>
#include <boost/units/make_scaled_unit.hpp>

#include "lanelet2_core/Forward.h"

namespace lanelet {
namespace units {
using Mile = boost::units::us::mile_base_unit::unit_type;
using Meter = boost::units::si::meter_base_unit::unit_type;
using Exp3 = boost::units::scale<10, boost::units::static_rational<3> >;
using NoUnit = boost::units::si::dimensionless::unit_type;
using Kilo = boost::units::make_scaled_unit<NoUnit, Exp3>::type;
using Km = boost::units::multiply_typeof_helper<Kilo, Meter>::type;
using Second = boost::units::si::second_base_unit::unit_type;
using Hour = boost::units::metric::hour_base_unit::unit_type;
using MPH = boost::units::divide_typeof_helper<Mile, Hour>::type;
using KmH = boost::units::divide_typeof_helper<Km, Hour>::type;

using MeterQuantity = boost::units::quantity<Meter>;
using SecondQuantity = boost::units::quantity<Second>;
using KmHQuantity = boost::units::quantity<KmH>;
using MPHQuantity = boost::units::quantity<MPH>;
using Distance = MeterQuantity;
using Time = SecondQuantity;

namespace literals {
inline Distance operator"" _m(long double d) { return Distance(d * Meter()); }
inline Distance operator"" _m(unsigned long long int d) {  // NOLINT
  return Distance(d * Meter());
}

inline Time operator"" _s(long double s) { return Time(s * Second()); }
inline Time operator"" _s(unsigned long long int s) {  // NOLINT
  return Time(s * Second());
}

inline Velocity operator"" _mps(long double d) { return Velocity{d * MPS()}; }
inline Velocity operator"" _mps(unsigned long long int d) {  // NOLINT
  return Velocity{d * MPS()};
}
inline Velocity operator"" _kmh(long double d) { return Velocity{d * KmH()}; }
inline Velocity operator"" _kmh(unsigned long long int d) {  // NOLINT
  return Velocity{d * KmH()};
}
inline Velocity operator"" _mph(long double d) { return Velocity{d * MPH()}; }
inline Velocity operator"" _mph(unsigned long long int d) {  // NOLINT
  return Velocity{d * MPH()};
}
inline Acceleration operator"" _mps2(long double d) { return Acceleration{d * MPS2()}; }
inline Acceleration operator"" _mps2(unsigned long long int d) {  // NOLINT
  return Acceleration{d * MPS2()};
}
}  // namespace literals
}  // namespace units
}  // namespace lanelet
