#include "util/units.hpp"

#include <cmath>
#include <numbers>

#include <gtest/gtest.h>

namespace eyas::units {
namespace {

// Reference values: NIST SP 811 (2008), Appendix B.
TEST(Units, ExactDefinitions) {
    EXPECT_EQ(kFoot, 0.3048);
    EXPECT_EQ(kInch, 0.0254);
    EXPECT_EQ(kStandardGravity, 9.80665);
    EXPECT_EQ(kPoundMass, 0.45359237);
}

TEST(Units, DerivedFactorsMatchNist) {
    EXPECT_NEAR(kPoundForce, 4.4482216152605, 1e-13);
    EXPECT_NEAR(kSlug, 14.593902937206364, 1e-12);
    EXPECT_NEAR(kKnot, 0.51444444444444444, 1e-15);
    EXPECT_NEAR(kDegree, std::numbers::pi / 180.0, 0.0);
}

TEST(Units, RegistryScales) {
    EXPECT_EQ(si_scale("1"), 1.0);
    EXPECT_EQ(si_scale("m"), 1.0);
    EXPECT_EQ(si_scale("ft"), kFoot);
    EXPECT_EQ(si_scale("in"), kInch);
    EXPECT_EQ(si_scale("deg"), kDegree);
    EXPECT_EQ(si_scale("lbf"), kPoundForce);
    EXPECT_EQ(si_scale("lbm"), kPoundMass);
    EXPECT_EQ(si_scale("slug"), kSlug);
    EXPECT_EQ(si_scale("kt"), kKnot);
    EXPECT_EQ(si_scale("rpm"), kRpm);
    EXPECT_EQ(si_scale("1/deg"), 1.0 / kDegree);
}

// GenHel lists inertias in slug·ft² and lb·in·s² (NASA CR-166309, Table 4.2
// and p. 5.10-19); both must convert to the same SI values.
TEST(Units, InertiaUnitsAgree) {
    const double slug_ft2 = *si_scale("slug*ft^2");
    const double lbf_in_s2 = *si_scale("lbf*in*s^2");
    EXPECT_NEAR(slug_ft2, 1.3558179483314004, 1e-15);
    EXPECT_NEAR(lbf_in_s2, 0.1129848290276167, 1e-15);
    EXPECT_NEAR(12.0 * lbf_in_s2, slug_ft2, 1e-15);
}

// Every unit carries the dimension its name says; the pairs that only differ
// by dimension (Hz vs rad/s, rad vs 1, N*m vs kg*m) must not collapse.
TEST(Units, Dimensions) {
    const auto dim = [](std::string_view u) { return find_unit(u)->dimension; };
    EXPECT_EQ(dim("1"), Dimension::dimensionless);
    EXPECT_EQ(dim("ft"), Dimension::length);
    EXPECT_EQ(dim("s"), Dimension::time);
    EXPECT_EQ(dim("Hz"), Dimension::frequency);
    EXPECT_EQ(dim("rpm"), Dimension::angular_rate);
    EXPECT_EQ(dim("rad/s"), Dimension::angular_rate);
    EXPECT_EQ(dim("deg"), Dimension::angle);
    EXPECT_EQ(dim("1/deg"), Dimension::per_angle);
    EXPECT_EQ(dim("lbm"), Dimension::mass);
    EXPECT_EQ(dim("slug"), Dimension::mass);
    EXPECT_EQ(dim("lbf"), Dimension::force);
    EXPECT_EQ(dim("kt"), Dimension::speed);
    EXPECT_EQ(dim("ft/s^2"), Dimension::acceleration);
    EXPECT_EQ(dim("slug/ft^3"), Dimension::density);
    EXPECT_EQ(dim("ft*lbf"), Dimension::moment);
    EXPECT_EQ(dim("slug*ft"), Dimension::mass_moment);
    EXPECT_EQ(dim("lbf*in*s^2"), Dimension::inertia);
    EXPECT_EQ(dim("ft*lbf/rad"), Dimension::rotational_stiffness);
    EXPECT_EQ(dim("N*m/rad"), Dimension::rotational_stiffness);
    EXPECT_EQ(dim("ft*lbf*s/rad"), Dimension::rotational_damping);
    EXPECT_EQ(dim("N*m*s/rad"), Dimension::rotational_damping);
    EXPECT_EQ(dim("ft^2"), Dimension::area);
    EXPECT_EQ(dim("deg/lbf"), Dimension::angle_per_force);
    EXPECT_EQ(find_unit("ft^2")->si_scale, kFoot * kFoot);
    EXPECT_EQ(dim("ft^3"), Dimension::volume);
    EXPECT_EQ(dim("m^3"), Dimension::volume);
    EXPECT_EQ(find_unit("ft^3")->si_scale, kFoot * kFoot * kFoot);
    EXPECT_EQ(find_unit("deg/lbf")->si_scale, kDegree / kPoundForce);
    EXPECT_EQ(find_unit("ft*lbf/rad")->si_scale, find_unit("ft*lbf")->si_scale);
    EXPECT_EQ(find_unit("ft")->si_scale, kFoot);
    EXPECT_FALSE(find_unit("lb").has_value());
}

// AFCS units (GenHel §5.5, M8c2): the SAS and PBA work in percent of control
// authority, a fraction in SI (1 % = 0.01). A gain in % per deg/s is a
// fraction per angular rate, kept apart from time (deg per deg/s, a time),
// as per angle is from dimensionless.
TEST(Units, AfcsUnits) {
    const auto dim = [](std::string_view u) { return find_unit(u)->dimension; };
    EXPECT_EQ(dim("%"), Dimension::dimensionless);
    EXPECT_EQ(*si_scale("%"), 0.01);
    EXPECT_EQ(dim("%/s"), Dimension::frequency);
    EXPECT_EQ(dim("%/deg"), Dimension::per_angle);
    EXPECT_EQ(*si_scale("%/deg"), 0.01 / kDegree);
    EXPECT_EQ(dim("%/(deg/s)"), Dimension::per_angular_rate);
    EXPECT_EQ(dim("1/(rad/s)"), Dimension::per_angular_rate);
    EXPECT_EQ(*si_scale("%/(deg/s)"), 0.01 / kDegree);
    EXPECT_EQ(dim("%/kt"), Dimension::per_speed);
    EXPECT_EQ(dim("s/m"), Dimension::per_speed);
    EXPECT_EQ(*si_scale("%/kt"), 0.01 / kKnot);
    EXPECT_EQ(dim("%/(ft/s^2)"), Dimension::per_acceleration);
    EXPECT_EQ(dim("s^2/m"), Dimension::per_acceleration);
    EXPECT_EQ(*si_scale("%/(ft/s^2)"), 0.01 / kFoot);
    EXPECT_EQ(dim("(deg/s)/(ft/s^2)"), Dimension::rate_per_acceleration);
    EXPECT_EQ(dim("(rad/s)/(m/s^2)"), Dimension::rate_per_acceleration);
    EXPECT_EQ(*si_scale("(deg/s)/(ft/s^2)"), kDegree / kFoot);
    EXPECT_EQ(dim("deg/%"), Dimension::angle);
    EXPECT_EQ(*si_scale("deg/%"), kDegree / 0.01);
    EXPECT_NE(Dimension::per_angular_rate, Dimension::time);
    EXPECT_NE(dimension_name(Dimension::per_angular_rate), "unknown");
    EXPECT_NE(dimension_name(Dimension::per_speed), "unknown");
    EXPECT_NE(dimension_name(Dimension::per_acceleration), "unknown");
    EXPECT_NE(dimension_name(Dimension::rate_per_acceleration), "unknown");
    // FPS integrator gains (GenHel Fig. 5.1.5) and the airspeed rate limit:
    // %/(ft/s²·s) is a gain per speed, %/(kt·s) one per length.
    EXPECT_EQ(dim("%/(deg*s)"), Dimension::per_angle_time);
    EXPECT_EQ(*si_scale("%/(deg*s)"), 0.01 / kDegree);
    EXPECT_EQ(dim("%/(kt*s)"), Dimension::per_length);
    EXPECT_EQ(*si_scale("%/(kt*s)"), 0.01 / kKnot);
    EXPECT_EQ(dim("%/(ft/s^2*s)"), Dimension::per_speed);
    EXPECT_EQ(*si_scale("%/(ft/s^2*s)"), 0.01 / kFoot);
    EXPECT_EQ(dim("kt/s"), Dimension::acceleration);
    EXPECT_EQ(*si_scale("kt/s"), kKnot);
    EXPECT_NE(dimension_name(Dimension::per_angle_time), "unknown");
    EXPECT_NE(dimension_name(Dimension::per_length), "unknown");
}

// Landing-gear units (GenHel §5.7.5): tire pressure, tire stiffness in lbf/in
// and lbf/ft, linear and quadratic damping. NIST: 1 psi = 6894.757293168 Pa.
TEST(Units, GearUnits) {
    const auto dim = [](std::string_view u) { return find_unit(u)->dimension; };
    EXPECT_EQ(dim("psi"), Dimension::pressure);
    EXPECT_EQ(dim("Pa"), Dimension::pressure);
    EXPECT_NEAR(kPsi, 6894.757293168361, 1e-9);
    EXPECT_EQ(dim("lbf/in"), Dimension::stiffness);
    EXPECT_EQ(dim("lbf/ft"), Dimension::stiffness);
    EXPECT_EQ(dim("N/m"), Dimension::stiffness);
    EXPECT_NEAR(*si_scale("lbf/in"), 12.0 * *si_scale("lbf/ft"), 1e-10);
    EXPECT_EQ(dim("lbf*s/ft"), Dimension::damping);
    EXPECT_EQ(dim("N*s/m"), Dimension::damping);
    EXPECT_EQ(dim("lbf*s^2/ft^2"), Dimension::quadratic_damping);
    EXPECT_EQ(dim("N*s^2/m^2"), Dimension::quadratic_damping);
    EXPECT_NEAR(*si_scale("lbf*s^2/ft^2"), kSlug / kFoot, 1e-12);
}

// Engine and fuel-control units (GenHel §5.6, D-053): fuel flow in lbm/hr
// (1 lbm/hr = 0.45359237/3600 kg/s), map partials per fuel flow, and the
// ECU's volts, milliamps and inches. Each compound unit times its
// denominator's unit gives its numerator's.
TEST(Units, EngineUnits) {
    const auto dim = [](std::string_view u) { return find_unit(u)->dimension; };
    const auto si = [](std::string_view u) { return *si_scale(u); };
    EXPECT_EQ(dim("lbm/hr"), Dimension::mass_flow);
    EXPECT_NEAR(si("lbm/hr"), 1.259978805555556e-4, 1e-18);
    EXPECT_EQ(dim("psi*hr/lbm"), Dimension::pressure_per_mass_flow);
    EXPECT_NEAR(si("psi*hr/lbm") * si("lbm/hr"), si("psi"), 1e-9 * si("psi"));
    EXPECT_EQ(dim("ft*lbf*hr/lbm"), Dimension::moment_per_mass_flow);
    EXPECT_NEAR(si("ft*lbf*hr/lbm") * si("lbm/hr"), si("ft*lbf"), 1e-12);
    EXPECT_EQ(dim("ft*lbf/rpm"), Dimension::rotational_damping);
    EXPECT_NEAR(si("ft*lbf/rpm") * si("rpm"), si("ft*lbf"), 1e-12);
    EXPECT_EQ(dim("lbm/(hr*psi*%)"), Dimension::mass_flow_per_pressure);
    EXPECT_NEAR(si("lbm/(hr*psi*%)") * si("psi") * si("%"), si("lbm/hr"), 1e-18);
    EXPECT_EQ(dim("%/(ft*lbf*s)"), Dimension::per_moment_time);
    EXPECT_NEAR(si("%/(ft*lbf*s)") * si("ft*lbf"), 0.01, 1e-15);
    EXPECT_EQ(dim("%/(ft*lbf)"), Dimension::per_moment);
    EXPECT_NEAR(si("%/(ft*lbf)") * si("ft*lbf"), 0.01, 1e-15);
    EXPECT_EQ(dim("V/%"), Dimension::voltage);
    EXPECT_EQ(si("V/%") * si("%"), 1.0);
    EXPECT_EQ(dim("V/(ft*lbf)"), Dimension::voltage_per_moment);
    EXPECT_NEAR(si("V/(ft*lbf)") * si("ft*lbf"), 1.0, 1e-15);
    EXPECT_EQ(dim("mA/V"), Dimension::current_per_voltage);
    EXPECT_EQ(dim("in/mA"), Dimension::length_per_current);
    EXPECT_NEAR(si("in/mA") * 0.001, si("in"), 1e-15);
    EXPECT_EQ(dim("V/in"), Dimension::voltage_per_length);
    EXPECT_NEAR(si("V/in") * si("in"), 1.0, 1e-15);
    EXPECT_EQ(dim("%/in"), Dimension::per_length);
    EXPECT_NEAR(si("%/in") * si("in"), 0.01, 1e-15);
    EXPECT_EQ(dim("psi/%"), Dimension::pressure);
    EXPECT_NEAR(si("psi/%") * si("%"), si("psi"), 1e-9);
    EXPECT_EQ(dim("ft*lbf/%"), Dimension::moment);
    EXPECT_NEAR(si("ft*lbf/%") * si("%"), si("ft*lbf"), 1e-12);
    EXPECT_EQ(dim("lbm/(hr*psi)"), Dimension::mass_flow_per_pressure);
    EXPECT_NEAR(si("lbm/(hr*psi)") * si("psi"), si("lbm/hr"), 1e-18);
    for (const Dimension d :
         {Dimension::mass_flow, Dimension::pressure_per_mass_flow, Dimension::moment_per_mass_flow,
          Dimension::mass_flow_per_pressure, Dimension::per_moment_time, Dimension::voltage,
          Dimension::voltage_per_moment, Dimension::current_per_voltage,
          Dimension::length_per_current, Dimension::voltage_per_length, Dimension::per_moment}) {
        EXPECT_NE(dimension_name(d), "unknown");
    }
}

// Ballin's T700 (TM-100991, D-054): each factor derived again from the base
// definitions (the IT Btu is 1055.05585262 J, NIST SP 811 App. B.9; °R = 5/9 K).
TEST(Units, BallinT700Units) {
    const auto dim = [](std::string_view u) { return find_unit(u)->dimension; };
    const auto si = [](std::string_view u) { return *si_scale(u); };
    const double btu = 1055.05585262;  // [J]
    const double lbm = 0.45359237;     // [kg]
    const double rankine = 5.0 / 9.0;  // [K]
    const double psi = 6894.757293168361;
    EXPECT_NEAR(kBtu, btu, 1e-8);
    EXPECT_EQ(dim("degR"), Dimension::temperature);
    EXPECT_NEAR(si("degR") * 518.67, 288.15, 1e-12);  // the standard day's T
    EXPECT_EQ(dim("1/degR"), Dimension::per_temperature);
    EXPECT_NEAR(si("1/degR") * si("degR"), 1.0, 1e-15);
    EXPECT_EQ(dim("Btu/lbm"), Dimension::specific_energy);
    EXPECT_NEAR(si("Btu/lbm"), 2326.0, 1e-9);
    EXPECT_NEAR(si("Btu/lbm"), btu / lbm, 1e-6);
    EXPECT_EQ(dim("Btu/(lbm*degR)"), Dimension::specific_heat);
    EXPECT_NEAR(si("Btu/(lbm*degR)"), 4186.8, 1e-9);
    EXPECT_EQ(dim("lbm*degR/Btu"), Dimension::temperature_per_specific_energy);
    EXPECT_NEAR(si("lbm*degR/Btu") * si("Btu/lbm"), rankine, 1e-15);
    EXPECT_EQ(dim("lbm*in^2/(lbf*hr)"), Dimension::mass_flow_per_pressure);
    EXPECT_EQ(si("lbm*in^2/(lbf*hr)"), si("lbm/(hr*psi)"));
    EXPECT_EQ(dim("lbm*in^2/(lbf*hr*%)"), Dimension::mass_flow_per_pressure);
    EXPECT_EQ(si("lbm*in^2/(lbf*hr*%)"), si("lbm/(hr*psi*%)"));
    EXPECT_EQ(dim("lbm*in^2/(lbf*s)"), Dimension::mass_flow_per_pressure);
    EXPECT_NEAR(si("lbm*in^2/(lbf*s)"), lbm / psi, 1e-15);
    EXPECT_EQ(dim("lbm*in^2/(lbf*hr*in)"), Dimension::mass_flow_per_pressure_length);
    EXPECT_NEAR(si("lbm*in^2/(lbf*hr*in)") * si("in"), si("lbm/(hr*psi)"), 1e-18);
    EXPECT_EQ(dim("lbm/s"), Dimension::mass_flow);
    EXPECT_EQ(si("lbm/s"), lbm);
    EXPECT_EQ(dim("lbf^2*s^2/(lbm^2*in^4*degR)"), Dimension::burner_resistance);
    EXPECT_NEAR(si("lbf^2*s^2/(lbm^2*in^4*degR)") / (psi * psi / (lbm * lbm * rankine)), 1.0,
                1e-12);
    EXPECT_EQ(dim("lbm^0.8*s^0.2/degR^0.5"), Dimension::heat_sink_constant);
    EXPECT_NEAR(si("lbm^0.8*s^0.2/degR^0.5"), std::pow(lbm, 0.8) / std::sqrt(rankine), 1e-15);
    EXPECT_EQ(dim("ft*lbf*s^2"), Dimension::inertia);
    EXPECT_EQ(si("ft*lbf*s^2"), si("slug*ft^2"));
    EXPECT_EQ(dim("mA"), Dimension::current);
    EXPECT_EQ(si("mA"), 0.001);
    EXPECT_EQ(dim("lbm*degR^0.5/(s*psi)"), Dimension::flow_parameter);
    EXPECT_NEAR(si("lbm*degR^0.5/(s*psi)") / (lbm * std::sqrt(rankine) / psi), 1.0, 1e-14);
    EXPECT_EQ(dim("ft*lbf*s"), Dimension::moment_time);
    EXPECT_EQ(si("ft*lbf*s"), si("ft*lbf"));
    EXPECT_EQ(dim("V/degR"), Dimension::voltage_per_temperature);
    EXPECT_NEAR(si("V/degR") * si("degR"), 1.0, 1e-15);
    EXPECT_EQ(dim("V*s/%"), Dimension::voltage_time);
    EXPECT_EQ(si("V*s/%") * si("%"), 1.0);
    EXPECT_EQ(dim("hp"), Dimension::power);
    EXPECT_NEAR(si("hp"), 745.69987158227022, 1e-10);  // NIST SP 811 App. B.9
    EXPECT_EQ(dim("in/(mA*s)"), Dimension::length_per_current_time);
    EXPECT_NEAR(si("in/(mA*s)") * si("mA"), si("in"), 1e-15);
    for (const Dimension d :
         {Dimension::temperature, Dimension::per_temperature, Dimension::specific_energy,
          Dimension::specific_heat, Dimension::temperature_per_specific_energy,
          Dimension::burner_resistance, Dimension::heat_sink_constant, Dimension::current,
          Dimension::length_per_current_time, Dimension::mass_flow_per_pressure_length,
          Dimension::flow_parameter, Dimension::moment_time, Dimension::power,
          Dimension::voltage_per_temperature, Dimension::voltage_time}) {
        EXPECT_NE(dimension_name(d), "unknown");
    }
}

// Flight-control units (GenHel §5.5, M8c): rigging gains in deg/in and a
// lateral-acceleration feedback gain in deg/(ft/s²).
TEST(Units, FlightControlUnits) {
    const auto dim = [](std::string_view u) { return find_unit(u)->dimension; };
    EXPECT_EQ(dim("deg/in"), Dimension::angle_per_length);
    EXPECT_EQ(dim("rad/m"), Dimension::angle_per_length);
    EXPECT_NEAR(*si_scale("deg/in"), kDegree / 0.0254, 1e-12);
    EXPECT_EQ(dim("deg/(ft/s^2)"), Dimension::angle_per_acceleration);
    EXPECT_EQ(dim("rad/(m/s^2)"), Dimension::angle_per_acceleration);
    EXPECT_NEAR(*si_scale("deg/(ft/s^2)"), kDegree / 0.3048, 1e-12);
    EXPECT_EQ(dim("s^2"), Dimension::time_squared);
}

TEST(Units, RejectsUnknownAndAmbiguous) {
    EXPECT_FALSE(si_scale("lb").has_value());
    EXPECT_FALSE(si_scale("furlong").has_value());
    EXPECT_FALSE(si_scale("").has_value());
    EXPECT_FALSE(si_scale("FT").has_value());
}

}  // namespace
}  // namespace eyas::units
