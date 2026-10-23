#include "util/units.hpp"

#include <array>
#include <utility>

namespace eyas::units {
namespace {

// Every unit accepted in config and table files. Add entries here as source
// data needs them; each must be an exact product of the definitions in
// units.hpp.
using D = Dimension;
// clang-format off
constexpr std::array<std::pair<std::string_view, Unit>, 98> kRegistry{{
    {"1",          {1.0, D::dimensionless}},
    {"m",          {1.0, D::length}},
    {"ft",         {kFoot, D::length}},
    {"in",         {kInch, D::length}},
    {"s",          {1.0, D::time}},
    {"Hz",         {1.0, D::frequency}},
    {"1/s",        {1.0, D::frequency}},  // a rate constant, e.g. an actuator's gain
    {"kg",         {1.0, D::mass}},
    {"slug",       {kSlug, D::mass}},
    {"lbm",        {kPoundMass, D::mass}},
    {"N",          {1.0, D::force}},
    {"lbf",        {kPoundForce, D::force}},
    {"rad",        {1.0, D::angle}},
    {"deg",        {kDegree, D::angle}},
    {"1/rad",      {1.0, D::per_angle}},
    {"1/deg",      {1.0 / kDegree, D::per_angle}},
    {"m/s",        {1.0, D::speed}},
    {"ft/s",       {kFoot, D::speed}},
    {"kt",         {kKnot, D::speed}},
    {"rad/s",      {1.0, D::angular_rate}},
    {"deg/s",      {kDegree, D::angular_rate}},
    {"rpm",        {kRpm, D::angular_rate}},
    {"m/s^2",      {1.0, D::acceleration}},
    {"kt/s",       {kKnot, D::acceleration}},
    {"ft/s^2",     {kFoot, D::acceleration}},
    {"kg/m^3",     {1.0, D::density}},
    {"slug/ft^3",  {kSlug / (kFoot * kFoot * kFoot), D::density}},
    {"N*m",        {1.0, D::moment}},
    {"ft*lbf",     {kFoot * kPoundForce, D::moment}},
    {"slug*ft",    {kSlug * kFoot, D::mass_moment}},
    {"kg*m",       {1.0, D::mass_moment}},
    {"kg*m^2",     {1.0, D::inertia}},
    {"slug*ft^2",  {kSlug * kFoot * kFoot, D::inertia}},
    {"lbf*in*s^2", {kPoundForce * kInch, D::inertia}},
    {"N*m/rad",      {1.0, D::rotational_stiffness}},
    {"ft*lbf/rad",   {kFoot * kPoundForce, D::rotational_stiffness}},
    {"N*m*s/rad",    {1.0, D::rotational_damping}},
    {"ft*lbf*s/rad", {kFoot * kPoundForce, D::rotational_damping}},
    {"m^2",          {1.0, D::area}},
    {"ft^2",         {kFoot * kFoot, D::area}},
    {"m^3",          {1.0, D::volume}},
    {"ft^3",         {kFoot * kFoot * kFoot, D::volume}},
    {"rad/N",        {1.0, D::angle_per_force}},
    {"deg/lbf",      {kDegree / kPoundForce, D::angle_per_force}},
    {"Pa",           {1.0, D::pressure}},
    {"psi",          {kPsi, D::pressure}},
    {"N/m",          {1.0, D::stiffness}},
    {"lbf/in",       {kPoundForce / kInch, D::stiffness}},
    {"lbf/ft",       {kPoundForce / kFoot, D::stiffness}},
    {"N*s/m",        {1.0, D::damping}},
    {"lbf*s/ft",     {kPoundForce / kFoot, D::damping}},
    {"N*s^2/m^2",    {1.0, D::quadratic_damping}},
    {"lbf*s^2/ft^2", {kPoundForce / (kFoot * kFoot), D::quadratic_damping}},
    {"rad/m",        {1.0, D::angle_per_length}},
    {"deg/in",       {kDegree / kInch, D::angle_per_length}},
    {"rad/(m/s^2)",  {1.0, D::angle_per_acceleration}},
    {"deg/(ft/s^2)", {kDegree / kFoot, D::angle_per_acceleration}},
    {"s^2",          {1.0, D::time_squared}},
    // Percent of AFCS authority, a fraction in SI (GenHel §5.5).
    {"%",                {0.01, D::dimensionless}},
    {"%/s",              {0.01, D::frequency}},
    {"%/deg",            {0.01 / kDegree, D::per_angle}},
    {"1/(rad/s)",        {1.0, D::per_angular_rate}},
    {"%/(deg/s)",        {0.01 / kDegree, D::per_angular_rate}},
    {"s/m",              {1.0, D::per_speed}},
    {"%/kt",             {0.01 / kKnot, D::per_speed}},
    {"s^2/m",            {1.0, D::per_acceleration}},
    {"%/(ft/s^2)",       {0.01 / kFoot, D::per_acceleration}},
    {"(rad/s)/(m/s^2)",  {1.0, D::rate_per_acceleration}},
    {"(deg/s)/(ft/s^2)", {kDegree / kFoot, D::rate_per_acceleration}},
    {"deg/%",            {kDegree / 0.01, D::angle}},
    {"%/(deg*s)",        {0.01 / kDegree, D::per_angle_time}},
    {"%/(kt*s)",         {0.01 / kKnot, D::per_length}},
    {"%/(ft/s^2*s)",     {0.01 / kFoot, D::per_speed}},
    // Engines (GenHel §5.6, D-053); "lbm/hr" is pounds mass per hour.
    {"kg/s",             {1.0, D::mass_flow}},
    {"lbm/hr",           {kPoundMass / 3600.0, D::mass_flow}},
    {"Pa*s/kg",          {1.0, D::pressure_per_mass_flow}},
    {"psi*hr/lbm",       {kPsi * 3600.0 / kPoundMass, D::pressure_per_mass_flow}},
    {"N*m*s/kg",         {1.0, D::moment_per_mass_flow}},
    {"ft*lbf*hr/lbm",    {kFoot * kPoundForce * 3600.0 / kPoundMass, D::moment_per_mass_flow}},
    {"ft*lbf/rpm",       {kFoot * kPoundForce / kRpm, D::rotational_damping}},
    {"kg/(s*Pa)",        {1.0, D::mass_flow_per_pressure}},
    {"lbm/(hr*psi*%)",   {kPoundMass / (3600.0 * kPsi * 0.01), D::mass_flow_per_pressure}},
    {"1/(N*m*s)",        {1.0, D::per_moment_time}},
    {"%/(ft*lbf*s)",     {0.01 / (kFoot * kPoundForce), D::per_moment_time}},
    {"V",                {1.0, D::voltage}},
    {"V/%",              {1.0 / 0.01, D::voltage}},
    {"V/(N*m)",          {1.0, D::voltage_per_moment}},
    {"V/(ft*lbf)",       {1.0 / (kFoot * kPoundForce), D::voltage_per_moment}},
    {"A/V",              {1.0, D::current_per_voltage}},
    {"mA/V",             {0.001, D::current_per_voltage}},
    {"m/A",              {1.0, D::length_per_current}},
    {"in/mA",            {kInch / 0.001, D::length_per_current}},
    {"V/m",              {1.0, D::voltage_per_length}},
    {"V/in",             {1.0 / kInch, D::voltage_per_length}},
    {"%/in",             {0.01 / kInch, D::per_length}},
    {"psi/%",            {kPsi / 0.01, D::pressure}},
    {"ft*lbf/%",         {kFoot * kPoundForce / 0.01, D::moment}},
    {"lbm/(hr*psi)",     {kPoundMass / (3600.0 * kPsi), D::mass_flow_per_pressure}},
}};
// clang-format on

}  // namespace

std::optional<Unit> find_unit(std::string_view unit) noexcept {
    for (const auto& [name, u] : kRegistry) {
        if (name == unit) {
            return u;
        }
    }
    return std::nullopt;
}

std::optional<double> si_scale(std::string_view unit) noexcept {
    const auto u = find_unit(unit);
    if (!u) {
        return std::nullopt;
    }
    return u->si_scale;
}

std::string_view dimension_name(Dimension d) noexcept {
    switch (d) {
        case Dimension::dimensionless:
            return "dimensionless";
        case Dimension::length:
            return "length";
        case Dimension::time:
            return "time";
        case Dimension::frequency:
            return "frequency";
        case Dimension::mass:
            return "mass";
        case Dimension::force:
            return "force";
        case Dimension::angle:
            return "angle";
        case Dimension::per_angle:
            return "per angle";
        case Dimension::speed:
            return "speed";
        case Dimension::angular_rate:
            return "angular rate";
        case Dimension::acceleration:
            return "acceleration";
        case Dimension::density:
            return "density";
        case Dimension::moment:
            return "moment";
        case Dimension::mass_moment:
            return "mass moment";
        case Dimension::inertia:
            return "inertia";
        case Dimension::rotational_stiffness:
            return "rotational stiffness";
        case Dimension::rotational_damping:
            return "rotational damping";
        case Dimension::area:
            return "area";
        case Dimension::volume:
            return "volume";
        case Dimension::angle_per_force:
            return "angle per force";
        case Dimension::pressure:
            return "pressure";
        case Dimension::stiffness:
            return "stiffness";
        case Dimension::damping:
            return "damping";
        case Dimension::quadratic_damping:
            return "quadratic damping";
        case Dimension::angle_per_length:
            return "angle per length";
        case Dimension::angle_per_acceleration:
            return "angle per acceleration";
        case Dimension::time_squared:
            return "time squared";
        case Dimension::per_angular_rate:
            return "per angular rate";
        case Dimension::per_speed:
            return "per speed";
        case Dimension::per_acceleration:
            return "per acceleration";
        case Dimension::rate_per_acceleration:
            return "angular rate per acceleration";
        case Dimension::per_angle_time:
            return "per angle time";
        case Dimension::per_length:
            return "per length";
        case Dimension::mass_flow:
            return "mass flow";
        case Dimension::pressure_per_mass_flow:
            return "pressure per mass flow";
        case Dimension::moment_per_mass_flow:
            return "moment per mass flow";
        case Dimension::mass_flow_per_pressure:
            return "mass flow per pressure";
        case Dimension::per_moment_time:
            return "per moment time";
        case Dimension::voltage:
            return "voltage";
        case Dimension::voltage_per_moment:
            return "voltage per moment";
        case Dimension::current_per_voltage:
            return "current per voltage";
        case Dimension::length_per_current:
            return "length per current";
        case Dimension::voltage_per_length:
            return "voltage per length";
    }
    return "unknown";
}

}  // namespace eyas::units
