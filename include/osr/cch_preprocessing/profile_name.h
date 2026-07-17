#pragma once

#include "osr/types.h"
#include "osr/routing/profile.h"
#include "osr/routing/profiles/bike.h"
#include "osr/routing/profiles/bike_sharing.h"
#include "osr/routing/profiles/car.h"
#include "osr/routing/profiles/car_parking.h"
#include "osr/routing/profiles/car_sharing.h"
#include "osr/routing/profiles/ferry.h"
#include "osr/routing/profiles/foot.h"
#include "osr/routing/profiles/railway.h"
#include "osr/routing/profiles/hgv.h"

namespace osr::cch_preprocessing {

    template <Profile P>
    struct profile_name;

    template <>
    struct profile_name<foot<false, elevator_tracking>> {
        static constexpr std::string_view const value = "foot";
    };
    template <>
    struct profile_name<foot<true, elevator_tracking>> {
        static constexpr std::string_view const value = "wheelchair";
    };
    template <>
    struct profile_name<bike<bike_costing::kSafe, kElevationNoCost>> {
        static constexpr std::string_view const value = "bike";
    };
    template <>
    struct profile_name<bike<bike_costing::kFast, kElevationNoCost>> {
        static constexpr std::string_view const value = "bike_fast";
    };
    template <>
    struct profile_name<bike<bike_costing::kSafe, kElevationLowCost>> {
        static constexpr std::string_view const value = "bike_elevation_low";
    };
    template <>
    struct profile_name<bike<bike_costing::kSafe, kElevationHighCost>> {
        static constexpr std::string_view const value = "bike_elevation_high";
    };
    template <>
    struct profile_name<car> {
        static constexpr std::string_view const value = "car";
    };
    template <>
    struct profile_name<car_parking<false, false>> {
        static constexpr std::string_view const value = "car_dropoff";
    };
    template <>
    struct profile_name<car_parking<true, false>> {
        static constexpr std::string_view const value = "car_dropoff_wheelchair";
    };
    template <>
    struct profile_name<car_parking<false, true>> {
        static constexpr std::string_view const value = "car_parking";
    };
    template <>
    struct profile_name<car_parking<true, true>> {
        static constexpr std::string_view const value = "car_parking_wheelchair";
    };
    template <>
    struct profile_name<bike_sharing> {
        static constexpr std::string_view const value = "bike_sharing";
    };
    template <>
    struct profile_name<car_sharing<track_node_tracking>> {
        static constexpr std::string_view const value = "car_sharing";
    };
    template <>
    struct profile_name<bus> {
        static constexpr std::string_view const value = "bus";
    };
    template <>
    struct profile_name<railway> {
        static constexpr std::string_view const value = "railway";
    };
    template <>
    struct profile_name<ferry> {
        static constexpr std::string_view const value = "ferry";
    };
    template <>
    struct profile_name<hgv> {
        static constexpr std::string_view const value = "hgv";
    };
    
} // namespace osr::cch_preprocessing