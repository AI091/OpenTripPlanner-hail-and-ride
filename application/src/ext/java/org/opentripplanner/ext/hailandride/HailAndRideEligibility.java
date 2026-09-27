package org.opentripplanner.ext.hailandride;

import org.opentripplanner.model.PickDrop;
import org.opentripplanner.transit.model.network.TripPattern;

/**
 * Decides which patterns, and which hops of a pattern, allow hail-and-ride.
 * <p>
 * GTFS: {@code stop_times.continuous_pickup} / {@code continuous_drop_off} on the stop_time at
 * position {@code i} apply to the segment from stop {@code i} to stop {@code i+1}. Value 0
 * (continuous stopping) and 3 (coordinate with driver) allow hail-and-ride; 1 or empty (none) and
 * 2 (phone the agency) do not. The route-level columns in routes.txt are not read, because the
 * GTFS library OTP uses (onebusaway-gtfs 14.2.2) does not parse them; feeds must set the
 * stop_times columns.
 * <p>
 * Only headway-based trips (GTFS frequencies.txt with exact_times=0) are supported for now.
 */
final class HailAndRideEligibility {

  private HailAndRideEligibility() {}

  static boolean isEligible(TripPattern pattern) {
    var timetable = pattern.getScheduledTimetable();
    var entries = timetable.getFrequencyEntries();
    if (entries.isEmpty() || !timetable.getTripTimes().isEmpty()) {
      return false;
    }
    // exact_times=1 entries have routingSlack()==0; only headway-based entries are supported.
    if (!entries.stream().allMatch(e -> e.routingSlack() > 0)) {
      return false;
    }
    for (int hop = 0; hop < pattern.numberOfStops() - 1; hop++) {
      if (hopAllowsPickup(pattern, hop) || hopAllowsDropOff(pattern, hop)) {
        return true;
      }
    }
    return false;
  }

  static boolean hopAllowsPickup(TripPattern pattern, int hop) {
    return allows(pattern.getStopPattern().getContinuousPickup(hop));
  }

  static boolean hopAllowsDropOff(TripPattern pattern, int hop) {
    return allows(pattern.getStopPattern().getContinuousDropOff(hop));
  }

  private static boolean allows(PickDrop value) {
    return value == PickDrop.SCHEDULED || value == PickDrop.COORDINATE_WITH_DRIVER;
  }
}
