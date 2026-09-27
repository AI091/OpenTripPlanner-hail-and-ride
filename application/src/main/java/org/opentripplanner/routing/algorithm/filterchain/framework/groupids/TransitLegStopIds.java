package org.opentripplanner.routing.algorithm.filterchain.framework.groupids;

import org.opentripplanner.core.model.id.FeedScopedId;
import org.opentripplanner.model.plan.Leg;
import org.opentripplanner.model.plan.leg.ScheduledTransitLeg;

/**
 * Station-or-stop ids of a transit leg's ends. A hail-and-ride leg may start or end between two
 * stops ({@code from().stop == null}); then the stop that brackets the hail point (the stop before
 * the boarding point / after the alighting point) is used, so similar rides group together.
 */
final class TransitLegStopIds {

  private TransitLegStopIds() {}

  static FeedScopedId from(Leg leg) {
    var stop = leg.from().stop;
    if (stop == null && leg instanceof ScheduledTransitLeg stl) {
      stop = stl.tripPattern().getStop(stl.boardStopPosInPattern());
    }
    return stop.getStationOrStopId();
  }

  static FeedScopedId to(Leg leg) {
    var stop = leg.to().stop;
    if (stop == null && leg instanceof ScheduledTransitLeg stl) {
      stop = stl.tripPattern().getStop(stl.alightStopPosInPattern());
    }
    return stop.getStationOrStopId();
  }
}
