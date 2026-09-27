package org.opentripplanner.ext.hailandride;

import java.util.ArrayList;
import java.util.List;
import org.opentripplanner.model.plan.Itinerary;
import org.opentripplanner.raptor.spi.RaptorConstants;
import org.opentripplanner.routing.api.request.RouteRequest;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Direct hail-and-ride itineraries: walk, wave down one vehicle, ride, get off, walk. These never
 * touch a stop, so Raptor cannot find them; they are added next to the Raptor results and go
 * through the normal itinerary filter chain.
 */
public final class HailAndRideDirect {

  /** How many different routes to offer as direct rides (the filter chain trims further). */
  private static final int MAX_DIRECT_RIDES = 3;

  private static final Logger LOG = LoggerFactory.getLogger(HailAndRideDirect.class);

  private HailAndRideDirect() {}

  public static List<Itinerary> itineraries(
    HailAndRideRouter router,
    HailAndRideLegMapper mapper,
    RouteRequest request,
    int departureTime
  ) {
    var result = new ArrayList<Itinerary>();
    for (var ride : router.findDirect(departureTime, MAX_DIRECT_RIDES)) {
      int dep = ride.earliestDepartureTime(departureTime);
      if (dep == RaptorConstants.TIME_NOT_SET) {
        continue;
      }
      LOG.debug(
        "Direct hail-and-ride: {} departs {} duration {}s cost {}s walkBefore {}s/{} walkAfter {}s/{}",
        ride,
        dep,
        ride.durationSeconds(),
        router.directCostSeconds(ride),
        ride.walkBeforeSeconds(),
        ride.walkBefore() == null ? 0 : ride.walkBefore().weight(),
        ride.walkAfterSeconds(),
        ride.walkAfter() == null ? 0 : ride.walkAfter().weight()
      );
      result.add(mapper.mapDirect(ride, dep, router.directCostSeconds(ride)));
    }
    return result;
  }
}
