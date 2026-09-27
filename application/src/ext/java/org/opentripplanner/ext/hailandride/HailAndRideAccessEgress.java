package org.opentripplanner.ext.hailandride;

import java.util.Optional;
import org.opentripplanner.framework.model.TimeAndCost;
import org.opentripplanner.raptor.api.model.RaptorAccessEgress;
import org.opentripplanner.raptor.spi.RaptorConstants;
import org.opentripplanner.routing.algorithm.raptoradapter.transit.RoutingAccessEgress;
import org.opentripplanner.street.search.state.State;

/**
 * A Raptor access or egress that contains a hail-and-ride ride.
 * <pre>
 * ACCESS:  origin --walk--> board point p (between stops) --ride pattern P--> stop k
 * EGRESS:  stop k --ride pattern P--> alight point q (between stops) --walk--> destination
 * </pre>
 * For Raptor this looks exactly like a FLEX access/egress: it has one ride and it is connected to
 * the Raptor stop {@code k} "on board". Raptor can then transfer at {@code k}, board another
 * route at {@code k}, or (for access) use an egress at {@code k}.
 * <p>
 * Timing follows OTP's rule for headway-based (exact_times=0) trips: the rider waits a full
 * headway at the boarding point. The duration is therefore constant, and the only time
 * dependency is the service window (vehicles run e.g. 07:00-22:00), exposed as opening hours.
 */
public final class HailAndRideAccessEgress implements RoutingAccessEgress {

  private final int stop;
  private final int durationInSeconds;
  private final int c1;
  private final TimeAndCost penalty;
  private final HailAndRideRide ride;

  HailAndRideAccessEgress(int stop, int durationInSeconds, int c1, HailAndRideRide ride) {
    this(stop, durationInSeconds, c1, TimeAndCost.ZERO, ride);
  }

  private HailAndRideAccessEgress(
    int stop,
    int durationInSeconds,
    int c1,
    TimeAndCost penalty,
    HailAndRideRide ride
  ) {
    this.stop = stop;
    this.durationInSeconds = durationInSeconds;
    this.c1 = c1;
    this.penalty = penalty;
    this.ride = ride;
  }

  /**
   * Find the hail-and-ride access/egress behind Raptor's decorators, if this is one. (The
   * {@code findOriginal} method throws when the type is not in the chain.)
   */
  public static Optional<HailAndRideAccessEgress> find(RaptorAccessEgress accessEgress) {
    return accessEgress
      .findOriginal(RoutingAccessEgress.class)
      .filter(HailAndRideAccessEgress.class::isInstance)
      .map(HailAndRideAccessEgress.class::cast);
  }

  public HailAndRideRide ride() {
    return ride;
  }

  @Override
  public int stop() {
    return stop;
  }

  @Override
  public int c1() {
    return c1;
  }

  @Override
  public int durationInSeconds() {
    return durationInSeconds;
  }

  @Override
  public int timePenalty() {
    return penalty.isZero() ? RaptorConstants.TIME_NOT_SET : penalty.timeInSeconds();
  }

  @Override
  public int numberOfRides() {
    return 1;
  }

  @Override
  public boolean arrivedOnBoard() {
    return true;
  }

  @Override
  public boolean hasOpeningHours() {
    return true;
  }

  /**
   * The rider leaves the origin (access) or the stop (egress) at time {@code t}; they are at the
   * boarding place {@code preBoardSeconds} later and the vehicle passes one headway after that.
   * That vehicle must run inside a frequency window.
   */
  @Override
  public int earliestDepartureTime(int requestedDepartureTime) {
    return ride.earliestDepartureTime(requestedDepartureTime);
  }

  @Override
  public int latestArrivalTime(int requestedArrivalTime) {
    return ride.latestArrivalTime(requestedArrivalTime, durationInSeconds);
  }

  @Override
  public RoutingAccessEgress withPenalty(TimeAndCost penalty) {
    if (!this.penalty.isZero()) {
      throw new IllegalStateException("Can not add penalty twice...");
    }
    return new HailAndRideAccessEgress(
      stop,
      durationInSeconds,
      c1 + penalty.cost().toCentiSeconds(),
      penalty,
      ride
    );
  }

  @Override
  public State getFinalState() {
    return ride.walkState();
  }

  @Override
  public boolean isWalkOnly() {
    return false;
  }

  @Override
  public TimeAndCost penalty() {
    return penalty;
  }

  @Override
  public String toString() {
    return asString(true, true, ride.toString());
  }
}
