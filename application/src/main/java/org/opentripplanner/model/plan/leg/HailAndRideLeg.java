package org.opentripplanner.model.plan.leg;

import java.util.ArrayList;
import java.util.List;
import javax.annotation.Nullable;
import org.locationtech.jts.geom.LineString;
import org.opentripplanner.model.plan.Leg;
import org.opentripplanner.model.plan.Place;
import org.opentripplanner.transit.model.site.StopLocation;
import org.opentripplanner.utils.time.ServiceDateUtils;

/**
 * A ride on a frequency-based (headway) trip where the rider boards and/or alights between two
 * stops - hail-and-ride / GTFS continuous stopping.
 * <p>
 * {@link #boardStopPosInPattern} is the stop at or before the boarding point and
 * {@link #alightStopPosInPattern} the stop at or after the alighting point. When the rider boards
 * or alights exactly at a stop, the corresponding place override is {@code null} and the stop is
 * used, exactly as for a normal transit leg.
 * <p>
 * The trip times must be materialized so that they are the real (pessimistic, headway-wait
 * included) passing times; intermediate stops are reported without further headway shifting.
 */
public class HailAndRideLeg extends FrequencyTransitLeg {

  @Nullable
  private final Place boardPlace;

  @Nullable
  private final Place alightPlace;

  private final LineString geometry;
  private final double distanceMeters;

  HailAndRideLeg(HailAndRideLegBuilder builder) {
    super(builder);
    this.boardPlace = builder.boardPlace();
    this.alightPlace = builder.alightPlace();
    this.geometry = builder.geometry();
    this.distanceMeters = builder.distanceMeters();
  }

  @Override
  public HailAndRideLegBuilder copyOf() {
    return new HailAndRideLegBuilder(this);
  }

  @Nullable
  Place boardPlace() {
    return boardPlace;
  }

  @Nullable
  Place alightPlace() {
    return alightPlace;
  }

  /** True if the rider waves the vehicle down between two stops. */
  public boolean isHailedBetweenStops() {
    return boardPlace != null;
  }

  /** True if the rider gets off between two stops. */
  public boolean isAlightingBetweenStops() {
    return alightPlace != null;
  }

  @Override
  public Place from() {
    return boardPlace != null ? boardPlace : super.from();
  }

  @Override
  public Place to() {
    return alightPlace != null ? alightPlace : super.to();
  }

  @Override
  public LineString legGeometry() {
    return geometry != null ? geometry : super.legGeometry();
  }

  @Override
  public double distanceMeters() {
    return distanceMeters;
  }

  @Override
  public boolean isPartiallySameTransitLeg(Leg other) {
    if (!(other instanceof HailAndRideLeg)) {
      return false;
    }
    return super.isPartiallySameTransitLeg(other);
  }

  @Override
  public List<StopArrival> listIntermediateStops() {
    List<StopArrival> visits = new ArrayList<>();
    for (int i = boardStopPosInPattern + 1; i < alightStopPosInPattern; i++) {
      StopLocation stop = tripPattern().getStop(i);
      visits.add(
        new StopArrival(
          Place.forStop(stop),
          LegCallTime.ofStatic(
            ServiceDateUtils.toZonedDateTime(serviceDate(), zoneId(), tripTimes().getArrivalTime(i))
          ),
          LegCallTime.ofStatic(
            ServiceDateUtils.toZonedDateTime(
              serviceDate(),
              zoneId(),
              tripTimes().getDepartureTime(i)
            )
          ),
          i,
          tripTimes().gtfsSequenceOfStopIndex(i),
          null
        )
      );
    }
    return visits;
  }
}
