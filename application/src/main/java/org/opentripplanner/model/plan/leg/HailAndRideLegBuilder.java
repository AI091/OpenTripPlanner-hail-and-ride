package org.opentripplanner.model.plan.leg;

import javax.annotation.Nullable;
import org.locationtech.jts.geom.LineString;
import org.opentripplanner.model.plan.Place;

/**
 * Builder for {@link HailAndRideLeg}. Board/alight stop positions must be set to the stops that
 * bracket the hail-and-ride points: the stop at or before the boarding point, and the stop at or
 * after the alighting point.
 */
public class HailAndRideLegBuilder extends FrequencyTransitLegBuilder {

  private Place boardPlace;
  private Place alightPlace;
  private LineString geometry;
  private double distanceMeters;

  public HailAndRideLegBuilder() {}

  public HailAndRideLegBuilder(HailAndRideLeg original) {
    super(original);
    this.boardPlace = original.boardPlace();
    this.alightPlace = original.alightPlace();
    this.geometry = original.legGeometry();
    this.distanceMeters = original.distanceMeters();
  }

  /** The boarding place when boarding between stops, {@code null} if boarding at the stop. */
  public HailAndRideLegBuilder withBoardPlace(@Nullable Place boardPlace) {
    this.boardPlace = boardPlace;
    return this;
  }

  /** The alighting place when alighting between stops, {@code null} if alighting at the stop. */
  public HailAndRideLegBuilder withAlightPlace(@Nullable Place alightPlace) {
    this.alightPlace = alightPlace;
    return this;
  }

  public HailAndRideLegBuilder withGeometry(LineString geometry) {
    this.geometry = geometry;
    return this;
  }

  public HailAndRideLegBuilder withDistanceMeters(double distanceMeters) {
    this.distanceMeters = distanceMeters;
    return this;
  }

  Place boardPlace() {
    return boardPlace;
  }

  Place alightPlace() {
    return alightPlace;
  }

  LineString geometry() {
    return geometry;
  }

  double distanceMeters() {
    return distanceMeters;
  }

  @Override
  public HailAndRideLeg build() {
    return new HailAndRideLeg(this);
  }
}
