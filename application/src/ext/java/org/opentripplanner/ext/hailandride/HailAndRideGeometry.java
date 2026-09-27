package org.opentripplanner.ext.hailandride;

import java.util.ArrayList;
import java.util.List;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.LineString;
import org.opentripplanner.street.geometry.GeometryUtils;
import org.opentripplanner.transit.model.network.TripPattern;

/** Cut the part of a pattern's shape between two (hop, fraction) positions. */
final class HailAndRideGeometry {

  private HailAndRideGeometry() {}

  /**
   * @param fromHop hop index of the start; a stop position {@code s} is (s, 0)
   * @param toHop   hop index of the end; a stop position {@code s} is (s, 0)
   */
  static LineString between(
    TripPattern pattern,
    int fromHop,
    double fromFraction,
    int toHop,
    double toFraction
  ) {
    // An end exactly at stop s (s > 0) is the end of hop s-1.
    if (toFraction == 0 && toHop > 0) {
      toHop = toHop - 1;
      toFraction = 1.0;
    }
    List<Coordinate> out = new ArrayList<>();
    for (int hop = fromHop; hop <= toHop; hop++) {
      Coordinate[] cs = pattern.getHopGeometry(hop).getCoordinates();
      double[] cum = HailAndRideIndex.cumulativeMeters(cs);
      double length = cum[cum.length - 1];
      double d0 = (hop == fromHop ? fromFraction : 0) * length;
      double d1 = (hop == toHop ? toFraction : 1) * length;
      add(out, HailAndRideIndex.interpolate(cs, cum, d0));
      for (int i = 0; i < cs.length; i++) {
        if (cum[i] > d0 && cum[i] < d1) {
          add(out, cs[i]);
        }
      }
      add(out, HailAndRideIndex.interpolate(cs, cum, d1));
    }
    if (out.size() == 1) {
      out.add(out.getFirst());
    }
    return GeometryUtils.makeLineString(out);
  }

  private static void add(List<Coordinate> out, Coordinate c) {
    if (out.isEmpty() || !out.getLast().equals2D(c)) {
      out.add(new Coordinate(c.x, c.y));
    }
  }
}
