package org.opentripplanner.ext.hailandride;

import java.util.function.Function;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Envelope;
import org.opentripplanner.street.model.edge.Edge;
import org.opentripplanner.street.model.edge.StreetEdge;
import org.opentripplanner.street.search.TraverseMode;

/**
 * Decides whether a rider can stand at a point of a route shape and wave the vehicle down, and
 * which street edge they walk on to get there.
 * <ol>
 *   <li>The vehicle's road is the nearest car-drivable street edge within
 *   {@link #MAX_LINK_DISTANCE_M} that runs roughly parallel to the route shape. (The parallel test
 *   keeps a flyover from being matched to the street it crosses.)</li>
 *   <li>If the vehicle's road allows walking, the point is hail-able and linked to that road.</li>
 *   <li>If the vehicle's road bans walking (motorway, foot=no, e.g. the Lake Mariout highways), the
 *   point is NOT hail-able.</li>
 *   <li>If no drivable road runs along the shape here (shape and OSM disagree), fall back to the
 *   nearest walkable edge within {@link #MAX_LINK_DISTANCE_M}.</li>
 * </ol>
 */
final class HailPointLinker {

  /** Max distance between the route shape and the road the vehicle is assumed to drive on. */
  static final double MAX_LINK_DISTANCE_M = 30.0;

  /** |cos(angle)| between shape and edge above which the edge counts as "the same road". */
  private static final double PARALLEL_COS = 0.8;

  static final double METERS_PER_DEGREE = 111_195.0;

  enum Status {
    OK,
    OK_NO_PARALLEL_ROAD,
    NO_PEDESTRIANS,
    NO_STREET,
  }

  record Link(Status status, StreetEdge edge, float edgeFraction, float distance) {
    boolean hailable() {
      return status == Status.OK || status == Status.OK_NO_PARALLEL_ROAD;
    }
  }

  record Projection(double distance, double fraction, double dirX, double dirY) {}

  private final Function<Envelope, Iterable<? extends Edge>> edgesInEnvelope;

  HailPointLinker(Function<Envelope, Iterable<? extends Edge>> edgesInEnvelope) {
    this.edgesInEnvelope = edgesInEnvelope;
  }

  /**
   * @param c   the point on the route shape (lon/lat)
   * @param dir unit direction of the shape at c as (east, north), or {0,0} if unknown
   */
  Link link(Coordinate c, double[] dir) {
    double cosLat = Math.cos(Math.toRadians(c.y));
    double dLat = MAX_LINK_DISTANCE_M / METERS_PER_DEGREE;
    double dLon = dLat / cosLat;
    var env = new Envelope(c.x - dLon, c.x + dLon, c.y - dLat, c.y + dLat);
    boolean haveDir = dir[0] != 0 || dir[1] != 0;

    StreetEdge vehicleRoad = null;
    Projection vehicleProj = null;
    StreetEdge walkEdge = null;
    Projection walkProj = null;
    for (Edge e : edgesInEnvelope.apply(env)) {
      if (!(e instanceof StreetEdge se)) {
        continue;
      }
      var p = project(se, c, cosLat);
      if (p.distance > MAX_LINK_DISTANCE_M) {
        continue;
      }
      boolean parallel = !haveDir || Math.abs(p.dirX * dir[0] + p.dirY * dir[1]) >= PARALLEL_COS;
      if (
        parallel &&
        se.getPermission().allows(TraverseMode.CAR) &&
        (vehicleProj == null || p.distance < vehicleProj.distance)
      ) {
        vehicleRoad = se;
        vehicleProj = p;
      }
      if (
        se.getPermission().allows(TraverseMode.WALK) &&
        (walkProj == null || p.distance < walkProj.distance)
      ) {
        walkEdge = se;
        walkProj = p;
      }
    }
    if (vehicleRoad != null) {
      if (vehicleRoad.getPermission().allows(TraverseMode.WALK)) {
        return new Link(
          Status.OK,
          vehicleRoad,
          (float) vehicleProj.fraction,
          (float) vehicleProj.distance
        );
      }
      return new Link(Status.NO_PEDESTRIANS, vehicleRoad, 0, (float) vehicleProj.distance);
    }
    if (walkEdge != null) {
      return new Link(
        Status.OK_NO_PARALLEL_ROAD,
        walkEdge,
        (float) walkProj.fraction,
        (float) walkProj.distance
      );
    }
    return new Link(Status.NO_STREET, null, 0, 0);
  }

  /** Nearest point of an edge to c: distance (m), fraction along the edge, local direction. */
  static Projection project(StreetEdge se, Coordinate c, double cosLat) {
    Coordinate[] g = se.getGeometry().getCoordinates();
    double total = 0;
    double[] segLen = new double[g.length - 1];
    for (int i = 0; i < g.length - 1; i++) {
      double vx = (g[i + 1].x - g[i].x) * cosLat * METERS_PER_DEGREE;
      double vy = (g[i + 1].y - g[i].y) * METERS_PER_DEGREE;
      segLen[i] = Math.hypot(vx, vy);
      total += segLen[i];
    }
    double best = Double.MAX_VALUE;
    double bestAlong = 0;
    double bx = 0;
    double by = 0;
    double along = 0;
    for (int i = 0; i < g.length - 1; i++) {
      double ax = (g[i].x - c.x) * cosLat * METERS_PER_DEGREE;
      double ay = (g[i].y - c.y) * METERS_PER_DEGREE;
      double vx = (g[i + 1].x - g[i].x) * cosLat * METERS_PER_DEGREE;
      double vy = (g[i + 1].y - g[i].y) * METERS_PER_DEGREE;
      double len2 = vx * vx + vy * vy;
      double t = len2 == 0 ? 0 : Math.max(0, Math.min(1, (-ax * vx - ay * vy) / len2));
      double dist = Math.hypot(ax + t * vx, ay + t * vy);
      if (dist < best) {
        best = dist;
        bestAlong = along + t * segLen[i];
        double l = Math.sqrt(len2);
        bx = l == 0 ? 0 : vx / l;
        by = l == 0 ? 0 : vy / l;
      }
      along += segLen[i];
    }
    return new Projection(best, total == 0 ? 0 : bestAlong / total, bx, by);
  }

  /** Unit direction (east, north) of a polyline around distance d; {0,0} if degenerate. */
  static double[] direction(Coordinate[] cs, double[] cum, double d) {
    // Use a window of +-15 m around the point to smooth out tiny zig-zags in the shape.
    Coordinate a = HailAndRideIndex.interpolate(cs, cum, Math.max(0, d - 15));
    Coordinate b = HailAndRideIndex.interpolate(cs, cum, Math.min(cum[cum.length - 1], d + 15));
    double cosLat = Math.cos(Math.toRadians(a.y));
    double dx = (b.x - a.x) * cosLat;
    double dy = b.y - a.y;
    double len = Math.hypot(dx, dy);
    return len == 0 ? new double[] { 0, 0 } : new double[] { dx / len, dy / len };
  }
}
