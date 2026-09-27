package org.opentripplanner.ext.hailandride;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Envelope;
import org.opentripplanner.core.model.i18n.NonLocalizedString;
import org.opentripplanner.core.model.id.FeedScopedId;
import org.opentripplanner.model.PickDrop;
import org.opentripplanner.model.ShapePoint;
import org.opentripplanner.model.StopTime;
import org.opentripplanner.model.impl.TransitDataImportBuilder;
import org.opentripplanner.street.graph.Graph;
import org.opentripplanner.street.model.edge.Edge;
import org.opentripplanner.street.model.edge.StreetEdge;
import org.opentripplanner.street.model.vertex.Vertex;
import org.opentripplanner.transit.model.site.RegularStop;
import org.opentripplanner.transit.model.timetable.Trip;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Graph build: add <b>transfer points</b> - extra stops where hail-and-ride routes meet - so that
 * Raptor can change vehicles "anywhere", not only at the feed's real stops.
 * <p>
 * Why only where routes meet: with hail-and-ride, a rider on route P who continues on route Q
 * changes vehicle at the place that minimises
 * <pre>
 *   g(s, t) = timeP(s) + walk(P(s) -> Q(t)) - timeQ(t)
 * </pre>
 * (s, t = positions along P and Q). This does not depend on where the rider got on P or where they
 * will get off Q, so it can be computed once, here, for every ordered pair of routes. We keep the
 * best point of every "contact zone" (stretch where P passes within {@link #WALK_MAX_M} of Q):
 * a crossing gives one point, a shared corridor gives its better end.
 * <p>
 * Boarding and alighting near the origin/destination does not need extra stops at all - that is
 * done per request by {@link HailAndRideRouter}. So these points are the only stops we add, and
 * there are few of them compared with placing synthetic stops at a fixed spacing along every route.
 * <p>
 * A transfer point is only placed where hail-and-ride is allowed on both routes: the GTFS
 * continuous flags allow it and pedestrians can reach the road ({@link HailPointLinker}).
 * <p>
 * Runs inside the GTFS import, after stop times are validated and interpolated and before trip
 * patterns are built; so the new stops and stop times go through OTP's normal pipeline.
 */
public final class HailAndRideTransferPoints {

  private static final Logger LOG = LoggerFactory.getLogger(HailAndRideTransferPoints.class);

  static final double SAMPLE_M = 25.0;
  static final double WALK_MAX_M = 300.0;
  static final double MERGE_M = 100.0;
  static final double SHARE_M = 30.0;
  static final double NEAR_STOP_M = 100.0;
  static final double DETOUR = 1.25;
  static final double WALK_SPEED = 1.33;

  private static final double M_PER_DEG = HailPointLinker.METERS_PER_DEGREE;

  private final TransitDataImportBuilder builder;
  private final HailPointLinker linker;
  private final String feedId;
  private double lat0;
  private double lon0;
  private double cos0;

  private HailAndRideTransferPoints(
    TransitDataImportBuilder builder,
    HailPointLinker linker,
    String feedId
  ) {
    this.builder = builder;
    this.linker = linker;
    this.feedId = feedId;
  }

  public static void run(TransitDataImportBuilder builder, Graph graph, String feedId) {
    long t0 = System.currentTimeMillis();
    var grid = new EdgeGrid(graph);
    new HailAndRideTransferPoints(builder, new HailPointLinker(grid::find), feedId).run();
    LOG.info("Hail-and-ride transfer points done in {} ms", System.currentTimeMillis() - t0);
  }

  /** A distinct (shape, stop sequence) - the trips that will become one trip pattern. */
  private static final class Line {

    final List<Trip> trips = new ArrayList<>();
    List<StopTime> ref;
    Coordinate[] lonlat;
    double[] px;
    double[] py;
    double[] cum;
    double[] stopD;
    // samples
    double[] s;
    double[] sx;
    double[] sy;
    int[] tau;
    boolean[] board;
    boolean[] alight;
    final List<Double> candidates = new ArrayList<>();
    final List<Double> junctions = new ArrayList<>();
    final List<Integer> junctionStop = new ArrayList<>();
  }

  private void run() {
    var lines = collectLines();
    if (lines.isEmpty()) {
      LOG.info("Hail-and-ride transfer points: no trips with shapes and continuous stopping");
      return;
    }
    var first = lines.getFirst().lonlat[0];
    lat0 = first.y;
    lon0 = first.x;
    cos0 = Math.cos(Math.toRadians(lat0));

    int hailable = 0;
    int total = 0;
    for (var line : lines) {
      prepare(line);
      for (int i = 0; i < line.s.length; i++) {
        total++;
        if (line.board[i] || line.alight[i]) {
          hailable++;
        }
      }
    }
    int pairs = findCandidates(lines);
    int stopTimes = 0;
    for (var line : lines) {
      mergeCandidates(line);
      stopTimes += line.junctions.size() * line.trips.size();
    }
    int stops = createSharedStops(lines);
    insertStopTimes(lines);
    LOG.info(
      "Hail-and-ride transfer points: {} lines, {}/{} sample points hail-able, {} route pairs " +
      "in contact, {} transfer stops, {} stop times added",
      lines.size(),
      hailable,
      total,
      pairs,
      stops,
      stopTimes
    );
  }

  /* 1. collect lines */

  private List<Line> collectLines() {
    var byKey = new LinkedHashMap<String, Line>();
    var shapes = builder.getShapePoints();
    var stopTimesByTrip = builder.getStopTimesSortedByTrip();
    for (Trip trip : builder.getTripsById().values()) {
      var shapeId = trip.getShapeId();
      if (shapeId == null || shapes.get(shapeId) == null) {
        continue;
      }
      var sts = stopTimesByTrip.get(trip);
      if (sts == null || sts.size() < 2 || sts.stream().noneMatch(this::continuous)) {
        continue;
      }
      var key = new StringBuilder(shapeId.toString());
      for (var st : sts) {
        key.append('|').append(st.getStop().getId());
      }
      var line = byKey.computeIfAbsent(key.toString(), k -> new Line());
      line.trips.add(trip);
      if (line.ref == null) {
        line.ref = sts;
        var pts = new ArrayList<ShapePoint>();
        shapes.get(shapeId).forEach(pts::add);
        pts.sort(Comparator.comparingInt(ShapePoint::sequence));
        line.lonlat = pts.stream().map(p -> new Coordinate(p.lon(), p.lat())).toArray(Coordinate[]::new);
      }
    }
    return new ArrayList<>(byKey.values());
  }

  private boolean continuous(StopTime st) {
    return allows(st.getFlexContinuousPickup()) || allows(st.getFlexContinuousDropOff());
  }

  private static boolean allows(PickDrop p) {
    return p == PickDrop.SCHEDULED || p == PickDrop.COORDINATE_WITH_DRIVER;
  }

  /* 2. geometry, stop positions, samples */

  private void prepare(Line line) {
    int n = line.lonlat.length;
    line.px = new double[n];
    line.py = new double[n];
    line.cum = new double[n];
    for (int i = 0; i < n; i++) {
      line.px[i] = x(line.lonlat[i].x);
      line.py[i] = y(line.lonlat[i].y);
      if (i > 0) {
        line.cum[i] =
          line.cum[i - 1] + Math.hypot(line.px[i] - line.px[i - 1], line.py[i] - line.py[i - 1]);
      }
    }
    var ref = line.ref;
    line.stopD = new double[ref.size()];
    double prev = 0;
    for (int k = 0; k < ref.size(); k++) {
      var c = ref.get(k).getStop().getCoordinate();
      prev = projectMonotone(line, x(c.longitude()), y(c.latitude()), prev);
      line.stopD[k] = prev;
    }
    double from = line.stopD[0];
    double to = line.stopD[ref.size() - 1];
    int m = Math.max(1, (int) Math.floor((to - from) / SAMPLE_M) + 1);
    line.s = new double[m];
    line.sx = new double[m];
    line.sy = new double[m];
    line.tau = new int[m];
    line.board = new boolean[m];
    line.alight = new boolean[m];
    int dep0 = ref.getFirst().getDepartureTime();
    for (int j = 0; j < m; j++) {
      double d = from + j * SAMPLE_M;
      line.s[j] = d;
      line.sx[j] = interp(line.cum, line.px, d);
      line.sy[j] = interp(line.cum, line.py, d);
      int hop = hopAt(line, d);
      double span = line.stopD[hop + 1] - line.stopD[hop];
      double f = span <= 0 ? 0 : (d - line.stopD[hop]) / span;
      int dep = ref.get(hop).getDepartureTime();
      int arr = ref.get(hop + 1).getArrivalTime();
      line.tau[j] = (int) Math.round(dep + f * (arr - dep)) - dep0;
      var c = new Coordinate(lon(line.sx[j]), lat(line.sy[j]));
      var link = linker.link(c, HailPointLinker.direction(line.lonlat, lonlatCum(line), d));
      boolean ok = link.hailable();
      line.board[j] = ok && allows(ref.get(hop).getFlexContinuousPickup());
      line.alight[j] = ok && allows(ref.get(hop).getFlexContinuousDropOff());
    }
  }

  private final Map<Line, double[]> lonlatCumCache = new HashMap<>();

  /** Cumulative metres along the lon/lat polyline (for HailPointLinker.direction). */
  private double[] lonlatCum(Line line) {
    return lonlatCumCache.computeIfAbsent(line, l -> HailAndRideIndex.cumulativeMeters(l.lonlat));
  }

  private static int hopAt(Line line, double d) {
    int n = line.stopD.length;
    int hop = 0;
    while (hop < n - 2 && line.stopD[hop + 1] <= d) {
      hop++;
    }
    return hop;
  }

  /** Nearest point on the shape at along-distance >= start (keeps stops in order on loops). */
  private static double projectMonotone(Line line, double x, double y, double start) {
    double best = Double.MAX_VALUE;
    double bestD = start;
    for (int i = 0; i < line.px.length - 1; i++) {
      if (line.cum[i + 1] < start) {
        continue;
      }
      double ax = line.px[i];
      double ay = line.py[i];
      double vx = line.px[i + 1] - ax;
      double vy = line.py[i + 1] - ay;
      double len2 = vx * vx + vy * vy;
      double t = len2 == 0 ? 0 : Math.max(0, Math.min(1, ((x - ax) * vx + (y - ay) * vy) / len2));
      double along = line.cum[i] + t * (line.cum[i + 1] - line.cum[i]);
      if (along < start) {
        t = (start - line.cum[i]) / Math.max(1e-9, line.cum[i + 1] - line.cum[i]);
        along = start;
      }
      double dist = Math.hypot(ax + t * vx - x, ay + t * vy - y);
      if (dist < best - 1e-6) {
        best = dist;
        bestD = along;
      }
    }
    return bestD;
  }

  /* 3. best transfer point per contact zone of every ordered pair of lines */

  private int findCandidates(List<Line> lines) {
    // grid of boardable samples
    Map<Long, List<int[]>> grid = new HashMap<>();
    for (int q = 0; q < lines.size(); q++) {
      var line = lines.get(q);
      for (int j = 0; j < line.s.length; j++) {
        if (line.board[j]) {
          grid.computeIfAbsent(cell(line.sx[j], line.sy[j]), k -> new ArrayList<>()).add(new int[] { q, j });
        }
      }
    }
    int pairs = 0;
    for (int p = 0; p < lines.size(); p++) {
      var P = lines.get(p);
      // q -> (sample on P -> {g, t})
      Map<Integer, Map<Integer, double[]>> best = new HashMap<>();
      for (int i = 0; i < P.s.length; i++) {
        if (!P.alight[i]) {
          continue;
        }
        long cx = (long) Math.floor(P.sx[i] / WALK_MAX_M);
        long cy = (long) Math.floor(P.sy[i] / WALK_MAX_M);
        for (long dx = -1; dx <= 1; dx++) {
          for (long dy = -1; dy <= 1; dy++) {
            var list = grid.get(key(cx + dx, cy + dy));
            if (list == null) {
              continue;
            }
            for (int[] qt : list) {
              if (qt[0] == p) {
                continue;
              }
              var Q = lines.get(qt[0]);
              double dist = Math.hypot(P.sx[i] - Q.sx[qt[1]], P.sy[i] - Q.sy[qt[1]]);
              if (dist > WALK_MAX_M) {
                continue;
              }
              double g = P.tau[i] + (DETOUR * dist) / WALK_SPEED - Q.tau[qt[1]];
              var m = best.computeIfAbsent(qt[0], k -> new HashMap<>());
              var cur = m.get(i);
              if (cur == null || g < cur[0]) {
                m.put(i, new double[] { g, qt[1] });
              }
            }
          }
        }
      }
      for (var e : best.entrySet()) {
        pairs++;
        var Q = lines.get(e.getKey());
        var m = e.getValue();
        var idx = new ArrayList<>(m.keySet());
        idx.sort(Integer::compare);
        int zoneStart = 0;
        for (int k = 1; k <= idx.size(); k++) {
          if (k == idx.size() || idx.get(k) - idx.get(k - 1) > 2) {
            int bestI = -1;
            double bestG = Double.MAX_VALUE;
            for (int z = zoneStart; z < k; z++) {
              double g = m.get(idx.get(z))[0];
              if (g < bestG) {
                bestG = g;
                bestI = idx.get(z);
              }
            }
            P.candidates.add(P.s[bestI]);
            Q.candidates.add(Q.s[(int) m.get(bestI)[1]]);
            zoneStart = k;
          }
        }
      }
    }
    return pairs;
  }

  /* 4. merge candidates along each line, skip those near a real stop of the line */

  private static void mergeCandidates(Line line) {
    var pts = new ArrayList<>(line.candidates);
    pts.sort(Double::compare);
    List<List<Double>> clusters = new ArrayList<>();
    for (double d : pts) {
      boolean nearStop = false;
      for (double sd : line.stopD) {
        if (Math.abs(sd - d) < NEAR_STOP_M) {
          nearStop = true;
          break;
        }
      }
      if (nearStop) {
        continue;
      }
      if (!clusters.isEmpty() && d - clusters.getLast().getFirst() < MERGE_M) {
        clusters.getLast().add(d);
      } else {
        var c = new ArrayList<Double>();
        c.add(d);
        clusters.add(c);
      }
    }
    for (var c : clusters) {
      line.junctions.add(c.get(c.size() / 2));
    }
  }

  /* 5. transfer points of different lines within SHARE_M share one stop */

  private int createSharedStops(List<Line> lines) {
    record Pt(Line line, int k, double x, double y) {}
    var pts = new ArrayList<Pt>();
    for (var line : lines) {
      for (int k = 0; k < line.junctions.size(); k++) {
        double d = line.junctions.get(k);
        pts.add(new Pt(line, k, interp(line.cum, line.px, d), interp(line.cum, line.py, d)));
        line.junctionStop.add(-1);
      }
    }
    Map<Long, List<Integer>> grid = new HashMap<>();
    for (int i = 0; i < pts.size(); i++) {
      var p = pts.get(i);
      grid.computeIfAbsent(key(Math.floorDiv((long) p.x, (long) SHARE_M), Math.floorDiv((long) p.y, (long) SHARE_M)), k -> new ArrayList<>()).add(i);
    }
    int[] stopOf = new int[pts.size()];
    java.util.Arrays.fill(stopOf, -1);
    int nextStop = 0;
    for (int i = 0; i < pts.size(); i++) {
      if (stopOf[i] >= 0) {
        continue;
      }
      var p = pts.get(i);
      var members = new ArrayList<Integer>();
      long cx = Math.floorDiv((long) p.x, (long) SHARE_M);
      long cy = Math.floorDiv((long) p.y, (long) SHARE_M);
      for (long dx = -1; dx <= 1; dx++) {
        for (long dy = -1; dy <= 1; dy++) {
          var l = grid.get(key(cx + dx, cy + dy));
          if (l == null) {
            continue;
          }
          for (int j : l) {
            var o = pts.get(j);
            if (stopOf[j] < 0 && Math.hypot(o.x - p.x, o.y - p.y) <= SHARE_M) {
              members.add(j);
            }
          }
        }
      }
      double mx = 0;
      double my = 0;
      for (int j : members) {
        stopOf[j] = nextStop;
        mx += pts.get(j).x;
        my += pts.get(j).y;
        pts.get(j).line.junctionStop.set(pts.get(j).k, nextStop);
      }
      mx /= members.size();
      my /= members.size();
      var stop = builder
        .siteRepository()
        .regularStop(new FeedScopedId(feedId, "HAR_TRANSFER_" + nextStop))
        .withName(new NonLocalizedString("Transfer point " + nextStop))
        .withCoordinate(lat(my), lon(mx))
        .build();
      builder.siteRepository().withRegularStop(stop);
      stops.add(stop);
      nextStop++;
    }
    return nextStop;
  }

  private final List<RegularStop> stops = new ArrayList<>();

  /* 6. splice the new stop times into every trip of the line */

  private void insertStopTimes(List<Line> lines) {
    var stopTimesByTrip = builder.getStopTimesSortedByTrip();
    for (var line : lines) {
      if (line.junctions.isEmpty()) {
        continue;
      }
      // order of junctions along the line
      var order = new ArrayList<Integer>();
      for (int k = 0; k < line.junctions.size(); k++) {
        order.add(k);
      }
      order.sort(Comparator.comparingDouble(line.junctions::get));
      for (Trip trip : line.trips) {
        var orig = stopTimesByTrip.get(trip);
        var out = new ArrayList<StopTime>();
        int next = 0;
        for (int i = 0; i < orig.size(); i++) {
          var st = orig.get(i);
          int baseSeq = st.getStopSequence() * 100;
          st.setStopSequence(baseSeq);
          out.add(st);
          if (i == orig.size() - 1) {
            break;
          }
          int added = 0;
          while (next < order.size() && line.junctions.get(order.get(next)) < line.stopD[i + 1]) {
            int k = order.get(next++);
            double d = line.junctions.get(k);
            if (d <= line.stopD[i]) {
              continue;
            }
            out.add(newStopTime(trip, orig, i, line, d, stops.get(line.junctionStop.get(k)), baseSeq + (++added)));
          }
        }
        stopTimesByTrip.replace(trip, out);
      }
    }
  }

  private static StopTime newStopTime(
    Trip trip,
    List<StopTime> orig,
    int hop,
    Line line,
    double d,
    RegularStop stop,
    int seq
  ) {
    var a = orig.get(hop);
    var b = orig.get(hop + 1);
    double span = line.stopD[hop + 1] - line.stopD[hop];
    double f = span <= 0 ? 0 : (d - line.stopD[hop]) / span;
    int t = (int) Math.round(a.getDepartureTime() + f * (b.getArrivalTime() - a.getDepartureTime()));
    var st = new StopTime();
    st.setTrip(trip);
    st.setStop(stop);
    st.setStopSequence(seq);
    st.setArrivalTime(t);
    st.setDepartureTime(t);
    st.setTimepoint(0);
    st.setPickupType(PickDrop.SCHEDULED);
    st.setDropOffType(PickDrop.SCHEDULED);
    st.setFlexContinuousPickup(a.getFlexContinuousPickup());
    st.setFlexContinuousDropOff(a.getFlexContinuousDropOff());
    st.setStopHeadsign(a.getStopHeadsign());
    if (a.isShapeDistTraveledSet() && b.isShapeDistTraveledSet()) {
      st.setShapeDistTraveled(
        a.getShapeDistTraveled() + f * (b.getShapeDistTraveled() - a.getShapeDistTraveled())
      );
    }
    return st;
  }

  /* helpers */

  private double x(double lon) {
    return (lon - lon0) * cos0 * M_PER_DEG;
  }

  private double y(double lat) {
    return (lat - lat0) * M_PER_DEG;
  }

  private double lon(double x) {
    return lon0 + x / (cos0 * M_PER_DEG);
  }

  private double lat(double y) {
    return lat0 + y / M_PER_DEG;
  }

  private static double interp(double[] cum, double[] v, double d) {
    if (d <= cum[0]) {
      return v[0];
    }
    for (int i = 1; i < cum.length; i++) {
      if (cum[i] >= d) {
        double seg = cum[i] - cum[i - 1];
        double t = seg == 0 ? 0 : (d - cum[i - 1]) / seg;
        return v[i - 1] + t * (v[i] - v[i - 1]);
      }
    }
    return v[v.length - 1];
  }

  private static long cell(double x, double y) {
    return key((long) Math.floor(x / WALK_MAX_M), (long) Math.floor(y / WALK_MAX_M));
  }

  private static long key(long cx, long cy) {
    return (cx << 32) ^ (cy & 0xffffffffL);
  }

  /** Simple spatial grid over all street edges (the graph's own index is not built yet). */
  private static final class EdgeGrid {

    private static final double CELL_DEG = 0.001;
    private final Map<Long, List<StreetEdge>> cells = new HashMap<>();

    EdgeGrid(Graph graph) {
      for (Vertex v : graph.getVertices()) {
        for (Edge e : v.getOutgoing()) {
          if (e instanceof StreetEdge se) {
            var env = se.getGeometry().getEnvelopeInternal();
            for (long cx = cx(env.getMinX()); cx <= cx(env.getMaxX()); cx++) {
              for (long cy = cx(env.getMinY()); cy <= cx(env.getMaxY()); cy++) {
                cells.computeIfAbsent(key(cx, cy), k -> new ArrayList<>()).add(se);
              }
            }
          }
        }
      }
    }

    private static long cx(double deg) {
      return (long) Math.floor(deg / CELL_DEG);
    }

    Iterable<StreetEdge> find(Envelope env) {
      var result = new java.util.LinkedHashSet<StreetEdge>();
      for (long cx = cx(env.getMinX()); cx <= cx(env.getMaxX()); cx++) {
        for (long cy = cx(env.getMinY()); cy <= cx(env.getMaxY()); cy++) {
          var l = cells.get(key(cx, cy));
          if (l != null) {
            result.addAll(l);
          }
        }
      }
      return result;
    }
  }
}
