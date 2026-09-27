package org.opentripplanner.ext.hailandride;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.LineString;
import org.opentripplanner.street.geometry.SphericalDistanceLibrary;
import org.opentripplanner.street.graph.Graph;
import org.opentripplanner.transit.model.network.TripPattern;
import org.opentripplanner.transit.service.TransitService;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Hail-and-ride "board anywhere" points.
 * <p>
 * For every eligible trip pattern we walk along its shape (the hop geometries between stops) and
 * drop a <em>sample point</em> every {@link #SAMPLE_SPACING_M} metres. Each sample remembers
 * <ul>
 *   <li>which hop it is on (between stop {@code i} and stop {@code i+1}) and how far along that
 *   hop it is (fraction by distance) - this is used to interpolate the vehicle's passing time;</li>
 *   <li>the street edge the vehicle drives on and the position along that edge - this is how a
 *   per-request walking search "reaches" the sample.</li>
 * </ul>
 * A sample is only created where hail-and-ride is physically possible:
 * <ul>
 *   <li>GTFS allows it on that hop ({@code continuous_pickup}/{@code continuous_drop_off} on the
 *   stop_time that starts the hop, see {@link HailAndRideEligibility});</li>
 *   <li>the vehicle's road allows pedestrians. The vehicle's road is the nearest car-drivable
 *   street edge that runs roughly parallel to the route shape (the parallel test keeps a flyover
 *   from being matched to the street it crosses). If that road bans walking (motorways, the Lake
 *   Mariout highways, foot=no), there is no sample - nobody can stand there and wave.</li>
 * </ul>
 * Sample points are NOT transit stops. Raptor never sees them.
 * <p>
 * The index is built lazily on the first request and cached for the life of the graph.
 */
public final class HailAndRideIndex {

  private static final Logger LOG = LoggerFactory.getLogger(HailAndRideIndex.class);

  /** Distance between two sample points along a route shape. */
  static final double SAMPLE_SPACING_M = 20.0;

  /** If set, a GeoJSON file with all samples (hail-able or not, and why) is written here. */
  private static final String DEBUG_GEOJSON_PROPERTY = "hailandride.debugGeoJson";

  private static final Object LOCK = new Object();
  private static Graph cachedGraph;
  private static HailAndRideIndex cachedIndex;

  private final Map<Integer, PatternSamples> byRouteIndex;
  private final List<PatternSamples> all;

  private HailAndRideIndex(List<PatternSamples> all) {
    this.all = Collections.unmodifiableList(all);
    var map = new HashMap<Integer, PatternSamples>();
    for (var it : all) {
      map.put(it.routeIndex(), it);
    }
    this.byRouteIndex = map;
  }

  /** Return the index for the given graph, building it on first use. */
  public static HailAndRideIndex of(Graph graph, TransitService transitService) {
    synchronized (LOCK) {
      if (cachedGraph != graph || cachedIndex == null) {
        long t0 = System.currentTimeMillis();
        var stats = new Stats();
        cachedIndex = build(graph, transitService, stats);
        cachedGraph = graph;
        LOG.info(
          "Hail-and-ride index built in {} ms: {} patterns, {} hail points; not hail-able: " +
          "{} no pedestrians on the vehicle's road, {} no street within {} m, {} hop not " +
          "continuous in GTFS; {} hail points without a parallel drivable road (linked to nearest " +
          "walkable street)",
          System.currentTimeMillis() - t0,
          cachedIndex.all.size(),
          stats.ok,
          stats.noPedestrians,
          stats.noStreet,
          (int) HailPointLinker.MAX_LINK_DISTANCE_M,
          stats.notContinuous,
          stats.unmatched
        );
        stats.writeGeoJson();
      }
      return cachedIndex;
    }
  }

  public List<PatternSamples> patterns() {
    return all;
  }

  public PatternSamples forRouteIndex(int routeIndex) {
    return byRouteIndex.get(routeIndex);
  }

  private static HailAndRideIndex build(Graph graph, TransitService transitService, Stats stats) {
    graph.requestIndex();
    var linker = new HailPointLinker(graph::findEdges);
    var list = new ArrayList<PatternSamples>();
    for (TripPattern pattern : transitService.listTripPatterns()) {
      if (!HailAndRideEligibility.isEligible(pattern)) {
        continue;
      }
      var samples = buildSamples(pattern, linker, stats);
      if (samples != null) {
        list.add(samples);
      }
    }
    return new HailAndRideIndex(list);
  }

  private static PatternSamples buildSamples(
    TripPattern pattern,
    HailPointLinker linker,
    Stats stats
  ) {
    int nStops = pattern.numberOfStops();
    if (nStops < 2) {
      return null;
    }
    var b = new PatternSamples.Builder(pattern, nStops);
    double cumulative = 0.0;
    for (int hop = 0; hop < nStops - 1; hop++) {
      b.markHopStart(hop, cumulative);
      LineString geom = pattern.getHopGeometry(hop);
      Coordinate[] cs = geom.getCoordinates();
      double[] cum = cumulativeMeters(cs);
      double length = cum[cum.length - 1];
      boolean pickup = HailAndRideEligibility.hopAllowsPickup(pattern, hop);
      boolean dropOff = HailAndRideEligibility.hopAllowsDropOff(pattern, hop);
      int n = Math.max(1, (int) Math.ceil(length / SAMPLE_SPACING_M));
      for (int m = 0; m < n; m++) {
        double d = length * m / n;
        Coordinate c = interpolate(cs, cum, d);
        if (!pickup && !dropOff) {
          stats.add(pattern, c, "not continuous in GTFS");
          stats.notContinuous++;
          continue;
        }
        double[] dir = HailPointLinker.direction(cs, cum, d);
        float fraction = length == 0 ? 0f : (float) (d / length);
        var link = linker.link(c, dir);
        stats.count(pattern, c, link);
        if (!link.hailable()) {
          continue;
        }
        b.add(
          hop,
          fraction,
          cumulative + d,
          c,
          link.edge(),
          link.edgeFraction(),
          link.distance(),
          pickup,
          dropOff
        );
      }
      cumulative += length;
    }
    b.markHopStart(nStops - 1, cumulative);
    return b.size() == 0 ? null : b.build();
  }

  /* geometry helpers - equirectangular approximation, fine at city scale */

  static double lengthMeters(LineString ls) {
    double[] cum = cumulativeMeters(ls.getCoordinates());
    return cum[cum.length - 1];
  }

  static double[] cumulativeMeters(Coordinate[] cs) {
    double[] cum = new double[cs.length];
    for (int i = 1; i < cs.length; i++) {
      cum[i] = cum[i - 1] + SphericalDistanceLibrary.fastDistance(cs[i - 1], cs[i]);
    }
    return cum;
  }

  static Coordinate interpolate(Coordinate[] cs, double[] cum, double d) {
    if (cs.length == 1 || d <= 0) {
      return new Coordinate(cs[0].x, cs[0].y);
    }
    for (int i = 1; i < cs.length; i++) {
      if (cum[i] >= d) {
        double seg = cum[i] - cum[i - 1];
        double t = seg == 0 ? 0 : (d - cum[i - 1]) / seg;
        return new Coordinate(
          cs[i - 1].x + t * (cs[i].x - cs[i - 1].x),
          cs[i - 1].y + t * (cs[i].y - cs[i - 1].y)
        );
      }
    }
    var last = cs[cs.length - 1];
    return new Coordinate(last.x, last.y);
  }

  /** Counters and optional GeoJSON dump of every sample for inspection. */
  private static final class Stats {

    int ok;
    int noPedestrians;
    int noStreet;
    int notContinuous;
    int unmatched;
    private final String path = System.getProperty(DEBUG_GEOJSON_PROPERTY);
    private final StringBuilder features = new StringBuilder();

    void count(TripPattern p, Coordinate c, HailPointLinker.Link link) {
      switch (link.status()) {
        case OK -> {
          ok++;
          add(p, c, "ok");
        }
        case OK_NO_PARALLEL_ROAD -> {
          ok++;
          unmatched++;
          add(p, c, "ok (no parallel drivable road, nearest walkable street)");
        }
        case NO_PEDESTRIANS -> {
          noPedestrians++;
          add(p, c, "no pedestrians on vehicle road: " + link.edge().getName());
        }
        case NO_STREET -> {
          noStreet++;
          add(p, c, "no street");
        }
      }
    }

    void add(TripPattern p, Coordinate c, String status) {
      if (path == null) {
        return;
      }
      if (!features.isEmpty()) {
        features.append(",\n");
      }
      features.append(
        String.format(
          Locale.ROOT,
          "{\"type\":\"Feature\",\"geometry\":{\"type\":\"Point\",\"coordinates\":[%.6f,%.6f]}," +
          "\"properties\":{\"status\":\"%s\",\"route\":\"%s\",\"pattern\":\"%s\"}}",
          c.x,
          c.y,
          status.replace("\"", "'"),
          p.getRoute().getId().getId(),
          p.getId().getId()
        )
      );
    }

    void writeGeoJson() {
      if (path == null) {
        return;
      }
      try {
        Files.writeString(
          Path.of(path),
          "{\"type\":\"FeatureCollection\",\"features\":[\n" + features + "\n]}\n"
        );
        LOG.info("Hail-and-ride sample points written to {}", path);
      } catch (IOException e) {
        LOG.warn("Could not write {}: {}", path, e.getMessage());
      }
    }
  }
}
