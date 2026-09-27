package org.opentripplanner.ext.hailandride;

import java.time.Duration;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import org.locationtech.jts.geom.Envelope;
import org.opentripplanner.astar.model.ShortestPathTree;
import org.opentripplanner.astar.strategy.DurationSkipEdgeStrategy;
import org.opentripplanner.framework.application.OTPRequestTimeoutException;
import org.opentripplanner.routing.algorithm.raptoradapter.transit.RoutingAccessEgress;
import org.opentripplanner.routing.algorithm.raptoradapter.transit.request.RaptorRoutingRequestTransitData;
import org.opentripplanner.routing.algorithm.raptoradapter.transit.request.TripPatternForDates;
import org.opentripplanner.routing.api.request.RouteRequest;
import org.opentripplanner.routing.cost.CostLimit;
import org.opentripplanner.routing.linking.LinkingContext;
import org.opentripplanner.street.graph.Graph;
import org.opentripplanner.street.model.StreetMode;
import org.opentripplanner.street.model.edge.Edge;
import org.opentripplanner.street.model.edge.StreetEdge;
import org.opentripplanner.street.model.vertex.Vertex;
import org.opentripplanner.street.search.StreetSearchBuilder;
import org.opentripplanner.street.search.state.State;
import org.opentripplanner.street.search.strategy.DominanceFunctions;
import org.opentripplanner.streetadapter.StreetSearchRequestMapper;
import org.opentripplanner.transit.model.timetable.FrequencyEntry;
import org.opentripplanner.transit.model.timetable.TripTimes;
import org.opentripplanner.transit.service.TransitService;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Per-request hail-and-ride routing. Creates:
 * <ul>
 *   <li><b>access</b> paths: walk to the best point on a nearby route, wave it down, ride to a
 *   stop k (one path per reachable downstream stop);</li>
 *   <li><b>egress</b> paths: board a route at stop k, ride, get off at the best point near the
 *   destination, walk;</li>
 *   <li><b>direct</b> rides: one route from near the origin to near the destination, no stops
 *   involved at all.</li>
 * </ul>
 * The access and egress paths are handed to Raptor like FLEX paths (one ride, arriving on board),
 * so Raptor can combine them with normal transit and transfers at stops.
 * <p>
 * The "best point" is chosen continuously along the route (20 m resolution): for access to stop
 * k we minimise {@code walk(p) + headway + ride(p -> k)}; for egress from stop k we minimise
 * {@code ride(k -> q) + walk(q)}.
 */
public final class HailAndRideRouter {

  private static final Logger LOG = LoggerFactory.getLogger(HailAndRideRouter.class);

  /** Optional override of the hail-and-ride walk limit, for experiments. */
  private static final String MAX_WALK_PROPERTY = "hailandride.maxWalkMinutes";

  private final HailAndRideIndex index;
  private final RouteRequest request;
  private final RaptorRoutingRequestTransitData transitData;
  private final LinkingContext linkingContext;
  private final Duration maxWalk;
  private final double walkSpeed;
  private final double walkReluctance;
  private final int boardCost;

  private volatile Map<Integer, PatternReach> accessReach;
  private volatile Map<Integer, PatternReach> egressReach;

  public HailAndRideRouter(
    Graph graph,
    TransitService transitService,
    RouteRequest request,
    RaptorRoutingRequestTransitData transitData,
    LinkingContext linkingContext
  ) {
    this.index = HailAndRideIndex.of(graph, transitService);
    this.request = request;
    this.transitData = transitData;
    this.linkingContext = linkingContext;
    var prop = System.getProperty(MAX_WALK_PROPERTY);
    this.maxWalk = prop != null
      ? Duration.ofMinutes(Long.parseLong(prop))
      : request.preferences().street().accessEgress().maxDuration().valueOf(StreetMode.WALK);
    this.walkSpeed = request.preferences().walk().speed();
    this.walkReluctance = request.preferences().walk().reluctance();
    this.boardCost = request.preferences().walk().boardCost();
  }

  /** Per sample: how the rider can walk to (access) or from (egress) it. */
  record PatternReach(PatternSamples samples, WalkReach[] bySample) {}

  /** A street edge identified by its two end vertices, in either direction. */
  record EdgeKey(Vertex a, Vertex b) {
    static EdgeKey of(Edge e) {
      var x = e.getFromVertex();
      var y = e.getToVertex();
      return x.getLabelString().compareTo(y.getLabelString()) <= 0
        ? new EdgeKey(x, y)
        : new EdgeKey(y, x);
    }
  }

  /** Where the walking search started in the middle of a street edge, and the state there. */
  record Split(org.locationtech.jts.geom.Coordinate at, State state) {}

  /**
   * The origin (or destination) is linked into the street network by splitting nearby street
   * edges with temporary vertices. Collect those split points with their search state, keyed by
   * the edge they split, so samples on the same edge can be reached along the edge.
   */
  private static Map<EdgeKey, List<Split>> splitPoints(
    Set<Vertex> origin,
    ShortestPathTree<State, Edge, Vertex> spt
  ) {
    var result = new HashMap<EdgeKey, List<Split>>();
    var seen = new java.util.HashSet<Vertex>();
    var queue = new java.util.ArrayDeque<Vertex>(origin);
    while (!queue.isEmpty()) {
      var v = queue.poll();
      if (!seen.add(v) || seen.size() > 50) {
        continue;
      }
      var state = spt.getState(v);
      var edges = new ArrayList<Edge>(v.getOutgoing());
      edges.addAll(v.getIncoming());
      for (Edge e : edges) {
        if (e instanceof org.opentripplanner.street.model.edge.TemporaryPartialStreetEdge tp) {
          if (state != null && v instanceof org.opentripplanner.street.model.vertex.TemporaryVertex) {
            result
              .computeIfAbsent(EdgeKey.of(tp.getParentEdge()), k -> new ArrayList<>())
              .add(new Split(v.getCoordinate(), state));
          }
        }
        var other = e.getFromVertex() == v ? e.getToVertex() : e.getFromVertex();
        if (other instanceof org.opentripplanner.street.model.vertex.TemporaryVertex) {
          queue.add(other);
        }
      }
    }
    return result;
  }

  /** A frequency entry of a pattern and its service windows in this request. */
  private record Service(FrequencyEntry entry, ServiceWindows windows) {}

  public List<RoutingAccessEgress> findAccess() {
    var reach = search(true);
    this.accessReach = reach;
    return build(reach, true);
  }

  public List<RoutingAccessEgress> findEgress() {
    var reach = search(false);
    this.egressReach = reach;
    return build(reach, false);
  }

  /**
   * Best single-route rides from near the origin to near the destination. Must be called after
   * {@link #findAccess()} and {@link #findEgress()}. Returns at most one ride per pattern and at
   * most {@code maxResults} rides, best (earliest arrival) first.
   */
  public List<HailAndRideRide> findDirect(int departureTime, int maxResults) {
    if (accessReach == null || egressReach == null) {
      return List.of();
    }
    var result = new ArrayList<HailAndRideRide>();
    for (var e : accessReach.entrySet()) {
      var to = egressReach.get(e.getKey());
      if (to == null) {
        continue;
      }
      var from = e.getValue();
      HailAndRideRide best = null;
      int bestArrival = Integer.MAX_VALUE;
      for (var service : services(from.samples().routeIndex())) {
        var ride = bestDirect(from, to, service, departureTime);
        if (ride == null) {
          continue;
        }
        int dep = ride.earliestDepartureTime(departureTime);
        if (dep == org.opentripplanner.raptor.spi.RaptorConstants.TIME_NOT_SET) {
          continue;
        }
        int arrival = dep + ride.durationSeconds();
        if (arrival < bestArrival) {
          bestArrival = arrival;
          best = ride;
        }
      }
      if (best != null) {
        result.add(best);
      }
    }
    result.sort(Comparator.comparingInt(HailAndRideRide::durationSeconds));
    return result.size() > maxResults ? result.subList(0, maxResults) : result;
  }

  /** Generalized cost of a direct ride in seconds, computed like the access/egress cost. */
  public int directCostSeconds(HailAndRideRide ride) {
    double walk =
      (ride.walkBefore() == null ? 0 : ride.walkBefore().weight()) +
      (ride.walkAfter() == null ? 0 : ride.walkAfter().weight());
    double transit =
      transitReluctance(ride.samples().pattern()) * (ride.headway() + ride.rideSeconds());
    return (int) Math.round(walk + boardCost + transit);
  }

  /* street search */

  private Map<Integer, PatternReach> search(boolean access) {
    OTPRequestTimeoutException.checkForTimeout();
    Set<Vertex> origin = access
      ? linkingContext.findVertices(request.from())
      : linkingContext.findVertices(request.to());
    if (origin == null || origin.isEmpty()) {
      return Map.of();
    }
    var streetRequest = StreetSearchRequestMapper.map(request)
      .withMode(StreetMode.WALK)
      .withArriveBy(!access)
      .build();
    ShortestPathTree<State, Edge, Vertex> spt = StreetSearchBuilder.of()
      .withPreStartHook(OTPRequestTimeoutException::checkForTimeout)
      .withSkipEdgeStrategy(new DurationSkipEdgeStrategy<>(maxWalk))
      .withDominanceFunction(new DominanceFunctions.MinimumWeight())
      .withRequest(streetRequest)
      .withFrom(access ? origin : null)
      .withTo(access ? null : origin)
      .getShortestPathTree();
    var splits = splitPoints(origin, spt);

    // Only look at samples inside the box that can possibly be reached.
    var env = new Envelope();
    for (var v : origin) {
      env.expandToInclude(v.getLon(), v.getLat());
    }
    double radius = maxWalk.toSeconds() * walkSpeed;
    double dLat = radius / 111_195.0;
    double dLon = dLat / Math.cos(Math.toRadians(env.centre().y));
    env.expandBy(dLon, dLat);

    var result = new HashMap<Integer, PatternReach>();
    String debugRoute = System.getProperty("hailandride.debugRoute");
    for (PatternSamples ps : index.patterns()) {
      if (debugRoute != null && ps.pattern().getRoute().getId().getId().equals(debugRoute)) {
        LOG.info(
          "DEBUG search {} {}: active={} envelopeHit={}",
          access ? "access" : "egress",
          ps.pattern().getId(),
          transitData.getRouteForIndex(ps.routeIndex()) != null,
          ps.envelope().intersects(env)
        );
      }
      if (transitData.getRouteForIndex(ps.routeIndex()) == null) {
        // not running on the search days, or filtered away by the request (modes, banned...)
        continue;
      }
      if (!ps.envelope().intersects(env)) {
        continue;
      }
      WalkReach[] bySample = null;
      for (int j = 0; j < ps.size(); j++) {
        if (!env.contains(ps.lon(j), ps.lat(j))) {
          continue;
        }
        var reach = reachSample(spt, splits, ps, j);
        if (reach == null) {
          continue;
        }
        if (bySample == null) {
          bySample = new WalkReach[ps.size()];
        }
        bySample[j] = reach;
      }
      if (debugRoute != null && ps.pattern().getRoute().getId().getId().equals(debugRoute)) {
        var o = origin.iterator().next();
        int nearest = -1;
        double nd = Double.MAX_VALUE;
        for (int j = 0; j < ps.size(); j++) {
          double d = Math.hypot(
            (ps.lon(j) - o.getLon()) * Math.cos(Math.toRadians(o.getLat())),
            ps.lat(j) - o.getLat()
          );
          if (d < nd) {
            nd = d;
            nearest = j;
          }
        }
        if (nearest >= 0) {
          var e = ps.edge(nearest);
          LOG.info(
            "DEBUG nearest sample s{} ({} m): edge {} {} perm {} from-state {} to-state {} reach {}",
            nearest,
            Math.round(nd * 111195),
            e.getDefaultName(),
            e,
            e.getPermission(),
            spt.getState(e.getFromVertex()),
            spt.getState(e.getToVertex()),
            bySample == null ? null : bySample[nearest]
          );
        }
      }
      if (bySample != null) {
        result.put(ps.routeIndex(), new PatternReach(ps, bySample));
      }
    }
    return result;
  }

  /**
   * The sample lies at a fraction f along a street edge a->b. The rider reaches it via a (then
   * walks f * length) or via b (then walks (1-f) * length). The same holds for the reverse
   * (egress) search: from the sample to a or b, then on to the destination.
   */
  private WalkReach reachSample(
    ShortestPathTree<State, Edge, Vertex> spt,
    Map<EdgeKey, List<Split>> splits,
    PatternSamples ps,
    int j
  ) {
    StreetEdge edge = ps.edge(j);
    double length = edge.getDistanceMeters();
    double f = ps.edgeFraction(j);
    double link = ps.linkDistance(j);
    WalkReach best = null;
    State sa = spt.getState(edge.getFromVertex());
    if (sa != null) {
      best = candidate(sa, f * length + link, j);
    }
    State sb = spt.getState(edge.getToVertex());
    if (sb != null) {
      var c = candidate(sb, (1 - f) * length + link, j);
      if (best == null || c.weight() < best.weight()) {
        best = c;
      }
    }
    // The origin/destination itself may lie on this edge (long edges without intersections, e.g.
    // a 4.6 km frontage road): then walk along the edge directly from where the search started.
    var onEdge = splits.get(EdgeKey.of(edge));
    if (onEdge != null) {
      double cosLat = Math.cos(Math.toRadians(ps.lat(j)));
      for (var split : onEdge) {
        double fs = HailPointLinker.project(edge, split.at(), cosLat).fraction();
        var c = candidate(split.state(), Math.abs(f - fs) * length + link, j);
        if (best == null || c.weight() < best.weight()) {
          best = c;
        }
      }
    }
    if (best != null && best.seconds() > maxWalk.toSeconds()) {
      return null;
    }
    return best;
  }

  private WalkReach candidate(State s, double extraMeters, int sample) {
    double extraSec = extraMeters / walkSpeed;
    int seconds = (int) Math.round(s.getElapsedTimeSeconds() + extraSec);
    double weight = s.getWeight() + extraSec * walkReluctance;
    return new WalkReach(s, seconds, weight, extraMeters, sample);
  }

  /* access / egress construction */

  private List<RoutingAccessEgress> build(Map<Integer, PatternReach> reach, boolean access) {
    // Keep, per Raptor stop, the fastest and the cheapest candidate.
    Map<Integer, HailAndRideAccessEgress> fastest = new HashMap<>();
    Map<Integer, HailAndRideAccessEgress> cheapest = new HashMap<>();
    String debugRoute = System.getProperty("hailandride.debugRoute");
    for (var pr : reach.values()) {
      boolean debug =
        debugRoute != null && pr.samples().pattern().getRoute().getId().getId().equals(debugRoute);
      if (debug) {
        int reached = 0;
        for (var w : pr.bySample()) {
          if (w != null) {
            reached++;
          }
        }
        LOG.info(
          "DEBUG {} {}: {} samples reached, {} services",
          access ? "access" : "egress",
          pr.samples().pattern().getId(),
          reached,
          services(pr.samples().routeIndex()).size()
        );
      }
      for (var service : services(pr.samples().routeIndex())) {
        var list = access ? accessFor(pr, service) : egressFor(pr, service);
        if (debug) {
          for (var ae : list) {
            LOG.info("DEBUG   {} -> stop {} {}s", ae.ride(), ae.stop(), ae.durationInSeconds());
          }
        }
        for (var ae : list) {
          fastest.merge(ae.stop(), ae, (a, b) ->
            b.durationInSeconds() < a.durationInSeconds() ||
              (b.durationInSeconds() == a.durationInSeconds() && b.c1() < a.c1())
              ? b
              : a
          );
          cheapest.merge(ae.stop(), ae, (a, b) -> b.c1() < a.c1() ? b : a);
        }
      }
    }
    var result = new ArrayList<RoutingAccessEgress>(fastest.values());
    for (var e : cheapest.entrySet()) {
      if (fastest.get(e.getKey()) != e.getValue()) {
        result.add(e.getValue());
      }
    }
    LOG.debug(
      "Hail-and-ride {}: {} patterns reached, {} paths",
      access ? "access" : "egress",
      reach.size(),
      result.size()
    );
    return result;
  }

  private List<HailAndRideAccessEgress> accessFor(PatternReach pr, Service service) {
    var ps = pr.samples();
    var pattern = ps.pattern();
    var rtp = pattern.getRoutingTripPattern();
    TripTimes tt = service.entry().tripTimes();
    int headway = service.entry().routingSlack();
    double reluctance = transitReluctance(pattern);
    int nStops = pattern.numberOfStops();

    var result = new ArrayList<HailAndRideAccessEgress>();
    int best = -1;
    int bestKey = Integer.MAX_VALUE;
    int j = 0;
    for (int k = 1; k < nStops; k++) {
      int end = ps.firstSampleAtOrAfterStop(k);
      for (; j < end; j++) {
        var w = pr.bySample()[j];
        // A sample exactly at a stop is covered by normal walk access + Raptor boarding.
        if (w == null || ps.hopFraction(j) == 0f || !ps.canBoard(j)) {
          continue;
        }
        int key = w.seconds() - ps.vehicleOffset(tt, j);
        if (key < bestKey) {
          bestKey = key;
          best = j;
        }
      }
      if (best < 0 || !rtp.alightingPossibleAt(k)) {
        continue;
      }
      var w = pr.bySample()[best];
      int boardOffset = ps.vehicleOffset(tt, best);
      int alightOffset = PatternSamples.stopArrivalOffset(tt, k);
      if (alightOffset < boardOffset) {
        continue;
      }
      var ride = new HailAndRideRide(
        HailAndRideRide.Kind.ACCESS,
        ps,
        service.entry(),
        service.windows(),
        best,
        -1,
        -1,
        k,
        boardOffset,
        alightOffset,
        w,
        null
      );
      int c1 = CostLimit.toRaptorCost(
        w.weight() + boardCost + reluctance * (headway + ride.rideSeconds())
      );
      result.add(
        new HailAndRideAccessEgress(rtp.stopIndex(k), ride.durationSeconds(), c1, ride)
      );
    }
    return result;
  }

  private List<HailAndRideAccessEgress> egressFor(PatternReach pr, Service service) {
    var ps = pr.samples();
    var pattern = ps.pattern();
    var rtp = pattern.getRoutingTripPattern();
    TripTimes tt = service.entry().tripTimes();
    int headway = service.entry().routingSlack();
    double reluctance = transitReluctance(pattern);
    int nStops = pattern.numberOfStops();

    var result = new ArrayList<HailAndRideAccessEgress>();
    int best = -1;
    int bestKey = Integer.MAX_VALUE;
    int j = ps.size() - 1;
    for (int k = nStops - 2; k >= 0; k--) {
      int start = ps.firstSampleAtOrAfterStop(k);
      for (; j >= start; j--) {
        var w = pr.bySample()[j];
        if (w == null || ps.hopFraction(j) == 0f || !ps.canAlight(j)) {
          continue;
        }
        int key = ps.vehicleOffset(tt, j) + w.seconds();
        if (key < bestKey) {
          bestKey = key;
          best = j;
        }
      }
      if (best < 0 || !rtp.boardingPossibleAt(k)) {
        continue;
      }
      var w = pr.bySample()[best];
      int boardOffset = PatternSamples.stopDepartureOffset(tt, k);
      int alightOffset = ps.vehicleOffset(tt, best);
      if (alightOffset < boardOffset) {
        continue;
      }
      var ride = new HailAndRideRide(
        HailAndRideRide.Kind.EGRESS,
        ps,
        service.entry(),
        service.windows(),
        -1,
        k,
        best,
        -1,
        boardOffset,
        alightOffset,
        null,
        w
      );
      int c1 = CostLimit.toRaptorCost(
        w.weight() + boardCost + reluctance * (headway + ride.rideSeconds())
      );
      result.add(
        new HailAndRideAccessEgress(rtp.stopIndex(k), ride.durationSeconds(), c1, ride)
      );
    }
    return result;
  }

  private HailAndRideRide bestDirect(
    PatternReach from,
    PatternReach to,
    Service service,
    int departureTime
  ) {
    var ps = from.samples();
    TripTimes tt = service.entry().tripTimes();
    int bestBoard = -1;
    int bestBoardKey = Integer.MAX_VALUE;
    int bestTotal = Integer.MAX_VALUE;
    int bestA = -1;
    int bestB = -1;
    for (int j = 0; j < ps.size(); j++) {
      var d = to.bySample()[j];
      if (d != null && bestBoard >= 0 && ps.canAlight(j)) {
        int total = bestBoardKey + ps.vehicleOffset(tt, j) + d.seconds();
        if (total < bestTotal) {
          bestTotal = total;
          bestA = bestBoard;
          bestB = j;
        }
      }
      var o = from.bySample()[j];
      if (o != null && ps.canBoard(j)) {
        int key = o.seconds() - ps.vehicleOffset(tt, j);
        if (key < bestBoardKey) {
          bestBoardKey = key;
          bestBoard = j;
        }
      }
    }
    if (bestA < 0) {
      return null;
    }
    return new HailAndRideRide(
      HailAndRideRide.Kind.DIRECT,
      ps,
      service.entry(),
      service.windows(),
      bestA,
      -1,
      bestB,
      -1,
      ps.vehicleOffset(tt, bestA),
      ps.vehicleOffset(tt, bestB),
      from.bySample()[bestA],
      to.bySample()[bestB]
    );
  }

  /* helpers */

  private double transitReluctance(org.opentripplanner.transit.model.network.TripPattern p) {
    return request.preferences().transit().reluctanceForMode().getOrDefault(p.getMode(), 1.0);
  }

  private final Map<Integer, List<Service>> servicesCache = new java.util.concurrent.ConcurrentHashMap<>();

  private List<Service> services(int routeIndex) {
    return servicesCache.computeIfAbsent(routeIndex, this::createServices);
  }

  @SuppressWarnings("deprecation")
  private List<Service> createServices(int routeIndex) {
    var route = transitData.getRouteForIndex(routeIndex);
    if (!(route instanceof TripPatternForDates tpd)) {
      return List.of();
    }
    Map<FrequencyEntry, ServiceWindows> byEntry = new IdentityHashMap<>();
    // Group identical frequency entries across days; different entries get their own service.
    Map<TripTimes, FrequencyEntry> canonical = new IdentityHashMap<>();
    var it = tpd.tripPatternForDatesIndexIterator(true);
    while (it.hasNext()) {
      int i = it.next();
      var forDate = tpd.tripPatternForDate(i);
      int offset = tpd.tripPatternForDateOffsets(i);
      for (var f : forDate.getFrequencies()) {
        if (f.routingSlack() <= 0) {
          continue;
        }
        var key = canonical.computeIfAbsent(f.tripTimes(), k -> f);
        byEntry
          .computeIfAbsent(key, k -> new ServiceWindows())
          .add(f.startTime() + offset, f.endTime() + offset, forDate.getServiceDate(), offset);
      }
    }
    var list = new ArrayList<Service>();
    for (var e : byEntry.entrySet()) {
      list.add(new Service(e.getKey(), e.getValue().sorted()));
    }
    return list;
  }
}
