package org.opentripplanner.ext.hailandride;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.List;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.LineString;
import org.opentripplanner.core.model.basic.Cost;
import org.opentripplanner.core.model.i18n.NonLocalizedString;
import org.opentripplanner.model.plan.Itinerary;
import org.opentripplanner.model.plan.Leg;
import org.opentripplanner.model.plan.Place;
import org.opentripplanner.model.plan.leg.HailAndRideLegBuilder;
import org.opentripplanner.model.plan.leg.StreetLeg;
import org.opentripplanner.routing.algorithm.mapping.StreetPathToLegsMapper;
import org.opentripplanner.routing.api.request.RouteRequest;
import org.opentripplanner.street.geometry.GeometryUtils;
import org.opentripplanner.street.model.path.StreetPath;
import org.opentripplanner.street.search.TraverseMode;

/**
 * Turns hail-and-ride rides into itinerary legs:
 * <pre>
 *   access:  WALK (origin -> hail point)            + BUS (hail point -> stop k)
 *   egress:  BUS (stop k -> drop-off point)          + WALK (drop-off point -> destination)
 *   direct:  WALK + BUS (hail point -> drop-off point) + WALK
 * </pre>
 */
public final class HailAndRideLegMapper {

  private final StreetPathToLegsMapper streetMapper;
  private final RouteRequest request;
  private final ZonedDateTime transitSearchTimeZero;

  public HailAndRideLegMapper(
    StreetPathToLegsMapper streetMapper,
    RouteRequest request,
    ZonedDateTime transitSearchTimeZero
  ) {
    this.streetMapper = streetMapper;
    this.request = request;
    this.transitSearchTimeZero = transitSearchTimeZero;
  }

  /** Map an access or egress; {@code departureTime} is when the access/egress leg starts. */
  public List<Leg> mapAccessEgress(HailAndRideAccessEgress ae, int departureTime) {
    return mapRide(ae.ride(), departureTime, ae.c1() / 100);
  }

  public Itinerary mapDirect(HailAndRideRide ride, int departureTime, int c1Seconds) {
    var legs = mapRide(ride, departureTime, c1Seconds);
    return Itinerary.ofScheduledTransit(legs)
      .withGeneralizedCost(Cost.costOfSeconds(c1Seconds))
      .build();
  }

  private List<Leg> mapRide(HailAndRideRide ride, int departureTime, int c1Seconds) {
    var ps = ride.samples();
    var legs = new ArrayList<Leg>();

    int boardTime = ride.boardTime(departureTime);
    int tripStart = ride.tripStartTime(departureTime);
    int alightTime = tripStart + ride.alightOffset();

    Place boardPlace = ride.boardSample() >= 0 && ps.hopFraction(ride.boardSample()) > 0
      ? samplePlace(ps, ride.boardSample(), "Hail")
      : null;
    Place alightPlace = ride.alightSample() >= 0 && ps.hopFraction(ride.alightSample()) > 0
      ? samplePlace(ps, ride.alightSample(), "Get off")
      : null;

    if (ride.walkBefore() != null) {
      legs.addAll(walkBefore(ride.walkBefore(), departureTime, ps, ride.boardSample()));
    }
    legs.add(transitLeg(ride, tripStart, boardTime, alightTime, boardPlace, alightPlace, c1Seconds));
    if (ride.walkAfter() != null) {
      legs.addAll(walkAfter(ride.walkAfter(), alightTime, ps, ride.alightSample()));
    }
    return legs;
  }

  private Leg transitLeg(
    HailAndRideRide ride,
    int tripStart,
    int boardTime,
    int alightTime,
    Place boardPlace,
    Place alightPlace,
    int c1Seconds
  ) {
    var ps = ride.samples();
    int boardHop;
    double boardFraction;
    int boardStopPos;
    if (ride.boardSample() >= 0) {
      boardHop = ps.hop(ride.boardSample());
      boardFraction = ps.hopFraction(ride.boardSample());
      boardStopPos = boardHop;
    } else {
      boardHop = ride.boardStopPos();
      boardFraction = 0;
      boardStopPos = ride.boardStopPos();
    }
    int alightHop;
    double alightFraction;
    int alightStopPos;
    if (ride.alightSample() >= 0) {
      alightHop = ps.hop(ride.alightSample());
      alightFraction = ps.hopFraction(ride.alightSample());
      alightStopPos = alightFraction == 0 ? alightHop : alightHop + 1;
    } else {
      alightHop = ride.alightStopPos();
      alightFraction = 0;
      alightStopPos = ride.alightStopPos();
    }
    double fromDist = ride.boardSample() >= 0
      ? ps.distanceAlongPattern(ride.boardSample())
      : ps.stopDistanceAlongPattern(ride.boardStopPos());
    double toDist = ride.alightSample() >= 0
      ? ps.distanceAlongPattern(ride.alightSample())
      : ps.stopDistanceAlongPattern(ride.alightStopPos());

    int departure = departureOf(ride, boardTime);
    int dayOffset = ride.serviceDayOffset(departure);
    var tripTimes = ride.entry().materialize(0, tripStart - dayOffset, true);

    var builder = new HailAndRideLegBuilder()
      .withBoardPlace(boardPlace)
      .withAlightPlace(alightPlace)
      .withGeometry(
        HailAndRideGeometry.between(ps.pattern(), boardHop, boardFraction, alightHop, alightFraction)
      )
      .withDistanceMeters(Math.max(0, toDist - fromDist));
    return builder
      .withTripTimes(tripTimes)
      .withTripPattern(ps.pattern())
      .withBoardStopIndexInPattern(boardStopPos)
      .withAlightStopIndexInPattern(alightStopPos)
      .withStartTime(time(boardTime))
      .withEndTime(time(alightTime))
      .withServiceDate(ride.serviceDate(departureOf(ride, boardTime)))
      .withZoneId(transitSearchTimeZero.getZone().normalized())
      .withGeneralizedCost(c1Seconds)
      .withFrequencyHeadwayInSeconds(ride.headway())
      .build();
  }

  /** Inverse of {@link HailAndRideRide#boardTime(int)}. */
  private static int departureOf(HailAndRideRide ride, int boardTime) {
    return boardTime - ride.walkBeforeSeconds() - ride.headway();
  }

  private List<Leg> walkBefore(WalkReach w, int departureTime, PatternSamples ps, int sample) {
    var target = ps.coordinate(sample);
    var place = samplePlace(ps, sample, "Hail");
    var path = new StreetPath(w.state());
    List<Leg> legs = new ArrayList<>(streetMapper.map(path, request, time(departureTime)));
    int end = departureTime + w.seconds();
    if (legs.isEmpty() || !(legs.getLast() instanceof StreetLeg last)) {
      Place from = Place.normal(w.state().getVertex(), new NonLocalizedString("Origin"));
      legs.add(straightWalk(from, place, departureTime, end, w.extraMeters(), target, true));
      return legs;
    }
    var coords = new ArrayList<>(List.of(last.legGeometry().getCoordinates()));
    coords.add(target);
    legs.set(
      legs.size() - 1,
      last
        .copyOf()
        .withTo(place)
        .withEndTime(time(end))
        .withDistanceMeters(last.distanceMeters() + w.extraMeters())
        .withGeometry(GeometryUtils.makeLineString(coords))
        .build()
    );
    return legs;
  }

  private List<Leg> walkAfter(WalkReach w, int alightTime, PatternSamples ps, int sample) {
    var origin = ps.coordinate(sample);
    var place = samplePlace(ps, sample, "Get off");
    int extraSec = (int) Math.round(w.extraMeters() / request.preferences().walk().speed());
    var path = new StreetPath(w.state());
    List<Leg> legs = new ArrayList<>(streetMapper.map(path, request, time(alightTime + extraSec)));
    if (legs.isEmpty() || !(legs.getFirst() instanceof StreetLeg first)) {
      Place to = Place.normal(w.state().getVertex(), new NonLocalizedString("Destination"));
      legs.add(
        straightWalk(place, to, alightTime, alightTime + w.seconds(), w.extraMeters(), origin, false)
      );
      return legs;
    }
    var coords = new ArrayList<Coordinate>();
    coords.add(origin);
    coords.addAll(List.of(first.legGeometry().getCoordinates()));
    legs.set(
      0,
      first
        .copyOf()
        .withFrom(place)
        .withStartTime(time(alightTime))
        .withDistanceMeters(first.distanceMeters() + w.extraMeters())
        .withGeometry(GeometryUtils.makeLineString(coords))
        .build()
    );
    return legs;
  }

  private StreetLeg straightWalk(
    Place from,
    Place to,
    int start,
    int end,
    double meters,
    Coordinate samplePoint,
    boolean toSample
  ) {
    Coordinate a = new Coordinate(from.coordinate.longitude(), from.coordinate.latitude());
    Coordinate b = new Coordinate(to.coordinate.longitude(), to.coordinate.latitude());
    LineString geom = GeometryUtils.makeLineString(a, b);
    return StreetLeg.of()
      .withMode(TraverseMode.WALK)
      .withStartTime(time(start))
      .withEndTime(time(end))
      .withFrom(from)
      .withTo(to)
      .withDistanceMeters(meters)
      .withGeometry(geom)
      .withWalkSteps(List.of())
      .build();
  }

  private static Place samplePlace(PatternSamples ps, int sample, String verb) {
    var route = ps.pattern().getRoute();
    String routeName = route.getShortName() != null && !route.getShortName().isBlank()
      ? route.getShortName().trim()
      : route.getLongName() != null
        ? route.getLongName().toString()
        : route.getId().getId();
    String street = ps.edge(sample).getName().toString();
    return Place.normal(
      ps.lat(sample),
      ps.lon(sample),
      new NonLocalizedString(verb + " " + routeName + " on " + street)
    );
  }

  private ZonedDateTime time(int secondsSinceTimeZero) {
    return transitSearchTimeZero.plusSeconds(secondsSinceTimeZero);
  }
}
