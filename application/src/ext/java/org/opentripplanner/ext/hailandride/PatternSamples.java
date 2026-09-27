package org.opentripplanner.ext.hailandride;

import java.util.ArrayList;
import java.util.List;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Envelope;
import org.opentripplanner.street.model.edge.StreetEdge;
import org.opentripplanner.transit.model.network.TripPattern;
import org.opentripplanner.transit.model.timetable.TripTimes;

/**
 * The hail-and-ride sample points of one trip pattern, ordered along the pattern.
 * <p>
 * Position along the pattern is expressed as (hop, fraction): hop {@code i} is the segment
 * between stop position {@code i} and {@code i+1}; fraction is 0 at stop {@code i} and approaches
 * 1 at stop {@code i+1}. A sample with fraction 0 sits exactly at stop {@code i}.
 */
public final class PatternSamples {

  private final TripPattern pattern;
  private final int routeIndex;
  private final int[] hop;
  private final float[] hopFraction;
  private final double[] distanceAlongPattern;
  private final double[] lat;
  private final double[] lon;
  private final StreetEdge[] edge;
  private final float[] edgeFraction;
  private final float[] linkDistance;
  private final boolean[] canBoard;
  private final boolean[] canAlight;
  /** firstSampleOfHop[i] = index of the first sample with hop >= i; size = numberOfStops. */
  private final int[] firstSampleOfHop;
  private final double[] stopDistanceAlongPattern;
  private final Envelope envelope;

  private PatternSamples(Builder b) {
    this.pattern = b.pattern;
    this.routeIndex = b.pattern.getRoutingTripPattern().patternIndex();
    int n = b.hop.size();
    this.hop = new int[n];
    this.hopFraction = new float[n];
    this.distanceAlongPattern = new double[n];
    this.lat = new double[n];
    this.lon = new double[n];
    this.edge = new StreetEdge[n];
    this.edgeFraction = new float[n];
    this.linkDistance = new float[n];
    this.canBoard = new boolean[n];
    this.canAlight = new boolean[n];
    this.envelope = new Envelope();
    for (int i = 0; i < n; i++) {
      hop[i] = b.hop.get(i);
      hopFraction[i] = b.hopFraction.get(i);
      distanceAlongPattern[i] = b.distance.get(i);
      lat[i] = b.coord.get(i).y;
      lon[i] = b.coord.get(i).x;
      edge[i] = b.edge.get(i);
      edgeFraction[i] = b.edgeFraction.get(i);
      linkDistance[i] = b.linkDistance.get(i);
      canBoard[i] = b.canBoard.get(i);
      canAlight[i] = b.canAlight.get(i);
      envelope.expandToInclude(lon[i], lat[i]);
    }
    int nStops = b.stopDistance.length;
    this.stopDistanceAlongPattern = b.stopDistance;
    this.firstSampleOfHop = new int[nStops];
    int j = 0;
    for (int s = 0; s < nStops; s++) {
      while (j < n && hop[j] < s) {
        j++;
      }
      firstSampleOfHop[s] = j;
    }
  }

  public TripPattern pattern() {
    return pattern;
  }

  public int routeIndex() {
    return routeIndex;
  }

  public int size() {
    return hop.length;
  }

  public int hop(int i) {
    return hop[i];
  }

  public float hopFraction(int i) {
    return hopFraction[i];
  }

  public double lat(int i) {
    return lat[i];
  }

  public double lon(int i) {
    return lon[i];
  }

  public StreetEdge edge(int i) {
    return edge[i];
  }

  public float edgeFraction(int i) {
    return edgeFraction[i];
  }

  public float linkDistance(int i) {
    return linkDistance[i];
  }

  /** GTFS continuous_pickup allows waving the vehicle down here. */
  public boolean canBoard(int i) {
    return canBoard[i];
  }

  /** GTFS continuous_drop_off allows getting off here. */
  public boolean canAlight(int i) {
    return canAlight[i];
  }

  public double distanceAlongPattern(int i) {
    return distanceAlongPattern[i];
  }

  public double stopDistanceAlongPattern(int stopPos) {
    return stopDistanceAlongPattern[stopPos];
  }

  /** Index of the first sample on hop {@code stopPos} or later, i.e. at or after the stop. */
  public int firstSampleAtOrAfterStop(int stopPos) {
    return firstSampleOfHop[stopPos];
  }

  public Envelope envelope() {
    return envelope;
  }

  /**
   * Seconds after the trip's first departure at which the vehicle passes sample {@code i}.
   * Linear interpolation by distance between the departure from stop {@code hop} and the arrival
   * at stop {@code hop+1} - the same rule OTP uses elsewhere.
   */
  public int vehicleOffset(TripTimes tt, int i) {
    int h = hop[i];
    int dep = tt.getDepartureTime(h);
    int arr = tt.getArrivalTime(h + 1);
    return dep + Math.round(hopFraction[i] * (arr - dep)) - tt.getDepartureTime(0);
  }

  /** Seconds after the trip's first departure at which the vehicle arrives at a stop. */
  public static int stopArrivalOffset(TripTimes tt, int stopPos) {
    return tt.getArrivalTime(stopPos) - tt.getDepartureTime(0);
  }

  /** Seconds after the trip's first departure at which the vehicle departs a stop. */
  public static int stopDepartureOffset(TripTimes tt, int stopPos) {
    return tt.getDepartureTime(stopPos) - tt.getDepartureTime(0);
  }

  public Coordinate coordinate(int i) {
    return new Coordinate(lon[i], lat[i]);
  }

  static final class Builder {

    private final TripPattern pattern;
    private final List<Integer> hop = new ArrayList<>();
    private final List<Float> hopFraction = new ArrayList<>();
    private final List<Double> distance = new ArrayList<>();
    private final List<Coordinate> coord = new ArrayList<>();
    private final List<StreetEdge> edge = new ArrayList<>();
    private final List<Float> edgeFraction = new ArrayList<>();
    private final List<Float> linkDistance = new ArrayList<>();
    private final List<Boolean> canBoard = new ArrayList<>();
    private final List<Boolean> canAlight = new ArrayList<>();
    private final double[] stopDistance;

    Builder(TripPattern pattern, int nStops) {
      this.pattern = pattern;
      this.stopDistance = new double[nStops];
    }

    void markHopStart(int stopPos, double distanceAlong) {
      stopDistance[stopPos] = distanceAlong;
    }

    void add(
      int hop,
      float fraction,
      double distanceAlong,
      Coordinate c,
      StreetEdge e,
      float edgeFraction,
      float linkDistance,
      boolean canBoard,
      boolean canAlight
    ) {
      this.canBoard.add(canBoard);
      this.canAlight.add(canAlight);
      this.hop.add(hop);
      this.hopFraction.add(fraction);
      this.distance.add(distanceAlong);
      this.coord.add(c);
      this.edge.add(e);
      this.edgeFraction.add(edgeFraction);
      this.linkDistance.add(linkDistance);
    }

    int size() {
      return hop.size();
    }

    PatternSamples build() {
      return new PatternSamples(this);
    }
  }
}
