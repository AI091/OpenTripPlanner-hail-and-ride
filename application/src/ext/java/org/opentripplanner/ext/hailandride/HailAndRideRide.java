package org.opentripplanner.ext.hailandride;

import java.time.LocalDate;
import javax.annotation.Nullable;
import org.opentripplanner.raptor.spi.RaptorConstants;
import org.opentripplanner.street.search.state.State;
import org.opentripplanner.transit.model.timetable.FrequencyEntry;

/**
 * One hail-and-ride ride on a headway-based pattern, plus the walk that leads to it (access,
 * direct) or away from it (egress, direct).
 * <pre>
 *   access:  [walk] -> board at sample -> ride -> alight at stop k
 *   egress:  board at stop k -> ride -> alight at sample -> [walk]
 *   direct:  [walk] -> board at sample -> ride -> alight at sample -> [walk]
 * </pre>
 * All times are seconds since the request's transit-search-time-zero, except the vehicle offsets
 * which are seconds after the trip's first departure.
 */
public final class HailAndRideRide {

  public enum Kind {
    ACCESS,
    EGRESS,
    DIRECT,
  }

  private final Kind kind;
  private final PatternSamples samples;
  private final FrequencyEntry entry;
  private final int headway;
  private final ServiceWindows windows;

  /** Sample index where the rider boards, or -1 if boarding at stop {@link #boardStopPos}. */
  private final int boardSample;
  private final int boardStopPos;
  /** Sample index where the rider alights, or -1 if alighting at stop {@link #alightStopPos}. */
  private final int alightSample;
  private final int alightStopPos;

  /** Vehicle offset at the boarding place (seconds after trip start). */
  private final int boardOffset;
  /** Vehicle offset at the alighting place (seconds after trip start). */
  private final int alightOffset;

  @Nullable
  private final WalkReach walkBefore;

  @Nullable
  private final WalkReach walkAfter;

  HailAndRideRide(
    Kind kind,
    PatternSamples samples,
    FrequencyEntry entry,
    ServiceWindows windows,
    int boardSample,
    int boardStopPos,
    int alightSample,
    int alightStopPos,
    int boardOffset,
    int alightOffset,
    @Nullable WalkReach walkBefore,
    @Nullable WalkReach walkAfter
  ) {
    this.kind = kind;
    this.samples = samples;
    this.entry = entry;
    this.headway = entry.routingSlack();
    this.windows = windows;
    this.boardSample = boardSample;
    this.boardStopPos = boardStopPos;
    this.alightSample = alightSample;
    this.alightStopPos = alightStopPos;
    this.boardOffset = boardOffset;
    this.alightOffset = alightOffset;
    this.walkBefore = walkBefore;
    this.walkAfter = walkAfter;
  }

  public Kind kind() {
    return kind;
  }

  public PatternSamples samples() {
    return samples;
  }

  public FrequencyEntry entry() {
    return entry;
  }

  public int headway() {
    return headway;
  }

  public int boardSample() {
    return boardSample;
  }

  public int boardStopPos() {
    return boardStopPos;
  }

  public int alightSample() {
    return alightSample;
  }

  public int alightStopPos() {
    return alightStopPos;
  }

  public int boardOffset() {
    return boardOffset;
  }

  public int alightOffset() {
    return alightOffset;
  }

  /** In-vehicle seconds, from the boarding place to the alighting place. */
  public int rideSeconds() {
    return alightOffset - boardOffset;
  }

  @Nullable
  public WalkReach walkBefore() {
    return walkBefore;
  }

  @Nullable
  public WalkReach walkAfter() {
    return walkAfter;
  }

  public int walkBeforeSeconds() {
    return walkBefore == null ? 0 : walkBefore.seconds();
  }

  public int walkAfterSeconds() {
    return walkAfter == null ? 0 : walkAfter.seconds();
  }

  /** Total duration: walk + full-headway wait + ride + walk. */
  public int durationSeconds() {
    return walkBeforeSeconds() + headway + rideSeconds() + walkAfterSeconds();
  }

  /** The street state used as "final state" of the access/egress (walk part). */
  State walkState() {
    if (walkBefore != null) {
      return walkBefore.state();
    }
    return walkAfter == null ? null : walkAfter.state();
  }

  /**
   * Earliest time the rider can leave (the origin for access/direct, the stop for egress) so that
   * the vehicle passing the boarding place one headway after the rider arrives runs inside a
   * service window.
   */
  int earliestDepartureTime(int t) {
    int shift = boardOffset - walkBeforeSeconds() - headway;
    for (int w = 0; w < windows.size(); w++) {
      int lo = windows.start(w) + shift;
      int hi = windows.end(w) + shift;
      if (t < hi) {
        return Math.max(t, lo);
      }
    }
    return RaptorConstants.TIME_NOT_SET;
  }

  int latestArrivalTime(int t, int duration) {
    int shift = boardOffset - walkBeforeSeconds() - headway;
    for (int w = windows.size() - 1; w >= 0; w--) {
      int lo = windows.start(w) + shift;
      int hi = windows.end(w) + shift;
      int latestDep = Math.min(t - duration, hi - 1);
      if (latestDep >= lo) {
        return latestDep + duration;
      }
    }
    return RaptorConstants.TIME_NOT_SET;
  }

  /** Time (since search time zero) the vehicle passes the boarding place for a given departure. */
  public int boardTime(int departureTime) {
    return departureTime + walkBeforeSeconds() + headway;
  }

  /** Trip start time (since search time zero) for a given departure. */
  public int tripStartTime(int departureTime) {
    return boardTime(departureTime) - boardOffset;
  }

  /** The window index that contains the trip for a given departure (defaults to the first). */
  int windowFor(int departureTime) {
    int start = tripStartTime(departureTime);
    for (int w = 0; w < windows.size(); w++) {
      if (start >= windows.start(w) && start <= windows.end(w)) {
        return w;
      }
    }
    return 0;
  }

  public LocalDate serviceDate(int departureTime) {
    return windows.serviceDate(windowFor(departureTime));
  }

  /** Offset (seconds) between search time zero and midnight of the service date. */
  public int serviceDayOffset(int departureTime) {
    return windows.dayOffset(windowFor(departureTime));
  }

  @Override
  public String toString() {
    return (
      "HailAndRide{" +
      kind +
      " " +
      samples.pattern().getRoute().getShortName() +
      " board=" +
      (boardSample >= 0 ? "s" + boardSample : "stop" + boardStopPos) +
      " alight=" +
      (alightSample >= 0 ? "s" + alightSample : "stop" + alightStopPos) +
      " ride=" +
      rideSeconds() +
      "s headway=" +
      headway +
      "s}"
    );
  }
}
