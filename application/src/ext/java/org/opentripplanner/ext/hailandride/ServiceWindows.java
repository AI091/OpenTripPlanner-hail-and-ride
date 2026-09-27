package org.opentripplanner.ext.hailandride;

import java.time.LocalDate;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

/**
 * The trip-start windows of one frequency entry over the days covered by a request, in seconds
 * since the request's transit-search-time-zero, sorted by start. E.g. a 07:00-22:00 headway
 * service on yesterday, today and tomorrow gives three windows.
 */
final class ServiceWindows {

  private record Window(int start, int end, LocalDate serviceDate, int dayOffset) {}

  private final List<Window> windows = new ArrayList<>();

  void add(int start, int end, LocalDate serviceDate, int dayOffset) {
    windows.add(new Window(start, end, serviceDate, dayOffset));
  }

  ServiceWindows sorted() {
    windows.sort(Comparator.comparingInt(Window::start));
    return this;
  }

  int size() {
    return windows.size();
  }

  int start(int i) {
    return windows.get(i).start;
  }

  int end(int i) {
    return windows.get(i).end;
  }

  LocalDate serviceDate(int i) {
    return windows.get(i).serviceDate;
  }

  int dayOffset(int i) {
    return windows.get(i).dayOffset;
  }
}
