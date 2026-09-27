package org.opentripplanner.ext.hailandride;

import org.opentripplanner.street.search.state.State;

/**
 * How the rider walks between the origin/destination and one hail-and-ride sample point.
 *
 * @param state       street search state at the street vertex next to the sample (the walk path
 *                    is the back-state chain of this state)
 * @param seconds     total walking seconds including the last bit along the street edge to the
 *                    sample
 * @param weight      generalized cost (street search weight units = seconds * reluctance)
 * @param extraMeters metres walked beyond {@code state}'s vertex to reach the sample
 * @param sample      the sample index in the pattern's {@link PatternSamples}
 */
public record WalkReach(State state, int seconds, double weight, double extraMeters, int sample) {}
