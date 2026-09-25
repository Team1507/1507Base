//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.lib.core.auto;

/**
 * Tuning for the route runner. Built from the robot's Constants, so the
 * library holds no season values.
 *
 * @param headingKp         turning gain: rad/s per radian of heading error
 * @param arriveKp          endpoint slow-down gain: speed = arriveKp × distance (m/s per meter)
 * @param endpointHandoff   within this distance of an endpoint (m), every driver hands over to
 *                          the classic slow-down, so endpoints are always precise
 * @param stallDistance     the robot must move at least this far (m) ...
 * @param stallSeconds      ... every this many seconds, or the route gives up (robot stuck)
 * @param maxSecondsPerNode give up if one node takes longer than this (s); catches circling
 * @param headingWaitSeconds how long the robot may sit on a node waiting for its heading (s)
 *                          before the heading is waived
 * @param holdTimeoutSeconds how long the robot may sit on a node waiting for its
 *                          {@code .holdUntil} condition (s) before moving on anyway
 */
public record RouteConfig(
    double headingKp,
    double arriveKp,
    double endpointHandoff,
    double stallDistance,
    double stallSeconds,
    double maxSecondsPerNode,
    double headingWaitSeconds,
    double holdTimeoutSeconds
) {}
