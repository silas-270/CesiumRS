package com.example.focusflight.flight

import kotlin.math.*

object FlightTelemetryGenerator {

    private const val EARTH_RADIUS_M = 6_371_000.0

    // -- Data structures for 2D geometry -------------------------------------
    private data class Point2D(val x: Double, val y: Double)
    
    private interface Segment2D {
        val length: Double
        fun getPoint(s: Double): Point2D
    }

    private class LineSegment(val p1: Point2D, val p2: Point2D) : Segment2D {
        override val length = hypot(p2.x - p1.x, p2.y - p1.y)
        override fun getPoint(s: Double): Point2D {
            val frac = if (length > 0) (s / length).coerceIn(0.0, 1.0) else 0.0
            return Point2D(p1.x + (p2.x - p1.x) * frac, p1.y + (p2.y - p1.y) * frac)
        }
    }

    private class ArcSegment(
        val center: Point2D, 
        val radius: Double, 
        val startAngle: Double, 
        val sweepAngle: Double
    ) : Segment2D {
        override val length = radius * abs(sweepAngle)
        override fun getPoint(s: Double): Point2D {
            val frac = if (length > 0) (s / length).coerceIn(0.0, 1.0) else 0.0
            val angle = startAngle + sweepAngle * frac
            return Point2D(center.x + radius * cos(angle), center.y + radius * sin(angle))
        }
    }

    private class Path2D {
        val segments = mutableListOf<Segment2D>()
        val totalLength: Double get() = segments.sumOf { it.length }

        private object DubinsSolver {
            enum class TurnDir { L, R }

            data class DubinsPath(
                val type: String,
                val length: Double,
                val c1: Point2D, val dir1: TurnDir, val t1: Point2D,
                val t2: Point2D,
                val c2: Point2D, val dir2: TurnDir
            )

            fun solve(p1: Point2D, h1: Double, p2: Point2D, h2: Double, R: Double): DubinsPath {
                val paths = mutableListOf<DubinsPath>()
                
                // Right/Left vectors (Right is CW 90, Left is CCW 90)
                val r1 = Point2D(cos(h1), -sin(h1))
                val l1 = Point2D(-cos(h1), sin(h1))
                val r2 = Point2D(cos(h2), -sin(h2))
                val l2 = Point2D(-cos(h2), sin(h2))

                // Centers
                val cR1 = Point2D(p1.x + R * r1.x, p1.y + R * r1.y)
                val cL1 = Point2D(p1.x + R * l1.x, p1.y + R * l1.y)
                val cR2 = Point2D(p2.x + R * r2.x, p2.y + R * r2.y)
                val cL2 = Point2D(p2.x + R * l2.x, p2.y + R * l2.y)

                fun evaluate(type: String, c1: Point2D, dir1: TurnDir, c2: Point2D, dir2: TurnDir, t1: Point2D, t2: Point2D) {
                    // Arc 1
                    val a1Start = atan2(p1.y - c1.y, p1.x - c1.x)
                    val a1End = atan2(t1.y - c1.y, t1.x - c1.x)
                    var sweep1 = a1End - a1Start
                    while (sweep1 < -PI) sweep1 += 2 * PI
                    while (sweep1 > PI) sweep1 -= 2 * PI
                    if (dir1 == TurnDir.L && sweep1 < 0) sweep1 += 2 * PI
                    if (dir1 == TurnDir.R && sweep1 > 0) sweep1 -= 2 * PI
                    
                    // Arc 2
                    val a2Start = atan2(t2.y - c2.y, t2.x - c2.x)
                    val a2End = atan2(p2.y - c2.y, p2.x - c2.x)
                    var sweep2 = a2End - a2Start
                    while (sweep2 < -PI) sweep2 += 2 * PI
                    while (sweep2 > PI) sweep2 -= 2 * PI
                    if (dir2 == TurnDir.L && sweep2 < 0) sweep2 += 2 * PI
                    if (dir2 == TurnDir.R && sweep2 > 0) sweep2 -= 2 * PI
                    
                    val len = R * abs(sweep1) + hypot(t2.x - t1.x, t2.y - t1.y) + R * abs(sweep2)
                    paths.add(DubinsPath(type, len, c1, dir1, t1, t2, c2, dir2))
                }

                // LSL
                var v = Point2D(cL2.x - cL1.x, cL2.y - cL1.y)
                var D = hypot(v.x, v.y)
                if (D > 1e-6) {
                    val gamma = atan2(v.x, v.y)
                    val nx = cos(gamma)
                    val ny = -sin(gamma)
                    val t1 = Point2D(cL1.x + R * nx, cL1.y + R * ny)
                    val t2 = Point2D(cL2.x + R * nx, cL2.y + R * ny)
                    evaluate("LSL", cL1, TurnDir.L, cL2, TurnDir.L, t1, t2)
                }

                // RSR
                v = Point2D(cR2.x - cR1.x, cR2.y - cR1.y)
                D = hypot(v.x, v.y)
                if (D > 1e-6) {
                    val gamma = atan2(v.x, v.y)
                    val nx = -cos(gamma)
                    val ny = sin(gamma)
                    val t1 = Point2D(cR1.x + R * nx, cR1.y + R * ny)
                    val t2 = Point2D(cR2.x + R * nx, cR2.y + R * ny)
                    evaluate("RSR", cR1, TurnDir.R, cR2, TurnDir.R, t1, t2)
                }

                // RSL
                v = Point2D(cL2.x - cR1.x, cL2.y - cR1.y)
                D = hypot(v.x, v.y)
                if (D >= 2 * R) {
                    val gamma = atan2(v.x, v.y)
                    val beta = asin(2 * R / D)
                    val pathHeading = gamma + beta
                    val t1x = cR1.x + R * (-cos(pathHeading))
                    val t1y = cR1.y + R * (sin(pathHeading))
                    val t2x = cL2.x + R * (cos(pathHeading))
                    val t2y = cL2.y + R * (-sin(pathHeading))
                    evaluate("RSL", cR1, TurnDir.R, cL2, TurnDir.L, Point2D(t1x, t1y), Point2D(t2x, t2y))
                }

                // LSR
                v = Point2D(cR2.x - cL1.x, cR2.y - cL1.y)
                D = hypot(v.x, v.y)
                if (D >= 2 * R) {
                    val gamma = atan2(v.x, v.y)
                    val beta = asin(2 * R / D)
                    val pathHeading = gamma - beta
                    val t1x = cL1.x + R * (cos(pathHeading))
                    val t1y = cL1.y + R * (-sin(pathHeading))
                    val t2x = cR2.x + R * (-cos(pathHeading))
                    val t2y = cR2.y + R * (sin(pathHeading))
                    evaluate("LSR", cL1, TurnDir.L, cR2, TurnDir.R, Point2D(t1x, t1y), Point2D(t2x, t2y))
                }

                return paths.minByOrNull { it.length } ?: error("No valid Dubins path found")
            }
        }

        fun addDubinsPath(p1: Point2D, h1: Double, p2: Point2D, h2: Double, radius: Double) {
            val path = DubinsSolver.solve(p1, h1, p2, h2, radius)
            
            // Arc 1
            val a1Start = atan2(p1.y - path.c1.y, p1.x - path.c1.x)
            val a1End = atan2(path.t1.y - path.c1.y, path.t1.x - path.c1.x)
            var sweep1 = a1End - a1Start
            while (sweep1 < -PI) sweep1 += 2 * PI
            while (sweep1 > PI) sweep1 -= 2 * PI
            if (path.dir1 == DubinsSolver.TurnDir.L && sweep1 < 0) sweep1 += 2 * PI
            if (path.dir1 == DubinsSolver.TurnDir.R && sweep1 > 0) sweep1 -= 2 * PI
            
            if (abs(sweep1) > 1e-6) {
                segments.add(ArcSegment(path.c1, radius, a1Start, sweep1))
            }
            
            // Straight
            segments.add(LineSegment(path.t1, path.t2))
            
            // Arc 2
            val a2Start = atan2(path.t2.y - path.c2.y, path.t2.x - path.c2.x)
            val a2End = atan2(p2.y - path.c2.y, p2.x - path.c2.x)
            var sweep2 = a2End - a2Start
            while (sweep2 < -PI) sweep2 += 2 * PI
            while (sweep2 > PI) sweep2 -= 2 * PI
            if (path.dir2 == DubinsSolver.TurnDir.L && sweep2 < 0) sweep2 += 2 * PI
            if (path.dir2 == DubinsSolver.TurnDir.R && sweep2 > 0) sweep2 -= 2 * PI
            
            if (abs(sweep2) > 1e-6) {
                segments.add(ArcSegment(path.c2, radius, a2Start, sweep2))
            }
        }

        fun getPoint(s: Double): Point2D {
            var remaining = s.coerceIn(0.0, totalLength)
            for (seg in segments) {
                if (remaining <= seg.length + 1e-6) {
                    return seg.getPoint(remaining)
                }
                remaining -= seg.length
            }
            return segments.last().getPoint(segments.last().length)
        }
    }


    // -- Generator -----------------------------------------------------------

    fun generate(
        departureLon: Double, departureLat: Double,
        arrivalLon: Double, arrivalLat: Double,
        totalDurationMs: Long,
        depHeadingDeg: Double? = null,
        arrHeadingDeg: Double? = null
    ): List<TelemetryPoint> {
        
        // Equirectangular projection center
        val latMid = Math.toRadians((departureLat + arrivalLat) / 2.0)
        val mPerDegLat = 111320.0
        val mPerDegLon = 111320.0 * cos(latMid)

        fun to2D(lon: Double, lat: Double) = Point2D((lon - departureLon) * mPerDegLon, (lat - departureLat) * mPerDegLat)
        fun toGeo(p: Point2D) = Pair(departureLon + p.x / mPerDegLon, departureLat + p.y / mPerDegLat)

        val pDep = to2D(departureLon, departureLat)
        val pArr = to2D(arrivalLon, arrivalLat)

        val directHeadingRad = atan2(pArr.x - pDep.x, pArr.y - pDep.y)
        val depHRad = depHeadingDeg?.let { Math.toRadians(it) } ?: directHeadingRad
        val arrHRad = arrHeadingDeg?.let { Math.toRadians(it) } ?: directHeadingRad

        val w0 = pDep
        val w1 = Point2D(w0.x + 10000.0 * sin(depHRad), w0.y + 10000.0 * cos(depHRad))
        val w2 = Point2D(pArr.x - 15000.0 * sin(arrHRad), pArr.y - 15000.0 * cos(arrHRad))
        val w3 = pArr

        val path = Path2D()
        val turnRadius = 4000.0 
        
        path.segments.add(LineSegment(w0, w1))
        path.addDubinsPath(w1, depHRad, w2, arrHRad, turnRadius)
        path.segments.add(LineSegment(w2, w3))

        val S = path.totalLength
        
        // Dynamic scaling of phases for short flights to prevent overlaps
        val idealGround = 3000.0
        val idealLanding = 3000.0
        val idealClimb = 30000.0
        val idealDescent = 50000.0

        val groundDist = idealGround.coerceAtMost(S * 0.1)
        val landingDist = idealLanding.coerceAtMost(S * 0.1)
        val airDist = S - groundDist - landingDist

        val climbDist: Double
        val descDist: Double
        val cruiseDist: Double

        if (airDist >= idealClimb + idealDescent) {
            climbDist = idealClimb
            descDist = idealDescent
            cruiseDist = airDist - climbDist - descDist
        } else {
            // Split the available air distance in a 3:5 ratio between climb and descent
            climbDist = airDist * (3.0 / 8.0)
            descDist = airDist * (5.0 / 8.0)
            cruiseDist = 0.0
        }

        val sGroundEnd = groundDist
        val sClimbEnd = sGroundEnd + climbDist
        val sDescStart = sClimbEnd + cruiseDist
        val sLandStart = sDescStart + descDist
        
        val cruiseAltM = computeCruiseAltitude(S)

        // Realistic Altitude Profile Layer
        fun getAltitude(s: Double): Double {
            return when {
                s <= sGroundEnd -> 0.0
                
                // Climb Phase: 3-part realistic climb
                s <= sClimbEnd -> {
                    val climbDistActual = sClimbEnd - sGroundEnd
                    val sigma = s - sGroundEnd
                    
                    // 10% rotation, 20% level off
                    val dRot = climbDistActual * 0.1
                    val dLvl = climbDistActual * 0.2
                    val dLin = climbDistActual - dRot - dLvl
                    
                    // Constant climb slope in the linear section
                    val maxSlope = cruiseAltM / (climbDistActual - 0.5 * (dRot + dLvl))
                    
                    when {
                        sigma <= dRot -> {
                            // Rotation: quadratic acceleration
                            0.5 * maxSlope * (sigma * sigma / dRot)
                        }
                        sigma <= dRot + dLin -> {
                            // Linear climb
                            val zRotEnd = 0.5 * maxSlope * dRot
                            zRotEnd + maxSlope * (sigma - dRot)
                        }
                        else -> {
                            // Level-off: inverse quadratic
                            val sLvl = sigma - (dRot + dLin)
                            val zLinEnd = 0.5 * maxSlope * dRot + maxSlope * dLin
                            zLinEnd + maxSlope * sLvl - 0.5 * maxSlope * (sLvl * sLvl / dLvl)
                        }
                    }
                }
                
                s <= sDescStart -> cruiseAltM
                
                // Descent Phase: 3-part realistic descent
                s <= sLandStart -> {
                    val descDistActual = sLandStart - sDescStart
                    val sigma = s - sDescStart
                    
                    val dTod = descDistActual * 0.2
                    val dFlare = descDistActual * 0.1
                    val dLin = descDistActual - dTod - dFlare
                    
                    val maxSlope = cruiseAltM / (descDistActual - 0.5 * (dTod + dFlare))
                    
                    when {
                        sigma <= dTod -> {
                            // Top of Descent: pitch down quadratically
                            cruiseAltM - 0.5 * maxSlope * (sigma * sigma / dTod)
                        }
                        sigma <= dTod + dLin -> {
                            // Linear descent
                            val zTodEnd = cruiseAltM - 0.5 * maxSlope * dTod
                            zTodEnd - maxSlope * (sigma - dTod)
                        }
                        else -> {
                            // Flare: pitch up to level flight
                            val sFlare = sigma - (dTod + dLin)
                            val zLinEnd = cruiseAltM - 0.5 * maxSlope * dTod - maxSlope * dLin
                            zLinEnd - (maxSlope * sFlare - 0.5 * maxSlope * (sFlare * sFlare / dFlare))
                        }
                    }
                }
                
                else -> 0.0
            }
        }

        fun getSpeedShape(s: Double): Double {
            return when {
                s < sGroundEnd -> lerp(0.05, 0.3, s / sGroundEnd)
                s < sClimbEnd -> lerp(0.3, 1.0, (s - sGroundEnd) / (sClimbEnd - sGroundEnd))
                s < sDescStart -> 1.0
                s < sLandStart -> lerp(1.0, 0.2, (s - sDescStart) / (sLandStart - sDescStart))
                else -> lerp(0.2, 0.02, (s - sLandStart) / (S - sLandStart))
            }
        }

        // Integrate to find total unscaled time
        val integrationSteps = 1000
        val ds = S / integrationSteps
        var unscaledTime = 0.0
        for (i in 0 until integrationSteps) {
            val sMid = i * ds + ds / 2.0
            unscaledTime += ds / getSpeedShape(sMid)
        }

        val totalS = totalDurationMs / 1000.0

        // Generate points at fixed time intervals
        val points = mutableListOf<TelemetryPoint>()
        var currentS = 0.0
        var currentT = 0.0
        val dt = 2.0 // sample every 2 seconds
        
        while (currentT <= totalS) {
            val p2d = path.getPoint(currentS)
            val (lon, lat) = toGeo(p2d)
            val alt = getAltitude(currentS)
            
            points.add(TelemetryPoint(
                timeOffsetMs = (currentT * 1000).toLong(),
                longitude = lon,
                latitude = lat,
                altitude = alt
            ))

            val vShape = getSpeedShape(currentS)
            // Correct mathematically scaled speed: realV = vShape * (unscaledTime / totalS)
            val realV = vShape * (unscaledTime / totalS)
            currentS += realV * dt
            currentT += dt
        }
        
        val (finalLon, finalLat) = toGeo(path.getPoint(S))
        points.add(TelemetryPoint(
            timeOffsetMs = totalDurationMs,
            longitude = finalLon,
            latitude = finalLat,
            altitude = 0.0
        ))

        return points
    }

    private fun lerp(a: Double, b: Double, t: Double): Double = a + (b - a) * t

    private fun computeCruiseAltitude(distanceM: Double): Double {
        val km = distanceM / 1_000.0
        return when {
            km < 300  -> lerp(6_000.0, 7_000.0, (km / 300.0))
            km < 800  -> lerp(7_000.0, 10_000.0, ((km - 300.0) / 500.0))
            km < 3000 -> lerp(10_000.0, 11_500.0, ((km - 800.0) / 2_200.0))
            else      -> lerp(11_500.0, 13_000.0, ((km - 3_000.0) / 10_000.0).coerceAtMost(1.0))
        }
    }

    fun interpolateAt(
        telemetry: List<TelemetryPoint>,
        timeOffsetMs: Long
    ): TelemetryPoint {
        require(telemetry.isNotEmpty()) { "Telemetry list must not be empty." }

        if (timeOffsetMs <= telemetry.first().timeOffsetMs) return telemetry.first()
        if (timeOffsetMs >= telemetry.last().timeOffsetMs) return telemetry.last()

        var lo = 0
        var hi = telemetry.lastIndex
        while (lo < hi - 1) {
            val mid = (lo + hi) / 2
            if (telemetry[mid].timeOffsetMs <= timeOffsetMs) lo = mid else hi = mid
        }

        val a = telemetry[lo]
        val b = telemetry[hi]
        val segDuration = (b.timeOffsetMs - a.timeOffsetMs).toDouble()
        val frac = if (segDuration > 0) (timeOffsetMs - a.timeOffsetMs) / segDuration else 0.0

        return TelemetryPoint(
            timeOffsetMs = timeOffsetMs,
            longitude = a.longitude + (b.longitude - a.longitude) * frac,
            latitude = a.latitude + (b.latitude - a.latitude) * frac,
            altitude = a.altitude + (b.altitude - a.altitude) * frac
        )
    }
}
