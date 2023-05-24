//===----------------------------------------------------------------------===//
//
//  Copyright (c) 2023 Svyatoslav Popov (info@keyvar.com).
//
//  Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with
//  the License. You may obtain a copy of the License at
//
//  http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on
//  an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the
//  specific language governing permissions and limitations under the License.
//
//  SPDX-License-Identifier: Apache-2.0
//
//===----------------------------------------------------------------------===//
//
//  KvSegment2.swift
//  kvGeometry
//
//  Created by Svyatoslav Popov on 21.05.2023.
//

import kvKit



/// Implementation of a segment in 2D coordinate space.
///
/// The segment coordinates are represented with *endPoints.0* + *direction* · *step* equation where *step* ∈ [0, *length*]. See ``at(_:)`` method.
public struct KvSegment2<Vertex : KvVertex2Protocol> {

    public typealias Math = Vertex.Math
    public typealias Vertex = Vertex

    public typealias Scalar = Math.Scalar
    public typealias Vector = Math.Vector2

    public typealias Transform = KvTransform2<Math>
    public typealias AffineTransform = KvAffineTransform2<Math>



    /// End points of the segment.
    public let endPoints: (Vertex, Vertex)

    /// Direction unit vector from *endPoints.0* to *endPoints.1*.
    public let front: Vector
    /// Length of the receiver.
    public let length: Scalar
    /// Reciprocal of the receiver's length.
    public let length⁻¹: Scalar



    /// Memberwise inilializer.
    @usableFromInline
    internal init(endPoints: (Vertex, Vertex), direction: Vector, length: Scalar, length⁻¹: Scalar) {
        self.endPoints = endPoints
        self.front = direction
        self.length = length
        self.length⁻¹ = length⁻¹
    }


    /// Initalizes a segment with given end ponts.
    ///
    /// - Note: The caller is responsible for clonning provided end points.
    @inlinable
    public init(_ endPoints: (Vertex, Vertex)) {
        switch Math.isInequal(endPoints.0.coordinate, endPoints.1.coordinate) {
        case true:
            let vector = endPoints.1.coordinate - endPoints.0.coordinate
            let length = Math.length(vector)
            let length⁻¹ = 1 / length

            self.init(endPoints: endPoints, direction: vector * length⁻¹, length: length, length⁻¹: length⁻¹)

        case false:
            self.init(endPoints: endPoints, direction: .unitX, length: 0, length⁻¹: 0)
        }
    }

    /// Initalizes a segment with given end ponts.
    ///
    /// - Note: The caller is responsible for clonning provided end points.
    @inlinable public init(_ origin: Vertex, _ target: Vertex) { self.init((origin, target)) }



    // MARK: Operations

    @inlinable
    public var isDegenerate: Bool { KvIsZero(length) }


    /// - Returns: Linear combination of the receiver's endPoints where `at(0) == endPoints.0` and `at(length) == endPoints.1`.
    ///
    /// See ``coordinate(at:)``.
    @inlinable public func at(_ step: Scalar) -> Vertex { endPoints.0.mixed(endPoints.1, t: step * length⁻¹) }


    /// - Returns: The same coordinate as `at(step).coordinate`.
    ///
    /// See ``at(_:)``.
    @inlinable public func coordinate(at step: Scalar) -> Vertex.Coordinate { endPoints.0.coordinate + step * front }


    /// - Returns: Copy of the receiver where vertices are cloned.
    @inlinable public func clone() -> Self { Self(endPoints: (endPoints.0.clone(), endPoints.1.clone()), direction: front, length: length, length⁻¹: length⁻¹) }


    /// - Returns: A ray having the same direction but flipped origin.
    @inlinable public func flipped() -> Self { Self(endPoints: (endPoints.0.flipped(), endPoints.1.flipped()), direction: front, length: length, length⁻¹: length⁻¹) }


    /// - Returns: Step *t* where `at(t)` is a coordinate the receiver intersects given line.
    ///
    /// - Note: It's equal to distance from *endPoints.0* to the intersection coordinate.
    ///
    /// See ``at(_:)``, ``intersection(with:)``, ``intersects(with:)``.
    @inlinable
    public func step(to line: KvLine2<Math>) -> Scalar? {
        let divider = Math.dot(line.normal, front)

        guard KvIsNonzero(divider) else { return nil }

        let t = -line.at(endPoints.0.coordinate) / divider

        guard contains(step: t) else { return nil }

        return t
    }


    /// - Returns: Distance to a coordinate.
    @inlinable
    public func distance(to coordinate: Vertex.Coordinate) -> Scalar {
        let arg = (vector: coordinate - endPoints.0.coordinate, epsArg: Math.epsArg(coordinate) - Math.epsArg(endPoints.0.coordinate))
        let cross = Math.cross(front, arg.vector)

        let isOnLine = Math.isZero(cross, eps: Math.epsArg(front).cross(arg.epsArg).tolerance)
        /// Step to the projection of the coordinate.
        let step = Math.dot(front, arg.vector)

        guard KvIsNotNegative(step)
        else { return isOnLine ? -step : Math.length(arg.vector) }

        guard KvIs(step, lessThanOrEqualTo: length)
        else { return isOnLine ? step - length : Math.length(coordinate - endPoints.1.coordinate) }

        return isOnLine ? 0.0 as Scalar : Math.length(cross)
    }

    /// - Returns: Distance to a vertex.
    @inlinable public func distance(to v: Vertex) -> Scalar { distance(to: v.coordinate) }

    /// - Returns: Distance to a vertex.
    @inlinable
    public func distance<V>(to vertex: V) -> Scalar
    where V : KvVertex2Protocol, V.Math == Math {
        distance(to: vertex.coordinate)
    }

    /// - Returns: Distance to a segment.
    @inlinable
    public func distance(to segment: Self) -> Scalar {
        let (lFront, lOrigin) = (front, endPoints.0.coordinate)
        let (rFront, rOrigin) = (segment.front, segment.endPoints.0.coordinate)

        let nz = Math.cross(lFront, rFront).z
        let denominator = sqr(nz)

        switch KvIsNonzero(denominator) {
        case true:
            let cross: Vector = nz * { v -> Vector in Vector(-v.y, v.x) }(rOrigin - lOrigin)    // Math.cross(n, rOrigin - lOrigin)
            let denominator⁻¹ = 1.0 as Scalar / denominator

            let lt = Math.dot(rFront, cross) * denominator⁻¹
            // Closest coordinate to the intersection on the receiver.
            let lc = coordinate(at: clamp(lt, 0.0 as Scalar, length))

            let rt = Math.dot(lFront, cross) * denominator⁻¹
            // Closest coordinate to the intersection on `segment`.
            let rc = segment.coordinate(at: clamp(rt, 0.0 as Scalar, segment.length))

            return min(segment.distance(to: lc),
                       segment.distance(to: endPoints.0.coordinate),
                       segment.distance(to: endPoints.1.coordinate),
                       distance(to: rc),
                       distance(to: segment.endPoints.0.coordinate),
                       distance(to: segment.endPoints.1.coordinate))

        case false:
            let v00 = segment.endPoints.0.coordinate - endPoints.0.coordinate
            let v01 = segment.endPoints.1.coordinate - endPoints.0.coordinate

            // Projections of segment.endPoints.0 and segment.endPoints.1 on the receiver.
            let (p0, p1): (Scalar, Scalar) =
            { KvIs($0, lessThanOrEqualTo: $1) ? ($0, $1) : ($1, $0) }(Math.dot(v00, front), Math.dot(v01, front))

            guard KvIsPositive(p1) else { return Math.distance(endPoints.0.coordinate, segment.endPoints.1.coordinate) }
            guard KvIs(p0, lessThan: length) else { return Math.distance(endPoints.1.coordinate, segment.endPoints.0.coordinate) }

            // It's true due to direction is a unit vector.
            return -Math.cross(v00, front).z
        }
    }


    /// - Returns: A boolean value indicating whether the receiver contains `at(step)` coordinate.
    ///
    /// See ``at(_:).
    @inlinable public func contains(step: Scalar) -> Bool { KvIsNotNegative(step) && KvIs(step, lessThanOrEqualTo: length) }


    /// - Returns: A boolean value indicating whether the receiver contains given coordinate.
    @inlinable
    public func contains(_ coordinate: Vertex.Coordinate) -> Bool {
        let arg = (vector: coordinate - endPoints.0.coordinate, epsArg: Math.epsArg(coordinate) - Math.epsArg(endPoints.0.coordinate))

        guard KvIsZero(Math.cross(front, arg.vector).z, eps: Math.epsArg(front).cross(arg.epsArg).tolerance) else { return false }

        return contains(step: Math.dot(front, arg.vector))
    }

    /// - Returns: A boolean value indicating whether the receiver contains given vertex.
    @inlinable public func contains(_ vertex: Vertex) -> Bool { contains(vertex.coordinate) }


    /// - Returns: A boolean value indicating whether the receiver intersects given line.
    ///
    /// See ``step(to:)``, ``intersection(with:)``.
    @inlinable public func intersects(with line: KvLine2<Math>) -> Bool { step(to: line) != nil }


    /// - Returns: A copy of the origin translated to coordinate where the receiver and given line intersect.
    ///
    /// See ``step(to:)``, ``intersects(with:)``.
    @inlinable
    public func intersection(with line: KvLine2<Math>) -> Vertex? {
        step(to: line).map(self.at(_:))
    }


    /// - Returns: A segment produced applying translation by *offset* to all the receiver's ponts.
    ///
    /// - Note: It's faster then apply an arbitrary transformation.
    @inlinable
    public func translated(by offset: Vector) -> Self {
        Self(endPoints: (endPoints.0 + offset, endPoints.1 + offset), direction: front, length: length, length⁻¹: length⁻¹)
    }


    /// - Returns: A ray produced applying scale to all the receiver's ponts.
    ///
    /// - Note: It's faster then apply arbitraty transformation.
    @inlinable
    public func scaled(by scale: Scalar) -> Self {
        let t = AffineTransform(scale: scale)
        let endPoints = (t * endPoints.0, t * endPoints.1)

        let scale = abs(scale)
        /// Assuming scale >= 0
        switch KvIsPositive(scale) {
        case true:
            return Self(endPoints: endPoints, direction: front, length: length * scale, length⁻¹: length⁻¹ / scale)
        case false:
            return Self(endPoints: endPoints, direction: .unitX, length: 0, length⁻¹: 0)
        }
    }


    /// - Returns: Result of given transformation applied to *rhs*.
    @inlinable public static func *(lhs: Transform, rhs: Self) -> Self { Self(lhs * rhs.endPoints.0, lhs * rhs.endPoints.1) }

    /// - Returns: Result of given transformation applied to *rhs*.
    @inlinable public static func *(lhs: AffineTransform, rhs: Self) -> Self { Self(lhs * rhs.endPoints.0, lhs * rhs.endPoints.1) }

}



// MARK: : KvNumericallyEquatable

extension KvSegment2 : KvNumericallyEquatable where Vertex : KvNumericallyEquatable {

    /// - Returns: A boolean value indicating whether the receiver and *rhs* are numerically equal.
    ///
    /// - Note: Segments having common end-points in different order are numerically equal.
    @inlinable
    public func isEqual(to rhs: Self) -> Bool {
        Math.isEqual(endPoints.0.coordinate, rhs.endPoints.0.coordinate)
        ? Math.isEqual(endPoints.1.coordinate, rhs.endPoints.1.coordinate)
        : (Math.isEqual(endPoints.0.coordinate, rhs.endPoints.1.coordinate)
           && Math.isEqual(endPoints.1.coordinate, rhs.endPoints.0.coordinate))
    }

    /// - Returns: A boolean value indicating whether the receiver and *rhs* are numerically equal.
    ///
    /// - Note: Segments having common end-points in different order are numerically equal.
    @inlinable
    public func isEqual<V>(to rhs: KvSegment2<V>) -> Bool
    where V : KvVertex2Protocol, V.Math == Vertex.Math
    {
        Math.isEqual(endPoints.0.coordinate, rhs.endPoints.0.coordinate)
        ? Math.isEqual(endPoints.1.coordinate, rhs.endPoints.1.coordinate)
        : (Math.isEqual(endPoints.0.coordinate, rhs.endPoints.1.coordinate)
           && Math.isEqual(endPoints.1.coordinate, rhs.endPoints.0.coordinate))
    }

}



// MARK: : Equatable

extension KvSegment2 : Equatable where Vertex : Equatable {

    /// - Returns: A boolean value indicating whether two instances have the same `.endPoints` properties.
    @inlinable static public func ==(lhs: Self, rhs: Self) -> Bool { lhs.endPoints == rhs.endPoints }

}



// MARK: : Hashable

extension KvSegment2 : Hashable where Vertex : Hashable {

    @inlinable
    public func hash(into hasher: inout Hasher) {
        hasher.combine(endPoints.0)
        hasher.combine(endPoints.1)
    }

}
